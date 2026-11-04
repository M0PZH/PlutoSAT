/*
 * dvbs2-tx: feed the PL DVB-S2 transmitter (dvbs2/rtl) from user space.
 *
 *   TS (UDP or file/stdin) -> BBFRAMEs -> reserved DMA buffer -> axi_dmac
 *   -> dvbs2_tx core -> AD9363 channel 0
 *
 * Hardware access is through one UIO device ("dvbs2-tx") with three maps:
 *   map0  dvbs2_tx registers (see dvbs2/rtl/dvbs2_tx_axi.sv)
 *   map1  axi_dmac registers
 *   map2  reserved, uncached DMA buffer
 * The AD9363 is configured through the ADI IIO sysfs/debugfs attributes.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "bbframe.h"

/* ---------------------------------------------------------------- registers */
#define R_ID          0x04
#define R_CTRL        0x10
#define   CTRL_ENABLE   (1u << 0)
#define   CTRL_RESET    (1u << 1)
#define   CTRL_DACSEL   (1u << 2)
#define   CTRL_NCOBYP   (1u << 3)
#define R_SPS         0x14
#define R_GOLD_XA     0x18
#define R_GOLD_XB     0x1C
#define R_NCO_FINC    0x20
#define R_COEF_ADDR   0x24
#define R_COEF_DATA   0x28
#define R_DATA_FRAMES 0x30
#define R_DUMMY_FRAMES 0x34
#define R_LEN_ERR     0x38
#define R_HDR_ERR     0x3C
#define R_FIFO_LEVEL  0x40
#define R_DAC_UNDERRUN 0x44

#define DMAC_IRQ_MASK  0x080
#define DMAC_CONTROL   0x400
#define DMAC_SUBMIT    0x408
#define DMAC_FLAGS     0x40C
#define DMAC_SRC_ADDR  0x414
#define DMAC_X_LENGTH  0x418
#define DMAC_Y_LENGTH  0x41C

#define TPP 16
#define MAX_SPS 64
#define SLOT_BYTES 8192
#define NSLOTS 16

static volatile uint32_t *regs, *dmac;
static uint8_t *dmabuf;
static uint32_t dmabuf_phys;
static volatile sig_atomic_t stop;

static inline void wr(volatile uint32_t *b, unsigned off, uint32_t v) { b[off / 4] = v; }
static inline uint32_t rd(volatile uint32_t *b, unsigned off) { return b[off / 4]; }

static void on_signal(int s) { (void)s; stop = 1; }

static void die(const char *msg)
{
	perror(msg);
	exit(1);
}

/* ---------------------------------------------------------------- sysfs helpers */
static int read_file(const char *path, char *buf, size_t n)
{
	FILE *f = fopen(path, "r");
	if (!f)
		return -1;
	size_t r = fread(buf, 1, n - 1, f);
	fclose(f);
	buf[r] = 0;
	while (r && (buf[r - 1] == '\n' || buf[r - 1] == ' '))
		buf[--r] = 0;
	return 0;
}

static int write_file(const char *path, const char *val)
{
	FILE *f = fopen(path, "w");
	if (!f)
		return -1;
	int ok = fputs(val, f) >= 0;
	return (fclose(f) == 0 && ok) ? 0 : -1;
}

/* find /sys/<dir>/<prefix>N whose "name" file equals name; returns N or -1 */
static int find_by_name(const char *dir, const char *prefix, const char *name)
{
	DIR *d = opendir(dir);
	struct dirent *e;
	char path[512], buf[128];
	int found = -1;

	if (!d)
		return -1;
	while ((e = readdir(d))) {
		if (strncmp(e->d_name, prefix, strlen(prefix)))
			continue;
		snprintf(path, sizeof(path), "%s/%s/name", dir, e->d_name);
		if (!read_file(path, buf, sizeof(buf)) && !strcmp(buf, name)) {
			found = atoi(e->d_name + strlen(prefix));
			break;
		}
	}
	closedir(d);
	return found;
}

static void iio_set(int dev, const char *attr, const char *val)
{
	char path[256];
	snprintf(path, sizeof(path), "/sys/bus/iio/devices/iio:device%d/%s", dev, attr);
	if (write_file(path, val))
		fprintf(stderr, "warning: cannot set %s = %s\n", path, val);
}

/* ---------------------------------------------------------------- UIO */
static void *uio_map(int uio, int fd, int map, uint32_t *phys, size_t *size)
{
	char path[256], buf[64];

	snprintf(path, sizeof(path), "/sys/class/uio/uio%d/maps/map%d/size", uio, map);
	if (read_file(path, buf, sizeof(buf)))
		die(path);
	*size = strtoul(buf, NULL, 0);
	snprintf(path, sizeof(path), "/sys/class/uio/uio%d/maps/map%d/addr", uio, map);
	if (read_file(path, buf, sizeof(buf)))
		die(path);
	if (phys)
		*phys = strtoul(buf, NULL, 0);
	void *p = mmap(NULL, *size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
		       (off_t)map * sysconf(_SC_PAGESIZE));
	if (p == MAP_FAILED)
		die("mmap");
	return p;
}

/* ---------------------------------------------------------------- DSP setup */
/* Same formula and scaling as dvbs2/model/txmodel.py:rrc_coefs() */
static void rrc_coefs(int sps, double alpha, int32_t *c)
{
	const int n = sps * TPP;
	double h[MAX_SPS * TPP], worst = 0;

	for (int i = 0; i < n; i++) {
		double t = (i - (n - 1) / 2.0) / sps;
		if (fabs(t) < 1e-12)
			h[i] = 1 - alpha + 4 * alpha / M_PI;
		else if (fabs(fabs(4 * alpha * t) - 1) < 1e-9)
			h[i] = alpha / sqrt(2) * ((1 + 2 / M_PI) * sin(M_PI / (4 * alpha)) +
						  (1 - 2 / M_PI) * cos(M_PI / (4 * alpha)));
		else
			h[i] = (sin(M_PI * t * (1 - alpha)) + 4 * alpha * t * cos(M_PI * t * (1 + alpha))) /
			       (M_PI * t * (1 - (4 * alpha * t) * (4 * alpha * t)));
	}
	for (int p = 0; p < sps; p++) {
		double s = 0;
		for (int j = 0; j < TPP; j++)
			s += fabs(h[p + sps * j]);
		if (s > worst)
			worst = s;
	}
	double scale = 0.80 * 32767 * (1 << 17) / (worst * 23170);
	for (int i = 0; i < n; i++)
		c[i] = (int32_t)lround(h[i] * scale);
}

/* x LFSR of the PL scrambler advanced by n (Gold code n) */
static uint32_t gold_x(uint32_t n)
{
	uint32_t x = 1;
	while (n--) {
		uint32_t fb = (x ^ (x >> 7)) & 1;
		x = (x >> 1) | (fb << 17);
	}
	return x;
}

/* ---------------------------------------------------------------- TS input */
struct tsin {
	int fd, is_udp;
	uint8_t buf[65536];
	size_t len, pos;
	int eof;
};

static int ts_fill(struct tsin *in, int timeout_ms)
{
	struct pollfd pfd = { .fd = in->fd, .events = POLLIN };
	if (in->eof || poll(&pfd, 1, timeout_ms) <= 0)
		return 0;
	if (in->pos) {
		memmove(in->buf, in->buf + in->pos, in->len - in->pos);
		in->len -= in->pos;
		in->pos = 0;
	}
	size_t room = sizeof(in->buf) - in->len;
	if (room == 0)
		return 0;   /* full: let the framer drain it first */
	ssize_t r = in->is_udp ? recv(in->fd, in->buf + in->len, room, 0)
			       : read(in->fd, in->buf + in->len, room);
	if (r <= 0) {
		if (!in->is_udp)
			in->eof = 1;
		return 0;
	}
	in->len += r;
	return 1;
}

static int ts_next(void *ctx, uint8_t pkt[TS_LEN])
{
	struct tsin *in = ctx;
	/* resynchronise on the 0x47 sync byte */
	while (in->len - in->pos >= TS_LEN && in->buf[in->pos] != 0x47)
		in->pos++;
	if (in->len - in->pos < TS_LEN)
		return 0;
	memcpy(pkt, in->buf + in->pos, TS_LEN);
	in->pos += TS_LEN;
	return 1;
}

static size_t ts_avail(struct tsin *in) { return in->len - in->pos; }

/* ---------------------------------------------------------------- DMA */
static void dma_submit(unsigned slot, unsigned len)
{
	while (rd(dmac, DMAC_SUBMIT) & 1) {
		if (stop)
			return;
		usleep(100);
	}
	wr(dmac, DMAC_FLAGS, 0x2);                /* TLAST at end of transfer */
	wr(dmac, DMAC_SRC_ADDR, dmabuf_phys + slot * SLOT_BYTES);
	wr(dmac, DMAC_X_LENGTH, len - 1);
	wr(dmac, DMAC_Y_LENGTH, 0);
	wr(dmac, DMAC_SUBMIT, 1);
}

/* ---------------------------------------------------------------- main */
static const char *modcods[11] = {
	"qpsk1/4", "qpsk1/3", "qpsk2/5", "qpsk1/2", "qpsk3/5", "qpsk2/3",
	"qpsk3/4", "qpsk4/5", "qpsk5/6", "qpsk8/9", "qpsk9/10",
};

static void usage(const char *p)
{
	fprintf(stderr,
		"usage: %s [options]\n"
		"  -s, --sym-rate SR      symbol rate in S/s (fs/SR must be an integer, default 500000)\n"
		"  -F, --fs FS            AD9363 sample rate (default 12000000)\n"
		"  -m, --modcod MC        qpsk1/4 .. qpsk9/10 (default qpsk1/2)\n"
		"  -p, --pilots           pilots on\n"
		"  -r, --rolloff R        0.35, 0.25 or 0.20 (default 0.35)\n"
		"  -g, --gold N           PL scrambling Gold code (default 0)\n"
		"  -o, --offset HZ        digital frequency offset (NCO), default 0\n"
		"  -f, --freq HZ          TX LO frequency (default: leave unchanged)\n"
		"  -a, --atten DB         TX attenuation in dB (default: leave unchanged)\n"
		"  -u, --udp PORT         read TS from UDP port (default 1234)\n"
		"  -i, --input FILE       read TS from FILE ('-' = stdin) instead of UDP\n"
		"  -t, --flush-ms MS      send a partly filled BBFRAME after MS without data (default 100)\n"
		"  -n, --no-iio           do not touch the AD9363 / DAC data source\n",
		p);
	exit(2);
}

int main(int argc, char **argv)
{
	double sr = 500e3, fs = 12e6, rolloff = 0.35, offset = 0, freq = 0, atten = -1;
	int rate_idx = 3, pilots = 0, port = 1234, flush_ms = 100, no_iio = 0;
	uint32_t gold = 0;
	const char *input = NULL;

	static const struct option lo[] = {
		{ "sym-rate", 1, 0, 's' }, { "fs", 1, 0, 'F' }, { "modcod", 1, 0, 'm' },
		{ "pilots", 0, 0, 'p' }, { "rolloff", 1, 0, 'r' }, { "gold", 1, 0, 'g' },
		{ "offset", 1, 0, 'o' }, { "freq", 1, 0, 'f' }, { "atten", 1, 0, 'a' },
		{ "udp", 1, 0, 'u' }, { "input", 1, 0, 'i' }, { "flush-ms", 1, 0, 't' },
		{ "no-iio", 0, 0, 'n' }, { 0, 0, 0, 0 },
	};
	for (int c; (c = getopt_long(argc, argv, "s:F:m:pr:g:o:f:a:u:i:t:n", lo, NULL)) != -1;) {
		switch (c) {
		case 's': sr = atof(optarg); break;
		case 'F': fs = atof(optarg); break;
		case 'm':
			rate_idx = -1;
			for (int k = 0; k < 11; k++)
				if (!strcmp(optarg, modcods[k]))
					rate_idx = k;
			if (rate_idx < 0)
				usage(argv[0]);
			break;
		case 'p': pilots = 1; break;
		case 'r': rolloff = atof(optarg); break;
		case 'g': gold = strtoul(optarg, NULL, 0); break;
		case 'o': offset = atof(optarg); break;
		case 'f': freq = atof(optarg); break;
		case 'a': atten = atof(optarg); break;
		case 'u': port = atoi(optarg); break;
		case 'i': input = optarg; break;
		case 't': flush_ms = atoi(optarg); break;
		case 'n': no_iio = 1; break;
		default: usage(argv[0]);
		}
	}
	int sps = (int)lround(fs / sr);
	if (sps < 2 || sps > MAX_SPS || fabs(sps * sr - fs) > 0.5) {
		fprintf(stderr, "fs/sym-rate must be an integer in 2..%d\n", MAX_SPS);
		return 2;
	}
	if (gold >= 262143) {
		fprintf(stderr, "Gold code must be < 262143\n");
		return 2;
	}

	/* ---- hardware */
	int uio = find_by_name("/sys/class/uio", "uio", "dvbs2-tx");
	if (uio < 0) {
		fprintf(stderr, "no UIO device named dvbs2-tx (wrong bitstream/device tree?)\n");
		return 1;
	}
	char path[64];
	snprintf(path, sizeof(path), "/dev/uio%d", uio);
	int ufd = open(path, O_RDWR | O_SYNC);
	if (ufd < 0)
		die(path);
	size_t sz;
	regs = uio_map(uio, ufd, 0, NULL, &sz);
	dmac = uio_map(uio, ufd, 1, NULL, &sz);
	dmabuf = uio_map(uio, ufd, 2, &dmabuf_phys, &sz);
	if (sz < NSLOTS * SLOT_BYTES) {
		fprintf(stderr, "DMA buffer too small (%zu)\n", sz);
		return 1;
	}
	if (rd(regs, R_ID) != 0x44565332) {
		fprintf(stderr, "dvbs2-tx core not found (ID %08x)\n", rd(regs, R_ID));
		return 1;
	}

	if (!no_iio) {
		int phy = find_by_name("/sys/bus/iio/devices", "iio:device", "ad9361-phy");
		int dds = find_by_name("/sys/bus/iio/devices", "iio:device", "cf-ad9361-dds-core-lpc");
		char v[64];
		if (phy < 0 || dds < 0) {
			fprintf(stderr, "AD9363 IIO devices not found\n");
			return 1;
		}
		snprintf(v, sizeof(v), "%.0f", fs);
		iio_set(phy, "out_voltage_sampling_frequency", v);
		if (freq > 0) {
			snprintf(v, sizeof(v), "%.0f", freq);
			iio_set(phy, "out_altvoltage1_TX_LO_frequency", v);
		}
		if (atten >= 0) {
			snprintf(v, sizeof(v), "%.2f", -atten);
			iio_set(phy, "out_voltage0_hardwaregain", v);
		}
		/* DAC channel 0 (TX1 I/Q) takes its data from the PL input (DAC_DDS_SEL = 2) */
		snprintf(path, sizeof(path), "/sys/kernel/debug/iio/iio:device%d/direct_reg_access", dds);
		if (write_file(path, "0x418 0x2") || write_file(path, "0x458 0x2"))
			fprintf(stderr, "warning: cannot select DAC data source via %s\n", path);
	}

	int32_t coefs[MAX_SPS * TPP];
	rrc_coefs(sps, rolloff, coefs);
	wr(regs, R_CTRL, CTRL_RESET);
	usleep(10);
	wr(regs, R_SPS, sps);
	wr(regs, R_GOLD_XA, gold_x(gold));
	wr(regs, R_GOLD_XB, gold_x(gold + 131072));
	wr(regs, R_NCO_FINC, (uint32_t)(int64_t)llround(offset / fs * 4294967296.0));
	wr(regs, R_COEF_ADDR, 0);
	for (int p = 0; p < sps; p++)
		for (int j = 0; j < TPP; j++)
			wr(regs, R_COEF_DATA, (uint32_t)coefs[p + sps * j] & 0x3FFFF);
	uint32_t ctrl = CTRL_ENABLE | CTRL_DACSEL | (offset == 0 ? CTRL_NCOBYP : 0);
	wr(regs, R_CTRL, ctrl);

	wr(dmac, DMAC_CONTROL, 0);
	wr(dmac, DMAC_IRQ_MASK, 0x3);
	wr(dmac, DMAC_CONTROL, 1);

	/* ---- input */
	struct tsin *in = calloc(1, sizeof(*in));
	if (input) {
		in->fd = strcmp(input, "-") ? open(input, O_RDONLY) : 0;
		if (in->fd < 0)
			die(input);
	} else {
		struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(port),
					 .sin_addr.s_addr = htonl(INADDR_ANY) };
		int rcvbuf = 4 << 20;
		in->fd = socket(AF_INET, SOCK_DGRAM, 0);
		in->is_udp = 1;
		setsockopt(in->fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
		if (in->fd < 0 || bind(in->fd, (struct sockaddr *)&a, sizeof(a)))
			die("udp");
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	fprintf(stderr, "dvbs2-tx: %s%s, %.0f S/s (sps %d), roll-off %.2f, Gold %u, offset %.0f Hz\n",
		modcods[rate_idx], pilots ? " pilots" : "", sr, sps, rolloff, gold, offset);

	struct bbframer bf;
	bbframer_init(&bf, rate_idx, rolloff);
	const unsigned frame_bytes = bf.kbch / 8;
	const unsigned df_bytes = frame_bytes - BBHEADER_BYTES;
	unsigned slot = 0, frames = 0;
	struct timespec last_data, now;
	clock_gettime(CLOCK_MONOTONIC, &last_data);

	while (!stop) {
		if (ts_fill(in, 20))
			clock_gettime(CLOCK_MONOTONIC, &last_data);
		clock_gettime(CLOCK_MONOTONIC, &now);
		long idle_ms = (now.tv_sec - last_data.tv_sec) * 1000 +
			       (now.tv_nsec - last_data.tv_nsec) / 1000000;
		size_t pending = ts_avail(in) + (bf.have_pkt ? TS_LEN - bf.pkt_pos : 0);

		/* a full frame's worth of data, or a partial frame after flush_ms */
		while (!stop && (pending >= df_bytes || (pending > 0 && idle_ms >= flush_ms))) {
			/* built in normal memory, then copied with aligned words: the
			 * DMA buffer is mapped uncached (device memory), which takes
			 * no unaligned accesses */
			static uint64_t pkt[SLOT_BYTES / 8];
			uint8_t *buf = (uint8_t *)pkt;
			volatile uint64_t *dst = (volatile uint64_t *)(dmabuf + slot * SLOT_BYTES);
			uint8_t frame[BBFRAME_MAX_BYTES];
			if (!bbframer_build(&bf, ts_next, in, frame))
				break;
			uint32_t hdr = 0xB5000000u | (pilots << 4) | rate_idx;
			unsigned len = (4 + frame_bytes + 7) & ~7u;     /* 64-bit DMA beats */
			memset(buf, 0, len);
			memcpy(buf, &hdr, 4);
			memcpy(buf + 4, frame, frame_bytes);
			for (unsigned i = 0; i < len / 8; i++)
				dst[i] = pkt[i];
			dma_submit(slot, len);
			slot = (slot + 1) % NSLOTS;
			frames++;
			pending = ts_avail(in) + (bf.have_pkt ? TS_LEN - bf.pkt_pos : 0);
			if (pending < df_bytes)
				break;
		}
		if (in->eof && ts_avail(in) < TS_LEN && !bf.have_pkt)
			break;
	}

	/* let queued frames go out, then report */
	if (in->eof)
		sleep(1);
	fprintf(stderr, "dvbs2-tx: %u BBFRAMEs queued; core: %u data / %u dummy PLFRAMEs, "
		"%u length / %u header errors, %u DAC underruns\n", frames,
		rd(regs, R_DATA_FRAMES), rd(regs, R_DUMMY_FRAMES), rd(regs, R_LEN_ERR),
		rd(regs, R_HDR_ERR), rd(regs, R_DAC_UNDERRUN));
	wr(regs, R_CTRL, 0);   /* stop framing, give DAC channel 0 back */
	return 0;
}
