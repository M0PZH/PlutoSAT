/*
 * iqtx: transmit through the PL narrowband IQ transmitter
 * (iqtx/rtl/iqtx_axi.sv): a test tone, a two-tone signal or an IQ file, at an
 * offset from the TX LO. A board test tool; the station daemon has its own
 * driver (the interface is in iqtx/README.md).
 *
 *   iqtx [options]
 *
 *   -r RATE        input sample rate, 12 MHz / (5 R) for R = 8..400
 *                  (default 48k: R 50; 12k, 24k, 60k, 96k, 100k, 200k, ...)
 *   -o OFFSET      NCO offset from the LO in Hz (k/M suffix, may be negative)
 *   -t FREQ        a tone at FREQ Hz in the baseband (default: a tone at 0)
 *   -T F1,F2       two tones, each at half the amplitude
 *   -i FILE        play a file of int16 I/Q pairs at RATE ("-": stdin)
 *   -a AMP         tone amplitude, 0..1 of full scale (default 0.5)
 *   -g GAIN_DB     digital gain in dB (default 0; at most +12)
 *   -R RLOG        start/stop ramp of 2^RLOG input samples (default 8)
 *   -n SECONDS     stop after this long (default: at the end of the file,
 *                  or at Ctrl-C)
 *   -q             no status line
 *   -s             print the registers and exit
 *
 * It selects IQ TX as the DAC source (TX_SRC 1; this waits until the DVB-S2
 * output is silent), plays, stops (ramp down) and gives the DAC back to
 * DVB-S2 TX (TX_SRC 0). The TX LO, attenuation etc. are set elsewhere
 * (IIO, plutosat); like dvbs2-tx it makes the AD9361 core's DAC channel 0
 * take its data from the PL (DAC_DDS_SEL = 2, through debugfs
 * direct_reg_access). Map 0 of the "iqtx" UIO device: registers; map 1: the
 * ring buffer (all of it is used).
 */
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

/* registers (regs/iqtx.toml) */
#define R_VERSION       0x000
#define R_ID            0x004
#define R_CTRL          0x008
#define R_TX_SRC        0x00C
#define R_STATUS        0x010
#define R_RATE          0x014
#define R_GAIN          0x018
#define R_FINC          0x01C
#define R_RAMP          0x020
#define R_RING_BASE     0x024
#define R_RING_MASK     0x028
#define R_WPTR          0x02C
#define R_RPTR          0x030
#define R_LOWATER       0x034
#define R_IRQ_EN        0x038
#define R_IRQ_STATUS    0x03C
#define R_SAMPLES       0x040
#define R_UNDERRUNS     0x044
#define R_DAC_UNDERRUNS 0x048
#define R_INFO          0x04C
#define ID_IQTX         0x49515458u
#define ST_ACTIVE       0x03u
#define ST_BUSY         0x08u
#define SRC_DVBS2       0
#define SRC_IQ          1

#define FS 12e6

static volatile uint32_t *regs;
static volatile sig_atomic_t stop;

static inline void wr(unsigned off, uint32_t v) { regs[off / 4] = v; }
static inline uint32_t rd(unsigned off) { return regs[off / 4]; }
static void on_signal(int s) { (void)s; stop = 1; }

static void die(const char *msg)
{
	if (errno)
		perror(msg);
	else
		fprintf(stderr, "iqtx: %s\n", msg);
	exit(1);
}

/* ---------------------------------------------------------------- UIO */
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

/* /sys/<dir>/<prefix>N whose "name" file equals name: N, or -1 */
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

/* AD9361 core: DAC channel 0 (TX1 I/Q) takes its data from the PL input */
static void dac_select_pl(void)
{
	char path[256];
	int dds = find_by_name("/sys/bus/iio/devices", "iio:device", "cf-ad9361-dds-core-lpc");
	if (dds >= 0) {
		snprintf(path, sizeof(path), "/sys/kernel/debug/iio/iio:device%d/direct_reg_access", dds);
		if (!write_file(path, "0x418 0x2") && !write_file(path, "0x458 0x2"))
			return;
	}
	fprintf(stderr, "iqtx: warning: cannot select the DAC data source (cf-ad9361-dds-core-lpc, debugfs)\n");
}

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

/* ---------------------------------------------------------------- helpers */
static double now(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}

static double parse_hz(const char *s)
{
	char *e;
	double v = strtod(s, &e);
	if (*e == 'k' || *e == 'K')
		v *= 1e3;
	else if (*e == 'M')
		v *= 1e6;
	return v;
}

static int wait_status(uint32_t mask, uint32_t val, double timeout)
{
	double t0 = now();
	while ((rd(R_STATUS) & mask) != val) {
		if (now() - t0 > timeout)
			return -1;
		usleep(1000);
	}
	return 0;
}

static void print_regs(void)
{
	static const struct { unsigned off; const char *name; } r[] = {
		{ R_VERSION, "VERSION" }, { R_CTRL, "CTRL" }, { R_TX_SRC, "TX_SRC" },
		{ R_STATUS, "STATUS" }, { R_RATE, "RATE" }, { R_GAIN, "GAIN" },
		{ R_FINC, "FINC" }, { R_RAMP, "RAMP" }, { R_RING_BASE, "RING_BASE" },
		{ R_RING_MASK, "RING_MASK" }, { R_WPTR, "WPTR" }, { R_RPTR, "RPTR" },
		{ R_LOWATER, "LOWATER" }, { R_IRQ_STATUS, "IRQ_STATUS" },
		{ R_SAMPLES, "SAMPLES" }, { R_UNDERRUNS, "UNDERRUNS" },
		{ R_DAC_UNDERRUNS, "DAC_UNDERRUNS" }, { R_INFO, "INFO" },
	};
	for (size_t i = 0; i < sizeof(r) / sizeof(r[0]); i++)
		printf("%-14s 0x%08x\n", r[i].name, rd(r[i].off));
}

static void usage(void)
{
	fprintf(stderr,
		"usage: iqtx [-r RATE] [-o OFFSET] [-t FREQ | -T F1,F2 | -i FILE] [-a AMP]\n"
		"            [-g GAIN_DB] [-R RLOG] [-n SECONDS] [-q] | -s\n");
	exit(2);
}

/* ---------------------------------------------------------------- main */
int main(int argc, char **argv)
{
	double rate = 48e3, offset = 0, amp = 0.5, gain_db = 0, secs = 0;
	double f1 = 0, f2 = 0;
	int ntone = 1, rlog = 8, quiet = 0, status = 0, opt;
	const char *file = NULL;

	while ((opt = getopt(argc, argv, "r:o:t:T:i:a:g:R:n:qs")) != -1) {
		switch (opt) {
		case 'r': rate = parse_hz(optarg); break;
		case 'o': offset = parse_hz(optarg); break;
		case 't': f1 = parse_hz(optarg); ntone = 1; break;
		case 'T': {
			char *c = strchr(optarg, ',');
			if (!c)
				usage();
			*c = 0;
			f1 = parse_hz(optarg);
			f2 = parse_hz(c + 1);
			ntone = 2;
			break;
		}
		case 'i': file = optarg; ntone = 0; break;
		case 'a': amp = atof(optarg); break;
		case 'g': gain_db = atof(optarg); break;
		case 'R': rlog = atoi(optarg); break;
		case 'n': secs = atof(optarg); break;
		case 'q': quiet = 1; break;
		case 's': status = 1; break;
		default: usage();
		}
	}
	if (optind != argc)
		usage();

	int R = (int)lround(FS / 5 / rate);
	if (R < 8 || R > 400 || fabs(FS / 5 / R - rate) > 1e-6 * rate) {
		fprintf(stderr, "iqtx: rate %.3f Hz is not 12 MHz / (5 R), R = 8..400 (e.g. 48k: R 50, 12k: R 200)\n", rate);
		return 2;
	}
	if (rlog < 0 || rlog > 15 || amp < 0 || amp > 1 || gain_db > 12.04 || fabs(offset) > FS / 2)
		usage();
	uint32_t gain = (uint32_t)lround(16384 * pow(10, gain_db / 20));
	if (gain > 0xFFFF)
		gain = 0xFFFF;
	int32_t finc = (int32_t)llround(offset / FS * 4294967296.0);

	int uio = find_by_name("/sys/class/uio", "uio", "iqtx");
	if (uio < 0)
		die("no UIO device \"iqtx\" (device tree, uio_pdrv_genirq.of_id=generic-uio)");
	char path[64];
	snprintf(path, sizeof(path), "/dev/uio%d", uio);
	int ufd = open(path, O_RDWR);
	if (ufd < 0)
		die(path);
	size_t rsz, bsz;
	uint32_t bphys;
	regs = uio_map(uio, ufd, 0, NULL, &rsz);
	volatile uint32_t *ring = uio_map(uio, ufd, 1, &bphys, &bsz);
	if (rd(R_ID) != ID_IQTX) {
		fprintf(stderr, "iqtx: no IQ transmitter in the PL (ID 0x%08x)\n", rd(R_ID));
		return 1;
	}
	if (status) {
		print_regs();
		return 0;
	}
	if (bsz & (bsz - 1) || bsz < 4096 || bphys & (bsz - 1))
		die("ring buffer: size not a power of two or not aligned");
	uint32_t ring_words = bsz / 4;

	FILE *in = NULL;
	if (file) {
		in = strcmp(file, "-") ? fopen(file, "rb") : stdin;
		if (!in)
			die(file);
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	/* stop whatever runs, take the DAC */
	wr(R_CTRL, 0);
	if (wait_status(ST_BUSY, 0, 5))
		die("core does not stop");
	wr(R_TX_SRC, SRC_IQ);
	if (wait_status(ST_ACTIVE, SRC_IQ, 2)) {
		wr(R_TX_SRC, SRC_DVBS2);
		die("IQ TX does not become the DAC source: DVB-S2 TX still transmitting?");
	}
	dac_select_pl();
	wr(R_RATE, R);
	wr(R_GAIN, gain);
	wr(R_FINC, (uint32_t)finc);
	wr(R_RAMP, rlog);
	wr(R_RING_BASE, bphys);
	wr(R_RING_MASK, bsz - 1);
	wr(R_IRQ_EN, 0);
	wr(R_IRQ_STATUS, 7);

	/* RPTR = WPTR after a stop: continue from there */
	uint32_t wptr = rd(R_RPTR) & ~63u;
	wr(R_WPTR, wptr);
	double fs_in = FS / 5 / R;
	uint32_t target = (uint32_t)(0.25 * fs_in) * 4;       /* fill: 250 ms */
	if (target > bsz - 4096)
		target = bsz - 4096;
	target &= ~63u;
	double ph1 = 0, ph2 = 0, d1 = 2 * M_PI * f1 / fs_in, d2 = 2 * M_PI * f2 / fs_in;
	double a = amp * 32767 / (ntone == 2 ? 2 : 1);
	uint64_t total = secs > 0 ? (uint64_t)(secs * fs_in) : UINT64_MAX, made = 0;
	int eof = 0, started = 0;
	double t_stat = now();

	while (!stop && !(eof && (uint32_t)(rd(R_RPTR) - wptr) == 0)) {
		uint32_t fill = wptr - rd(R_RPTR);
		/* refill in blocks of 16 samples (64 bytes, the core's burst) */
		while (!eof && fill + 64 <= target) {
			uint32_t w = (wptr / 4) & (ring_words - 1);
			for (int k = 0; k < 16; k++) {
				int16_t si = 0, sq = 0;
				if (made >= total || eof) {
					eof = 1;
				} else if (in) {
					int16_t s[2];
					if (fread(s, sizeof(s), 1, in) == 1) {
						si = s[0];
						sq = s[1];
					} else
						eof = 1;
				} else {
					double x = a * cos(ph1) + (ntone == 2 ? a * cos(ph2) : 0);
					double y = a * sin(ph1) + (ntone == 2 ? a * sin(ph2) : 0);
					si = (int16_t)lround(x);
					sq = (int16_t)lround(y);
					ph1 = fmod(ph1 + d1, 2 * M_PI);
					ph2 = fmod(ph2 + d2, 2 * M_PI);
				}
				if (!eof)
					made++;
				ring[(w + k) & (ring_words - 1)] = (uint16_t)si | (uint32_t)(uint16_t)sq << 16;
			}
			wptr += 64;
			fill += 64;
		}
		__sync_synchronize();
		wr(R_WPTR, wptr);
		if (!started) {
			wr(R_CTRL, 1);
			started = 1;
		}
		usleep(10000);
		if (!quiet && now() - t_stat >= 1) {
			t_stat = now();
			fprintf(stderr, "\rplayed %u  fill %5.0f ms  underruns %u  ",
				rd(R_SAMPLES), (wptr - rd(R_RPTR)) / 4 / fs_in * 1e3, rd(R_UNDERRUNS));
		}
	}

	/* ramp down, flush, give the DAC back */
	wr(R_CTRL, 0);
	if (wait_status(ST_BUSY, 0, 5))
		fprintf(stderr, "iqtx: core does not stop\n");
	if (!quiet)
		fprintf(stderr, "\rplayed %u samples, %u underruns, %u DAC underruns\n",
			rd(R_SAMPLES), rd(R_UNDERRUNS), rd(R_DAC_UNDERRUNS));
	wr(R_TX_SRC, SRC_DVBS2);
	if (wait_status(ST_ACTIVE, SRC_DVBS2, 1))
		fprintf(stderr, "iqtx: DAC source still pending (STATUS 0x%02x)\n", rd(R_STATUS));
	return 0;
}
