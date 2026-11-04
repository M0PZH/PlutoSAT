/*
 * nbiq: run the PL narrowband IQ channels (nbiq/rtl/nbiq_axi.sv) and stream
 * their samples to files, stdout or UDP.
 *
 *   nbiq [options] -c CH:OFFSET:RATE[:rx1] [-c ...]
 *
 *   -c CH:OFFSET:RATE[:rx1]  channel 0..3: OFFSET Hz from the LO (k/M suffix
 *                            allowed, may be negative), output RATE = fs/(2R),
 *                            R = 2..64 (12 MSPS: 1M, 857.142k, 750k, ...,
 *                            100k); source rx0 unless ":rx1"
 *   -o TARGET               "-" (stdout, one channel), a file name (a "%d"
 *                            in it becomes the channel), or udp:HOST:PORT
 *                            (channel c to PORT + c). Default: nbiq%d.iq
 *   -f s16|cf32             sample format (default s16: int16 I, Q)
 *   -g GAIN                 output LSBs per ADC LSB (default 8: 12-bit full
 *                            scale -> 16384)
 *   -F FS                   ADC sample rate (default: from ad9361-phy)
 *   -n SECONDS              stop after this long
 *   -q                      no status line
 *   -D R                    print the filter design for R and exit
 *
 * Map 0 of the "nbiq" UIO device: registers; map 1: the ring buffers (one
 * quarter per channel). The filter design is model/nbiq.py:design().
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define NCH 4
#define R_MIN 2
#define R_MAX 64
#define NPAIRS_MAX 127
#define CIC_N 5
#define FIR_BUDGET (0.9 * 100e6 / NCH)

#define R_ID        0x004
#define R_CTRL      0x008
#define R_IRQ_PER   0x00C
#define R_IRQ_ST    0x010
#define R_CH(c)     (0x100 + 0x40 * (c))
#define C_CTRL      0x00
#define C_FINC      0x04
#define C_CIC       0x08
#define C_FIR       0x0C
#define C_BASE      0x10
#define C_MASK      0x14
#define C_WPTR      0x18
#define C_T0LO      0x1C
#define C_T0HI      0x20
#define C_DROPS     0x24
#define R_COEF(c, i) (0x1000 + 0x400 * (c) + 4 * (i))

static volatile uint32_t *regs;
static volatile sig_atomic_t stop;

static inline void wr(unsigned off, uint32_t v) { regs[off / 4] = v; }
static inline uint32_t rd(unsigned off) { return regs[off / 4]; }
static void on_signal(int s) { (void)s; stop = 1; }

static void die(const char *msg)
{
	perror(msg);
	exit(1);
}

/* ---------------------------------------------------------------- design */
struct design {
	int R, cshift, npairs, oshift;
	double fout;
	int32_t h[2 * NPAIRS_MAX];
};

static double cic_resp(double f, int R)
{
	if (fabs(f) < 1e-12)
		return 1.0;
	return pow(fabs(sin(M_PI * f) / (R * sin(M_PI * f / R))), CIC_N);
}

static double bessel_i0(double x)
{
	double s = 1, t = 1;
	for (int k = 1; k < 50; k++) {
		t *= (x / (2 * k)) * (x / (2 * k));
		s += t;
	}
	return s;
}

/* CIC-compensated Kaiser-windowed lowpass, cutoff fout/2; see nbiq.py */
static int design(double fs, double rate, double gain, struct design *d)
{
	const double atten = 70;
	int R = (int)lround(fs / (2 * rate));
	if (R < R_MIN || R > R_MAX || fabs(fs / (2 * R) - rate) > 1e-6 * rate)
		return -1;
	d->R = R;
	d->fout = fs / (2 * R);
	int np = (int)floor(FIR_BUDGET / d->fout + 1e-9) - 1;
	np = np < 1 ? 1 : np > NPAIRS_MAX ? NPAIRS_MAX : np;
	d->npairs = np;
	int ntaps = 2 * np;
	double G = pow(R, CIC_N);
	int cs = (int)ceil(CIC_N * log2(R)) - 6;
	d->cshift = cs < 0 ? 0 : cs;

	double h[2 * NPAIRS_MAX], c = (ntaps - 1) / 2.0, sum = 0, hmax = 0;
	double beta = 0.1102 * (atten - 8.7);
	const int NG = 4096;
	for (int i = 0; i < ntaps; i++) {
		double n = i - c, a = 0;
		for (int g = 0; g < NG; g++) {
			double f = (g + 0.5) / NG * 0.25;
			a += cos(2 * M_PI * f * n) / cic_resp(f, R);
		}
		a *= 2 * 0.25 / NG;
		double r = ntaps > 1 ? 2.0 * i / (ntaps - 1) - 1 : 0;
		h[i] = a * bessel_i0(beta * sqrt(1 - r * r)) / bessel_i0(beta);
		sum += h[i];
	}
	for (int i = 0; i < ntaps; i++) {
		h[i] /= sum;
		if (fabs(h[i]) > hmax)
			hmax = fabs(h[i]);
	}
	double hsum = gain / (4 * G / pow(2, d->cshift));
	int os = (int)floor(log2((1 << 17) * 0.95 / (hmax * hsum)));
	d->oshift = os < 0 ? 0 : os > 47 ? 47 : os;
	for (int i = 0; i < ntaps; i++)
		d->h[i] = (int32_t)lround(h[i] * hsum * pow(2, d->oshift));
	return 0;
}

/* ---------------------------------------------------------------- sysfs */
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

static double adc_rate(void)
{
	char path[256], buf[64];
	int dev = find_by_name("/sys/bus/iio/devices", "iio:device", "ad9361-phy");
	if (dev < 0)
		return 0;
	snprintf(path, sizeof(path), "/sys/bus/iio/devices/iio:device%d/in_voltage_sampling_frequency", dev);
	if (read_file(path, buf, sizeof(buf)))
		return 0;
	return atof(buf);
}

/* ---------------------------------------------------------------- output */
struct chan {
	int on, src;
	double off, rate;
	struct design d;
	int fd;                       /* file / stdout, or the UDP socket */
	struct sockaddr_storage to;
	socklen_t tolen;
	uint32_t rp;                  /* bytes consumed */
	uint64_t total, lost;
	const uint8_t *ring;
	uint32_t size;
};

static int cf32;

static void emit(struct chan *c, const uint8_t *p, size_t n)
{
	static float fbuf[2 * 4096];
	while (n) {
		size_t k = n;
		const void *out = p;
		size_t olen;
		if (c->tolen && k > 1024)
			k = 1024;             /* 256 samples per datagram */
		if (cf32) {
			if (k > 4 * 2048)
				k = 4 * 2048;
			const int16_t *s = (const int16_t *)p;
			for (size_t i = 0; i < k / 2; i++)
				fbuf[i] = s[i] / 32768.0f;
			out = fbuf;
			olen = 2 * k;
		} else
			olen = k;
		if (c->tolen) {
			if (sendto(c->fd, out, olen, 0, (struct sockaddr *)&c->to, c->tolen) < 0 && errno != ENOBUFS)
				die("sendto");
		} else {
			const uint8_t *q = out;
			while (olen) {
				ssize_t w = write(c->fd, q, olen);
				if (w < 0) {
					if (errno == EINTR)
						continue;
					die("write");
				}
				q += w;
				olen -= w;
			}
		}
		p += k;
		n -= k;
	}
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

static void usage(void)
{
	fprintf(stderr,
		"usage: nbiq [-o TARGET] [-f s16|cf32] [-g GAIN] [-F FS] [-n SEC] [-q]\n"
		"            -c CH:OFFSET:RATE[:rx1] [-c ...]\n"
		"       nbiq -D R [-F FS]      print the filter design\n");
	exit(2);
}

int main(int argc, char **argv)
{
	struct chan ch[NCH];
	const char *target = "nbiq%d.iq";
	double fs = 0, gain = 8, secs = 0;
	int quiet = 0, dshow = 0, opt;

	memset(ch, 0, sizeof(ch));
	while ((opt = getopt(argc, argv, "c:o:f:g:F:n:qD:h")) != -1) {
		switch (opt) {
		case 'c': {
			char *s = strdup(optarg), *t[4] = {0};
			int n = 0;
			for (char *p = strtok(s, ":"); p && n < 4; p = strtok(NULL, ":"))
				t[n++] = p;
			if (n < 3)
				usage();
			int c = atoi(t[0]);
			if (c < 0 || c >= NCH)
				usage();
			ch[c].on = 1;
			ch[c].off = parse_hz(t[1]);
			ch[c].rate = parse_hz(t[2]);
			ch[c].src = n > 3 && !strcmp(t[3], "rx1");
			break;
		}
		case 'o': target = optarg; break;
		case 'f': cf32 = !strcmp(optarg, "cf32"); break;
		case 'g': gain = atof(optarg); break;
		case 'F': fs = parse_hz(optarg); break;
		case 'n': secs = atof(optarg); break;
		case 'q': quiet = 1; break;
		case 'D': dshow = atoi(optarg); break;
		default: usage();
		}
	}

	if (dshow) {
		struct design d;
		if (!fs)
			fs = 12e6;
		if (design(fs, fs / (2 * dshow), gain, &d))
			usage();
		printf("R %d cshift %d npairs %d oshift %d\n", d.R, d.cshift, d.npairs, d.oshift);
		for (int i = 0; i < 2 * d.npairs; i++)
			printf("%d\n", d.h[i]);
		return 0;
	}

	if (!fs)
		fs = adc_rate();
	if (!fs) {
		fprintf(stderr, "cannot read the ADC rate from ad9361-phy, use -F\n");
		return 1;
	}

	int nch = 0;
	double load = 0;
	for (int c = 0; c < NCH; c++) {
		if (!ch[c].on)
			continue;
		nch++;
		if (design(fs, ch[c].rate, gain, &ch[c].d)) {
			fprintf(stderr, "channel %d: rate %.0f is not fs/(2R), R = %d..%d (fs %.0f)\n",
				c, ch[c].rate, R_MIN, R_MAX, fs);
			return 1;
		}
		if (fabs(ch[c].off) > fs / 2) {
			fprintf(stderr, "channel %d: offset beyond +-fs/2\n", c);
			return 1;
		}
		load += ch[c].d.fout * (ch[c].d.npairs + 1);
	}
	if (!nch)
		usage();
	if (load > 0.95e8) {
		fprintf(stderr, "FIR load %.0f%% of the clock\n", load / 1e6);
		return 1;
	}

	int uio = find_by_name("/sys/class/uio", "uio", "nbiq");
	if (uio < 0) {
		fprintf(stderr, "no nbiq UIO device\n");
		return 1;
	}
	char path[64];
	snprintf(path, sizeof(path), "/dev/uio%d", uio);
	int ufd = open(path, O_RDWR);
	if (ufd < 0)
		die(path);
	size_t rsz, bsz;
	uint32_t bphys;
	regs = uio_map(uio, ufd, 0, NULL, &rsz);
	uint8_t *buf = uio_map(uio, ufd, 1, &bphys, &bsz);
	if (rd(R_ID) != 0x4E424951) {
		fprintf(stderr, "nbiq: bad ID %08x\n", rd(R_ID));
		return 1;
	}
	uint32_t size = 1;
	while (size * 2 <= bsz / NCH)
		size *= 2;

	/* outputs */
	for (int c = 0; c < NCH; c++) {
		if (!ch[c].on)
			continue;
		if (!strncmp(target, "udp:", 4)) {
			char host[128];
			const char *pc = strrchr(target + 4, ':');
			if (!pc)
				usage();
			snprintf(host, sizeof(host), "%.*s", (int)(pc - target - 4), target + 4);
			char port[16];
			snprintf(port, sizeof(port), "%d", atoi(pc + 1) + c);
			struct addrinfo hints = {.ai_socktype = SOCK_DGRAM}, *ai;
			if (getaddrinfo(host, port, &hints, &ai))
				die("getaddrinfo");
			ch[c].fd = socket(ai->ai_family, SOCK_DGRAM, 0);
			if (ch[c].fd < 0)
				die("socket");
			memcpy(&ch[c].to, ai->ai_addr, ai->ai_addrlen);
			ch[c].tolen = ai->ai_addrlen;
			freeaddrinfo(ai);
		} else if (!strcmp(target, "-")) {
			if (nch > 1) {
				fprintf(stderr, "stdout takes one channel\n");
				return 1;
			}
			ch[c].fd = 1;
		} else {
			char name[256];
			snprintf(name, sizeof(name), target, c);
			ch[c].fd = open(name, O_WRONLY | O_CREAT | O_TRUNC, 0644);
			if (ch[c].fd < 0)
				die(name);
		}
	}

	/* program and start */
	for (int c = 0; c < NCH; c++) {
		wr(R_CH(c) + C_CTRL, 0);
		if (!ch[c].on)
			continue;
		struct design *d = &ch[c].d;
		int64_t finc = llround(ch[c].off / fs * 4294967296.0);
		wr(R_CH(c) + C_FINC, (uint32_t)finc);
		wr(R_CH(c) + C_CIC, d->R | d->cshift << 8);
		wr(R_CH(c) + C_FIR, d->npairs | d->oshift << 8);
		wr(R_CH(c) + C_BASE, bphys + c * size);
		wr(R_CH(c) + C_MASK, size - 1);
		for (int i = 0; i < 2 * d->npairs; i++)
			wr(R_COEF(c, i), (uint32_t)d->h[i] & 0x3FFFF);
		ch[c].ring = buf + c * size;
		ch[c].size = size;
		if (!quiet)
			fprintf(stderr, "ch%d: rx%d %+.0f Hz, %.1f kSPS (R %d, %d taps)\n", c, ch[c].src,
				ch[c].off, d->fout / 1e3, d->R, 2 * d->npairs);
	}
	wr(R_IRQ_PER, 1000000);                    /* 10 ms */
	wr(R_IRQ_ST, 1);
	wr(R_CTRL, 1);
	for (int c = 0; c < NCH; c++)
		if (ch[c].on)
			wr(R_CH(c) + C_CTRL, 1 | ch[c].src << 1);

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, on_signal);

	struct timespec t0, tl, tn;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	tl = t0;
	uint64_t lastn[NCH] = {0};
	while (!stop) {
		uint32_t one = 1, cnt;
		if (write(ufd, &one, 4) != 4)
			die("uio irq enable");
		if (read(ufd, &cnt, 4) != 4) {
			if (errno == EINTR)
				continue;
			die("uio read");
		}
		wr(R_IRQ_ST, 1);
		for (int c = 0; c < NCH; c++) {
			struct chan *h = &ch[c];
			if (!h->on)
				continue;
			uint32_t wp = rd(R_CH(c) + C_WPTR);
			uint32_t n = wp - h->rp;
			if (n > h->size - 4096) {         /* overrun: skip to half a ring behind */
				uint32_t keep = h->size / 2;
				h->lost += (n - keep) / 4;
				h->rp = wp - keep;
				n = keep;
			}
			while (n) {
				uint32_t o = h->rp & (h->size - 1);
				uint32_t k = n < h->size - o ? n : h->size - o;
				emit(h, h->ring + o, k);
				h->rp += k;
				h->total += k / 4;
				n -= k;
			}
		}
		clock_gettime(CLOCK_MONOTONIC, &tn);
		double dt = (tn.tv_sec - tl.tv_sec) + (tn.tv_nsec - tl.tv_nsec) * 1e-9;
		if (dt >= 1) {
			if (!quiet) {
				for (int c = 0; c < NCH; c++)
					if (ch[c].on) {
						uint32_t dr = rd(R_CH(c) + C_DROPS);
						fprintf(stderr, "ch%d %7.1f kS/s lost %llu drop %u/%u  ", c,
							(ch[c].total - lastn[c]) / dt / 1e3,
							(unsigned long long)ch[c].lost, dr & 0xFFFF, dr >> 16);
						lastn[c] = ch[c].total;
					}
				fprintf(stderr, "\r");
			}
			tl = tn;
		}
		if (secs && (tn.tv_sec - t0.tv_sec) + (tn.tv_nsec - t0.tv_nsec) * 1e-9 >= secs)
			break;
	}
	for (int c = 0; c < NCH; c++)
		wr(R_CH(c) + C_CTRL, 0);
	wr(R_CTRL, 0);
	if (!quiet)
		fprintf(stderr, "\n");
	return 0;
}
