/*
 * rx-awgn: the PL self-test noise source on the AD9363 rx0 samples
 * (awgn/rtl/rx_awgn_axi.sv), for loopback tests of the receivers.
 *
 *   rx-awgn                      show the state and the measured powers
 *   rx-awgn -x                   off (samples pass unchanged)
 *   rx-awgn -s SIGMA [-g GAIN]   noise sigma per component in ADC LSB
 *   rx-awgn -e ESN0 -r SYMRATE [-F FS] [-T DBFS]
 *                                noise for this Es/N0 (dB): measures the signal
 *                                power first (noise off), then sets the signal
 *                                gain and the noise so that signal + noise sit
 *                                at -T dBFS (default -12, full scale: a complex
 *                                amplitude of 2048), like the AD9363 AGC would
 *   -g GAIN                      rx0 signal gain (linear, 0 .. 3.99, default 1;
 *                                with -e: fixed, -T ignored)
 *   -q                           print only the resulting Es/N0 (or sigma)
 *
 * Es/N0 = P * fs / (2 sigma^2 * Rs), P the signal power (I^2 + Q^2) at the
 * noise source: with RF loopback the receiver's own noise is not counted.
 */
#define _GNU_SOURCE
#include <dirent.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define R_ID      0x04
#define R_CTRL    0x08
#define R_GAIN    0x0C
#define R_NSCALE  0x10
#define R_P_IN    0x14
#define R_P_OUT   0x18
#define R_P_COUNT 0x1C

#define SIGMA_G   295.6010825419961     /* generator sigma (16 bytes summed) */
#define NSCALE_MAX 131071

static volatile uint32_t *regs;
static inline void wr(unsigned off, uint32_t v) { regs[off / 4] = v; }
static inline uint32_t rd(unsigned off) { return regs[off / 4]; }

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

static double adc_rate(void)
{
	char path[256], buf[64];
	int dev = find_by_name("/sys/bus/iio/devices", "iio:device", "ad9361-phy");
	if (dev < 0)
		return 0;
	snprintf(path, sizeof(path), "/sys/bus/iio/devices/iio:device%d/in_voltage_sampling_frequency", dev);
	return read_file(path, buf, sizeof(buf)) ? 0 : atof(buf);
}

/* mean of n fresh meter windows (2^16 samples each) */
static void measure(int n, double *pin, double *pout)
{
	double a = 0, b = 0;
	uint32_t c = rd(R_P_COUNT);
	/* the window running now may straddle a change: skip it */
	while (rd(R_P_COUNT) - c < 2)
		usleep(1000);
	for (int k = 0; k < n; k++) {
		c = rd(R_P_COUNT);
		while (rd(R_P_COUNT) == c)
			usleep(1000);
		a += rd(R_P_IN);
		b += rd(R_P_OUT);
	}
	*pin = a / n;
	*pout = b / n;
}

static void usage(void)
{
	fprintf(stderr,
		"usage: rx-awgn [-x] [-s SIGMA | -e ESN0 -r SYMRATE [-T DBFS]] [-g GAIN] [-F FS] [-q]\n");
	exit(2);
}

int main(int argc, char **argv)
{
	double sigma = -1, esn0 = NAN, rs = 0, fs = 0, gain = 1, tdbfs = -12;
	int off = 0, quiet = 0, gain_set = 0, opt;

	while ((opt = getopt(argc, argv, "xs:e:r:g:F:T:qh")) != -1) {
		switch (opt) {
		case 'x': off = 1; break;
		case 's': sigma = atof(optarg); break;
		case 'e': esn0 = atof(optarg); break;
		case 'r': rs = atof(optarg); break;
		case 'g': gain = atof(optarg); gain_set = 1; break;
		case 'F': fs = atof(optarg); break;
		case 'T': tdbfs = atof(optarg); break;
		case 'q': quiet = 1; break;
		default: usage();
		}
	}
	if (!isnan(esn0) && rs <= 0)
		usage();
	if (gain < 0 || gain >= 4)
		usage();

	int uio = find_by_name("/sys/class/uio", "uio", "rx-awgn");
	if (uio < 0) {
		fprintf(stderr, "no rx-awgn UIO device\n");
		return 1;
	}
	char path[64];
	snprintf(path, sizeof(path), "/dev/uio%d", uio);
	int fd = open(path, O_RDWR | O_SYNC);
	if (fd < 0) {
		perror(path);
		return 1;
	}
	regs = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (regs == MAP_FAILED) {
		perror("mmap");
		return 1;
	}
	if (rd(R_ID) != 0x4157474E) {
		fprintf(stderr, "rx-awgn: bad ID %08x\n", rd(R_ID));
		return 1;
	}

	if (off) {
		wr(R_CTRL, 0);
		wr(R_GAIN, 16384);
		wr(R_NSCALE, 0);
		return 0;
	}

	if (!isnan(esn0)) {
		if (!fs)
			fs = adc_rate();
		if (!fs) {
			fprintf(stderr, "cannot read the ADC rate, use -F\n");
			return 1;
		}
		/* signal power without noise, at unity gain */
		wr(R_CTRL, 0);
		double pin, pout;
		measure(8, &pin, &pout);
		if (pin < 1) {
			fprintf(stderr, "no signal on rx0 (P = %.2f)\n", pin);
			return 1;
		}
		/* noise / signal power ratio k; total g^2 P (1 + k) = target */
		double k = fs / (rs * pow(10, esn0 / 10));
		if (!gain_set) {
			double pt = 2048.0 * 2048.0 * pow(10, tdbfs / 10);
			gain = sqrt(pt / (pin * (1 + k)));
			if (gain > 3.99)
				gain = 3.99;
		}
		sigma = sqrt(pin * gain * gain * k / 2);
	}

	if (sigma >= 0) {
		long ns = lround(sigma / SIGMA_G * 16384);
		if (ns > NSCALE_MAX) {
			fprintf(stderr, "sigma %.1f too large (max %.1f at this gain); lower -g\n",
				sigma, NSCALE_MAX * SIGMA_G / 16384);
			return 1;
		}
		wr(R_GAIN, (uint32_t)lround(gain * 16384));
		wr(R_NSCALE, (uint32_t)ns);
		wr(R_CTRL, 1);
	}

	/* report */
	int en = rd(R_CTRL) & 1;
	double g = rd(R_GAIN) / 16384.0, sg = rd(R_NSCALE) * SIGMA_G / 16384;
	double pin, pout;
	measure(4, &pin, &pout);
	if (quiet) {
		if (!isnan(esn0))
			printf("%.2f\n", 10 * log10(pin * g * g * fs / (2 * sg * sg * rs)));
		else
			printf("%.2f\n", sg);
		return 0;
	}
	printf("noise %s, gain %.4f, sigma %.2f LSB per component\n", en ? "on" : "off", g, en ? sg : 0.0);
	printf("rx0 power: in %.1f, out %.1f (expected %.1f)\n", pin, pout,
	       en ? pin * g * g + 2 * sg * sg : pin);
	if (en && rs > 0 && fs > 0)
		printf("Es/N0 %.2f dB at %.0f S/s, fs %.0f\n",
		       10 * log10(pin * g * g * fs / (2 * sg * sg * rs)), rs, fs);
	return 0;
}
