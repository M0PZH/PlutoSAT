/*
 * dvbs2-rx: run the PL DVB-S2 receiver (dvbs2/rtl/dvbs2_rx_axi.sv) from
 * user space.
 *
 *   AD9363 channel 0 -> DDC -> demodulator -> LDPC -> axi_dmac (one packet
 *   per BBFRAME) -> reserved buffer -> BBFRAME de-adaptation -> TS (UDP/file)
 *
 * Hardware access is through one UIO device ("dvbs2-rx") with three maps:
 *   map0  dvbs2_rx registers
 *   map1  axi_dmac registers (stream -> memory)
 *   map2  reserved, uncached DMA buffer
 * The AD9363 is configured through the ADI IIO sysfs attributes.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "bbdeframe.h"

/* ---------------------------------------------------------------- registers */
#define R_VERSION       0x00
#define R_ID            0x04
#define R_CTRL          0x10
#define   CTRL_ENABLE     (1u << 0)
#define   CTRL_RESET      (1u << 1)
#define R_MODCOD        0x14
#define R_GOLD_XA       0x18
#define R_GOLD_XB       0x1C
#define R_DDC_FINC      0x20
#define R_DDC_CFG       0x24
#define R_DDC_CADDR     0x28
#define R_DDC_CDATA     0x2C
#define R_RRC_CADDR     0x30
#define R_RRC_CDATA     0x34
#define R_TIMING_GAINS  0x38
#define R_FS_THRESH     0x3C
#define R_PLL_KP        0x40
#define R_PLL_KI        0x44
#define R_LDPC_MAX_IT   0x48
#define R_DDC_CIC       0x4C
#define R_IN_OVF        0x50
#define R_DDC_DROPPED   0x54
#define R_AGC_GAIN      0x58
#define R_FS_STATE      0x60
#define R_FS_FOUND      0x64
#define R_FS_FLYWHEEL   0x68
#define R_FS_LOST       0x6C
#define R_FREQ          0x70
#define R_AMP           0x74
#define R_ENERGY        0x78
#define R_FRAMES        0x80
#define R_LDPC_FRAMES   0x84
#define R_LDPC_FAILED   0x88
#define R_PKT_DROPPED   0x8C
#define R_CARRIER_ACQ   0x94
#define R_CARRIER_RETYPE 0x98
#define R_BCH           0x9C
#define R_SNAP_CTRL     0xA0
#define R_SNAP_ADDR     0xA4
#define R_SNAP_DATA     0xA8
#define R_RAW_ERR       0xAC
#define R_RAW_BITS      0xB0
#define R_BCH_BITERR    0xB4
#define R_PLS_LAST      0xB8     /* PLS decoder (core >= VERSION_PLS) */
#define R_PLS_DATA      0xBC
#define R_PLS_FREQ      0xC0
#define R_PLS_BEST      0xC4
#define R_PLS_SECOND    0xC8
#define R_PLS_DIFF      0xCC
#define R_PLS_SEEN      0xD0
#define R_PLS_CFG       0xD4
#define R_PLS_SKIPPED   0xD8
#define PLS_CONF        (1u << 12)
#define PLS_CARRIER     (1u << 13)
#define PLS_VALID       (1u << 15)

#define DMAC_IRQ_MASK   0x080
#define DMAC_CONTROL    0x400
#define DMAC_TRANSFER_ID 0x404
#define DMAC_SUBMIT     0x408
#define DMAC_FLAGS      0x40C
#define DMAC_DEST_ADDR  0x410
#define DMAC_X_LENGTH   0x418
#define DMAC_Y_LENGTH   0x41C
#define DMAC_DONE       0x428

/* defaults of rtl/gen/dvbs2_rx_pkg.sv (model/rxfixed.py) */
#define TIMING_KP 1394          /* loop bandwidth 0.002 */
#define TIMING_KI 952
#define PLL_KP 113624
#define PLL_KI 77567
#define FS_THRESH 324444

#define PKT_BYTES 8192            /* one DMA packet per BBFRAME (PKT_WORDS * 8) */
#define NSLOTS 64
#define NQUEUE 4                  /* axi_dmac transfer IDs */
#define RRC_NPH 32
#define RRC_NT 25
#define DDC_TPP 12                /* FIR alone: taps per decimation step */
#define DDC_NMAX 512
#define DDC_FIR_ONLY_MAX 32       /* model/rxfixed.py ddc_split() */
#define DDC_CIC_D_GOOD 8
#define DDC_CIC_DMIN 4
#define DDC_CIC_TPP 16            /* FIR after the CIC: taps per decimation step, Kaiser beta */
#define DDC_CIC_BETA 8.0
#define CIC_N 3
#define CIC_RMAX 32
#define DEC_MAX (CIC_RMAX * DDC_FIR_ONLY_MAX)
#define CORE_CLK 100e6           /* sys_cpu_clk */
#define VERSION_CIC 0x00010100   /* first core version with the CIC (DDC_CIC) */
#define VERSION_PLS 0x00010200   /* first core version with the PLS decoder */

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

static int write_file(const char *path, const char *val)
{
	FILE *f = fopen(path, "w");
	if (!f)
		return -1;
	int ok = fputs(val, f) >= 0;
	return (fclose(f) == 0 && ok) ? 0 : -1;
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

static void iio_set(int dev, const char *attr, const char *val)
{
	char path[256];
	snprintf(path, sizeof(path), "/sys/bus/iio/devices/iio:device%d/%s", dev, attr);
	if (write_file(path, val))
		fprintf(stderr, "warning: cannot set %s = %s\n", path, val);
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

/* ---------------------------------------------------------------- filters */
/* RRC matched filter at 2 samples/symbol, 32 fractional-delay phases of 25
 * taps; same as model/rxfixed.py:timing_coefs() (rxmodel.rrc_bank). */
static void rrc_bank(double a, int32_t c[RRC_NPH][32])
{
	const int n = 2 * RRC_NPH * 12 + 1;
	double h[2 * RRC_NPH * 12 + 1], e = 0;

	for (int i = 0; i < n; i++) {
		double t = (i - (n - 1) / 2.0) / (2 * RRC_NPH);
		if (fabs(t) < 1e-12)
			h[i] = 1 - a + 4 * a / M_PI;
		else if (fabs(fabs(4 * a * t) - 1) < 1e-9)
			h[i] = a / sqrt(2) * ((1 + 2 / M_PI) * sin(M_PI / (4 * a)) +
					      (1 - 2 / M_PI) * cos(M_PI / (4 * a)));
		else
			h[i] = (sin(M_PI * t * (1 - a)) + 4 * a * t * cos(M_PI * t * (1 + a))) /
			       (M_PI * t * (1 - (4 * a * t) * (4 * a * t)));
	}
	for (int i = 0; i < n; i += RRC_NPH)
		e += h[i] * h[i];
	e = sqrt(e);
	for (int k = 0; k < RRC_NPH; k++)
		for (int j = 0; j < 32; j++) {
			int i = k + RRC_NPH * j;
			c[k][j] = (j < RRC_NT && i < n) ? (int32_t)nearbyint(h[i] / e * 65536.0) : 0;
		}
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

/* DDC lowpass, same as model/rxfixed.py:ddc_coefs(): Kaiser-windowed sinc */
static int ddc_coefs(int dec, int32_t *c)
{
	int n = dec > 1 ? dec * DDC_TPP : 2;
	double h[DDC_NMAX], s = 0;

	if (dec == 1) {
		c[0] = 65536;
		c[1] = 0;
		return 2;
	}
	double fc = 1.0 / (2 * dec), beta = 5.65;
	for (int i = 0; i < n; i++) {
		double k = i - (n - 1) / 2.0;
		double x = 2.0 * i / (n - 1) - 1;
		double w = bessel_i0(beta * sqrt(1 - x * x)) / bessel_i0(beta);
		double sinc = k == 0 ? 1 : sin(M_PI * 2 * fc * k) / (M_PI * 2 * fc * k);
		h[i] = 2 * fc * sinc * w;
		s += h[i];
	}
	for (int i = 0; i < n; i++)
		c[i] = (int32_t)nearbyint(h[i] / s * 65536.0);
	return n;
}

/* (R, D) for a total decimation, same as model/rxfixed.py:ddc_split():
 * the CIC with the smallest R (largest FIR decimation D <= 32) if D >= 8, or
 * above 32 if D >= 4; else the FIR alone up to 32. -1: not supported. */
static int ddc_split(int dec, int *R, int *D)
{
	int r = 0;
	for (int k = 2; k <= CIC_RMAX && !r; k++)
		if (dec % k == 0 && dec / k <= DDC_FIR_ONLY_MAX)
			r = k;
	int d = r ? dec / r : 0;
	if (d >= DDC_CIC_D_GOOD || (dec > DDC_FIR_ONLY_MAX && d >= DDC_CIC_DMIN)) {
		*R = r;
		*D = d;
		return 0;
	}
	if (dec >= 1 && dec <= DDC_FIR_ONLY_MAX) {
		*R = 1;
		*D = dec;
		return 0;
	}
	return -1;
}

static int cic_shift_for(int R)
{
	unsigned v = (unsigned)(R * R * R) - 1;
	int bits = 0;
	while (v) {
		bits++;
		v >>= 1;
	}
	return bits > 5 ? bits - 5 : 0;
}

/* FIR after the CIC, same as model/rxfixed.py:ddc_cic_fir(): Kaiser-windowed
 * lowpass, cutoff 1/(2D), ideal response 1 + c f^2 (CIC droop compensation),
 * normalised to sum 1. Returns the number of taps. */
static int ddc_cic_fir(int R, int D, double *h)
{
	int n = D * DDC_CIC_TPP;
	double fc = 1.0 / (2 * D), c = CIC_N * M_PI * M_PI * (1 - 1.0 / ((double)R * R)) / 6, s = 0;

	for (int i = 0; i < n; i++) {
		double k = i - (n - 1) / 2.0, w = 2 * M_PI * k, g, sinc;
		if (k == 0) {
			g = 2 * fc * fc * fc / 3;
			sinc = 2 * fc;
		} else {
			g = 2 * (fc * fc * sin(w * fc) / w + 2 * fc * cos(w * fc) / (w * w) -
				 2 * sin(w * fc) / (w * w * w));
			sinc = sin(w * fc) / (M_PI * k);
		}
		double x = 2.0 * i / (n - 1) - 1;
		double win = bessel_i0(DDC_CIC_BETA * sqrt(1 - x * x)) / bessel_i0(DDC_CIC_BETA);
		h[i] = (sinc + c * g) * win;
		s += h[i];
	}
	for (int i = 0; i < n; i++)
		h[i] /= s;
	return n;
}

/* DDC configuration for a total decimation, same as model/rxfixed.py:ddc_config() */
struct ddc_cfg {
	int R, D, cic_shift, ntaps, shift;
	int32_t c[DDC_NMAX];
};

static int ddc_config(int dec, int gain_log2, struct ddc_cfg *cfg)
{
	if (ddc_split(dec, &cfg->R, &cfg->D))
		return -1;
	if (cfg->R == 1) {
		cfg->cic_shift = 0;
		cfg->ntaps = ddc_coefs(cfg->D, cfg->c);
		cfg->shift = 16 - gain_log2;
		return 0;
	}
	double h[DDC_NMAX], mx = 0;
	int R = cfg->R;
	cfg->cic_shift = cic_shift_for(R);
	double scale = (double)(1 << gain_log2) * (1 << cfg->cic_shift) / ((double)R * R * R);
	cfg->ntaps = ddc_cic_fir(R, cfg->D, h);
	for (int i = 0; i < cfg->ntaps; i++)
		mx = fmax(mx, fabs(h[i]));
	mx *= scale;
	cfg->shift = 0;
	while (cfg->shift < 31 && mx * ldexp(1.0, cfg->shift + 1) < (1 << 17) - 1)
		cfg->shift++;
	for (int i = 0; i < cfg->ntaps; i++)
		cfg->c[i] = (int32_t)nearbyint(h[i] * scale * ldexp(1.0, cfg->shift));
	return 0;
}

/* the DDC's input rate limit (dvbs2_rx_ddc.sv): one sample per 2 clocks, an
 * output per ntaps/2 clocks; 5 % margin */
static int ddc_fast_enough(const struct ddc_cfg *cfg, double fs)
{
	double dec = (double)cfg->R * cfg->D;
	return fs <= 0.95 * CORE_CLK / 2 && fs * (cfg->ntaps / 2) / dec <= 0.95 * CORE_CLK;
}

static uint32_t gold_x(uint32_t n)
{
	uint32_t x = 1;
	while (n--) {
		uint32_t fb = (x ^ (x >> 7)) & 1;
		x = (x >> 1) | (fb << 17);
	}
	return x;
}

/* ---------------------------------------------------------------- TS output */
struct tsout {
	int fd, is_udp;
	struct sockaddr_storage dst;
	socklen_t dstlen;
	uint8_t buf[7 * TS_LEN];
	unsigned n;
	unsigned long packets;
};

static void ts_flush(struct tsout *o)
{
	if (!o->n)
		return;
	if (o->is_udp)
		sendto(o->fd, o->buf, o->n, 0, (struct sockaddr *)&o->dst, o->dstlen);
	else if (write(o->fd, o->buf, o->n) < 0)
		stop = 1;
	o->n = 0;
}

static void ts_sink(void *ctx, const uint8_t pkt[TS_LEN])
{
	struct tsout *o = ctx;
	memcpy(o->buf + o->n, pkt, TS_LEN);
	o->n += TS_LEN;
	o->packets++;
	if (o->n == sizeof(o->buf))
		ts_flush(o);
}

/* ---------------------------------------------------------------- DMA ring */
struct xfer { unsigned slot, id; };

static int dma_submit(unsigned slot, struct xfer *x)
{
	if (rd(dmac, DMAC_SUBMIT) & 1)
		return 0;                           /* queue full */
	x->slot = slot;
	x->id = rd(dmac, DMAC_TRANSFER_ID) & 3;
	/* TLAST: each transfer is one whole packet. Without it axi_dmac takes the
	 * packet's tlast for an early end of the transfer and stops accepting
	 * data until a descriptor with this flag arrives (data_mover.v). */
	wr(dmac, DMAC_FLAGS, 2);
	wr(dmac, DMAC_DEST_ADDR, dmabuf_phys + slot * PKT_BYTES);
	wr(dmac, DMAC_X_LENGTH, PKT_BYTES - 1);
	wr(dmac, DMAC_Y_LENGTH, 0);
	wr(dmac, DMAC_SUBMIT, 1);
	return 1;
}

/* ---------------------------------------------------------------- main */
static const char *modcods[11] = {
	"qpsk1/4", "qpsk1/3", "qpsk2/5", "qpsk1/2", "qpsk3/5", "qpsk2/3",
	"qpsk3/4", "qpsk4/5", "qpsk5/6", "qpsk8/9", "qpsk9/10",
};

/* all 32 MODCODs (PLS decoder): 0 dummy, 29..31 reserved */
static const char *pls_names[32] = {
	"dummy", "qpsk1/4", "qpsk1/3", "qpsk2/5", "qpsk1/2", "qpsk3/5", "qpsk2/3", "qpsk3/4",
	"qpsk4/5", "qpsk5/6", "qpsk8/9", "qpsk9/10", "8psk3/5", "8psk2/3", "8psk3/4", "8psk5/6",
	"8psk8/9", "8psk9/10", "16apsk2/3", "16apsk3/4", "16apsk4/5", "16apsk5/6", "16apsk8/9",
	"16apsk9/10", "32apsk3/4", "32apsk4/5", "32apsk5/6", "32apsk8/9", "32apsk9/10",
	"reserved29", "reserved30", "reserved31",
};

static int has_pls;                 /* core version >= VERSION_PLS */

static void usage(const char *p)
{
	fprintf(stderr,
		"usage: %s [options]\n"
		"  -s, --sym-rate S/s   symbol rate (default 500000)\n"
		"  -F, --adc-rate Hz    ADC sample rate (default 12e6, the system's fixed rate);\n"
		"                       decimation = Hz / (2 x sym-rate), which must be (within\n"
		"                       100 ppm) an integer the DDC supports (1..32, larger ones\n"
		"                       with its CIC, e.g. 180 = 6 x 30 for 33.33 kS/s);\n"
		"                       'auto': the lowest rate >= 2.1 MS/s with a supported\n"
		"                       decimation\n"
		"  -d, --decim N        ADC samples per 2 symbols instead (ADC rate 2 x sym-rate x N)\n"
		"  -m, --modcod M       qpsk1/4 .. qpsk9/10 (default qpsk1/2)\n"
		"  -p, --pilots         frames have pilots\n"
		"  -r, --rolloff R      0.35, 0.25 or 0.20 (default 0.35)\n"
		"  -g, --gold N         PL scrambling Gold code (default 0)\n"
		"  -f, --freq Hz        RX LO frequency\n"
		"  -o, --offset Hz      signal offset from the LO, removed by the DDC NCO\n"
		"  -G, --gain dB        manual RX gain (default: slow_attack AGC)\n"
		"  -S, --ddc-gain N     DDC gain 2^N, 0..8 (default 4)\n"
		"  -I, --iterations N   LDPC iterations (default 30)\n"
		"  -u, --udp host:port  send TS over UDP (default 127.0.0.1:1234)\n"
		"  -O, --output FILE    write TS to FILE ('-': stdout) instead of UDP\n"
		"  -q, --quiet          no status line\n"
		"  -n, --no-iio         leave the AD9363 alone\n"
		"  -C, --constellation FILE  write 1024 symbols (\"I Q\" lines) to FILE every second\n"
		"      --constellation-raw   ... the timing loop output instead of the carrier output\n"
		"      --dump-coefs     print the filter coefficients and exit\n"
		"      --detect         detect the MODCOD and pilots from the PLHEADERs (any -m/-p\n"
		"                       will do; the frame sync needs the right pilots or dummy\n"
		"                       frames): print them and exit 0 after --detect-count equal\n"
		"                       decodes of data headers, exit 1 after --detect-timeout\n"
		"      --detect-count N equal decodes in a row (default 3)\n"
		"      --detect-timeout S  seconds (default 30)\n"
		"      --pls-grid H     PLS decoder frequency hypotheses -H..H x 0.004 cycles/symbol\n"
		"                       (default 32; 2H + 2 hypotheses x ~14 us must fit in a\n"
		"                       dummy frame, 3330 symbols)\n", p);
	exit(2);
}

static void status(int rate_idx, int pilots, double sr, unsigned long ts_pkts, struct bbdeframer *d,
		   unsigned long bad)
{
	/* PLS: the last confident decode of a data header ('!': not the configured one) */
	char pls[48] = "";
	if (has_pls) {
		uint32_t p = rd(regs, R_PLS_DATA);
		if (p & PLS_VALID)
			snprintf(pls, sizeof(pls), ", PLS %s%s%s%s", pls_names[p & 31], (p >> 8) & 1 ? " pilots" : "",
				 (p >> 9) & 1 ? " short" : "",
				 ((p & 31) != (uint32_t)rate_idx + 1 || ((p >> 8) & 1) != (uint32_t)pilots ||
				  ((p >> 9) & 1)) ? "!" : "");
		else
			snprintf(pls, sizeof(pls), ", PLS -");
	}
	static const char *st[4] = { "search", "check", "locked", "?" };
	double amp = rd(regs, R_AMP), en = (double)rd(regs, R_ENERGY) * 16;
	double snr = en > amp * amp ? 10 * log10(amp * amp / (en - amp * amp)) : 99;
	double f = (int32_t)rd(regs, R_FREQ) / 4294967296.0 * sr;
	/* bit error rates since the last status line (counters wrap) */
	static uint32_t raw0, bits0, bch0;
	uint32_t raw = rd(regs, R_RAW_ERR), bits = rd(regs, R_RAW_BITS), bch = rd(regs, R_BCH_BITERR);
	double nb = (double)(uint32_t)(bits - bits0);
	double ber_pre = nb > 0 ? (uint32_t)(raw - raw0) / nb : NAN;
	double ber_post = nb > 0 ? (uint32_t)(bch - bch0) / nb : NAN;
	raw0 = raw; bits0 = bits; bch0 = bch;
	fprintf(stderr, "\r%s %s, %+.0f Hz, Es/N0 %.1f dB, BER %.2e (after LDPC %.1e), AGC %.2f, frames %u (LDPC failed %u, BCH corrected %u failed %u, dropped %u), "
		"TS %lu (CRC %lu, bad %lu), fs %u/%u/%u, acq %u fa %u retype %u, ovf %u%s   ",
		modcods[rate_idx], st[rd(regs, R_FS_STATE) & 3], f, snr, ber_pre, ber_post, rd(regs, R_AGC_GAIN) / 4096.0,
		rd(regs, R_LDPC_FRAMES), rd(regs, R_LDPC_FAILED), rd(regs, R_BCH) & 0xFFFF, rd(regs, R_BCH) >> 16,
		rd(regs, R_PKT_DROPPED),
		ts_pkts, d->crc_errors, bad, rd(regs, R_FS_FOUND), rd(regs, R_FS_FLYWHEEL),
		rd(regs, R_FS_LOST), rd(regs, R_CARRIER_ACQ) & 0xFFFF, rd(regs, R_CARRIER_ACQ) >> 16,
		rd(regs, R_CARRIER_RETYPE), rd(regs, R_IN_OVF) + rd(regs, R_DDC_DROPPED), pls);
}

/* --detect: wait for `count` equal confident decodes of data headers in a
 * row; print them (stdout: "MODCOD name pilots short offset_hz") */
static int detect(double sr, double offset, int count, double timeout)
{
	uint32_t n0 = rd(regs, R_PLS_DATA) >> 16, code = 0;
	int n = 0;
	struct timespec t0, now;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	fprintf(stderr, "dvbs2-rx: detecting (%d equal PLS decodes, %.0f s)\n", count, timeout);
	while (!stop) {
		uint32_t p = rd(regs, R_PLS_DATA), nd = p >> 16;
		if (nd != n0) {
			n0 = nd;
			uint32_t c = p & 0x31F;          /* MODCOD, pilots, short */
			n = (n && c == code) ? n + 1 : 1;
			code = c;
			fprintf(stderr, "PLS %s%s%s (%d)\n", pls_names[c & 31], (c >> 8) & 1 ? " pilots" : "",
				(c >> 9) & 1 ? " short" : "", n);
			if (n >= count)
				break;
		}
		clock_gettime(CLOCK_MONOTONIC, &now);
		if ((now.tv_sec - t0.tv_sec) + (now.tv_nsec - t0.tv_nsec) * 1e-9 > timeout)
			break;
		usleep(2000);
	}
	if (n < count) {
		fprintf(stderr, "dvbs2-rx: no MODCOD detected (fs %s, %u PLS decodes, %u confident data decodes)\n",
			(const char *[]){ "search", "check", "locked", "?" }[rd(regs, R_FS_STATE) & 3],
			rd(regs, R_PLS_LAST) >> 16, rd(regs, R_PLS_DATA) >> 16);
		return 1;
	}
	/* signal frequency from the last decode (if confident): the carrier's
	 * estimate if it used it, else its frequency hypothesis (+-0.002 cycles/symbol) */
	uint32_t l = rd(regs, R_PLS_LAST);
	int32_t fr = (l & PLS_CARRIER) ? (int32_t)rd(regs, R_FREQ) : (int32_t)rd(regs, R_PLS_FREQ);
	double off = (l & PLS_CONF) ? offset + fr / 4294967296.0 * sr : offset;
	fprintf(stderr, "dvbs2-rx: detected %s%s%s, signal at %+.0f Hz from the LO\n", pls_names[code & 31],
		(code >> 8) & 1 ? " pilots" : "", (code >> 9) & 1 ? " short frames (not supported)" : "", off);
	printf("%u %s %u %u %.0f\n", code & 31, pls_names[code & 31], (code >> 8) & 1, (code >> 9) & 1, off);
	return 0;
}

/* constellation snapshot: write the 1024 captured symbols to path, arm again */
static void snapshot(const char *path, int raw)
{
	if ((rd(regs, R_SNAP_CTRL) & 3) == 2) {
		FILE *f = fopen(path, "w");
		if (f) {
			wr(regs, R_SNAP_ADDR, 0);
			for (int i = 0; i < 1024; i++) {
				uint32_t v = rd(regs, R_SNAP_DATA);
				fprintf(f, "%d %d\n", (int16_t)(v & 0xFFFF), (int16_t)(v >> 16));
			}
			fclose(f);
		}
	}
	if (!(rd(regs, R_SNAP_CTRL) & 1))
		wr(regs, R_SNAP_CTRL, 1 | (raw << 1));
}

int main(int argc, char **argv)
{
	double sr = 500e3, rolloff = 0.35, freq = 0, offset = 0, gain = NAN, adc_rate = 12e6;
	int rate_idx = 3, pilots = 0, dec = 0, ddc_gain = 4, max_it = 30, quiet = 0, no_iio = 0, dump = 0;
	uint32_t gold = 0;
	const char *udp = "127.0.0.1:1234", *output = NULL, *constel = NULL;
	int constel_raw = 0, adc_set = 0;
	int det = 0, det_count = 3, pls_h = -1;
	double det_timeout = 30;

	static const struct option lo[] = {
		{ "sym-rate", 1, 0, 's' }, { "decim", 1, 0, 'd' }, { "modcod", 1, 0, 'm' },
		{ "pilots", 0, 0, 'p' }, { "rolloff", 1, 0, 'r' }, { "gold", 1, 0, 'g' },
		{ "freq", 1, 0, 'f' }, { "offset", 1, 0, 'o' }, { "gain", 1, 0, 'G' },
		{ "ddc-gain", 1, 0, 'S' }, { "iterations", 1, 0, 'I' }, { "udp", 1, 0, 'u' },
		{ "output", 1, 0, 'O' }, { "quiet", 0, 0, 'q' }, { "no-iio", 0, 0, 'n' },
		{ "dump-coefs", 0, 0, 1 }, { "constellation", 1, 0, 'C' }, { "constellation-raw", 0, 0, 2 },
		{ "adc-rate", 1, 0, 'F' }, { "detect", 0, 0, 3 }, { "detect-count", 1, 0, 4 },
		{ "detect-timeout", 1, 0, 5 }, { "pls-grid", 1, 0, 6 },
		{ 0, 0, 0, 0 },
	};
	for (int c; (c = getopt_long(argc, argv, "s:d:m:pr:g:f:o:G:S:I:u:O:qnC:F:", lo, NULL)) != -1;) {
		switch (c) {
		case 's': sr = atof(optarg); break;
		case 'd': dec = atoi(optarg); break;
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
		case 'f': freq = atof(optarg); break;
		case 'o': offset = atof(optarg); break;
		case 'G': gain = atof(optarg); break;
		case 'S': ddc_gain = atoi(optarg); break;
		case 'I': max_it = atoi(optarg); break;
		case 'u': udp = optarg; break;
		case 'O': output = optarg; break;
		case 'q': quiet = 1; break;
		case 'n': no_iio = 1; break;
		case 1: dump = 1; break;
		case 'C': constel = optarg; break;
		case 2: constel_raw = 1; break;
		case 'F': adc_rate = strcmp(optarg, "auto") ? atof(optarg) : 0; adc_set = 1; break;
		case 3: det = 1; break;
		case 4: det_count = atoi(optarg); break;
		case 5: det_timeout = atof(optarg); break;
		case 6: pls_h = atoi(optarg); break;
		default: usage(argv[0]);
		}
	}
	if (dec && !adc_set)
		adc_rate = 0;                       /* -d alone: ADC rate 2 x sr x dec */
	if (adc_rate > 0 && !dec) {
		dec = (int)lround(adc_rate / (2 * sr));
		/* within 100 ppm: 33333 S/s means 12e6 / 360 */
		if (dec < 1 || fabs(2 * sr * dec - adc_rate) > 1e-4 * adc_rate) {
			fprintf(stderr, "the ADC rate %.0f is not 2 x %.0f S/s x an integer (-F auto: choose one)\n",
				adc_rate, sr);
			return 2;
		}
	}
	if (!dec) {
		/* the AD9363 needs >= ~2.08 MS/s without its own FIR decimation;
		 * the next decimation the DDC supports */
		dec = (int)ceil(2.1e6 / (2 * sr));
		if (dec < 1)
			dec = 1;
		int R, D;
		while (dec < DEC_MAX && ddc_split(dec, &R, &D))
			dec++;
	}
	double fs = 2 * sr * dec;
	if (adc_rate > 0) {
		if (fabs(fs - adc_rate) > 1e-4 * adc_rate) {
			fprintf(stderr, "-d %d and -F %.0f disagree (ADC rate %.0f)\n", dec, adc_rate, fs);
			return 2;
		}
		/* the ADC runs at the given rate (12 MS/s by default); the symbol
		 * rate is the exact fs / (2 dec), e.g. 33333.33 for -s 33333 */
		fs = adc_rate;
		sr = fs / (2.0 * dec);
	}
	struct ddc_cfg dc;
	if (dec < 1 || dec > DEC_MAX || fs > 61.44e6 || ddc_gain < 0 || ddc_gain > 8 ||
	    max_it < 1 || max_it > 63 || gold >= 262143 || det_count < 1 || pls_h > 63 ||
	    ddc_config(dec, ddc_gain, &dc)) {
		fprintf(stderr, "bad parameters (decimation %d, ADC rate %.0f)\n", dec, fs);
		return 2;
	}
	if (!ddc_fast_enough(&dc, fs)) {
		fprintf(stderr, "the DDC is too slow for an ADC rate of %.0f S/s with decimation %d (%d taps)\n",
			fs, dec, dc.ntaps);
		return 2;
	}

	int32_t rrc[RRC_NPH][32];
	rrc_bank(rolloff, rrc);
	const int32_t *ddc = dc.c;
	int ntaps = dc.ntaps;
	if (dump) {
		for (int k = 0; k < RRC_NPH; k++)
			for (int j = 0; j < RRC_NT; j++)
				printf("%d%c", rrc[k][j], j == RRC_NT - 1 ? '\n' : ' ');
		for (int i = 0; i < ntaps; i++)
			printf("%d%c", ddc[i], i == ntaps - 1 ? '\n' : ' ');
		printf("%u %u\n", gold_x(gold), gold_x(gold + 131072));
		printf("%d %d %d %d\n", dc.R, dc.D, dc.cic_shift, dc.shift);
		return 0;
	}

	/* ---- hardware */
	int uio = find_by_name("/sys/class/uio", "uio", "dvbs2-rx");
	if (uio < 0) {
		fprintf(stderr, "no UIO device named dvbs2-rx (wrong bitstream/device tree?)\n");
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
	if (sz < NSLOTS * PKT_BYTES) {
		fprintf(stderr, "DMA buffer too small (%zu)\n", sz);
		return 1;
	}
	if (rd(regs, R_ID) != 0x44565232) {
		fprintf(stderr, "dvbs2-rx core not found (ID %08x)\n", rd(regs, R_ID));
		return 1;
	}
	if (dc.R > 1 && rd(regs, R_VERSION) < VERSION_CIC) {
		fprintf(stderr, "decimation %d needs the DDC's CIC (core version %08x < %08x: old bitstream)\n",
			dec, rd(regs, R_VERSION), VERSION_CIC);
		return 1;
	}

	has_pls = rd(regs, R_VERSION) >= VERSION_PLS;
	if ((det || pls_h >= 0) && !has_pls) {
		fprintf(stderr, "--detect / --pls-grid need the PLS decoder (core version %08x < %08x: old bitstream)\n",
			rd(regs, R_VERSION), VERSION_PLS);
		return 1;
	}

	if (!no_iio) {
		int phy = find_by_name("/sys/bus/iio/devices", "iio:device", "ad9361-phy");
		char v[64];
		if (phy < 0) {
			fprintf(stderr, "AD9363 IIO device not found\n");
			return 1;
		}
		snprintf(v, sizeof(v), "%.0f", fs);
		iio_set(phy, "in_voltage_sampling_frequency", v);
		/* at the system rate (12 MS/s) the AD9363's analog filter is shared
		 * with the other receive users (nbiq, the spectrum): leave it; narrow
		 * it only when the ADC rate was chosen for this receiver (-F, -d) */
		if (fabs(fs - 12e6) > 1.0) {
			snprintf(v, sizeof(v), "%.0f", fmax(sr * (1 + rolloff) * 1.2, 200e3));
			iio_set(phy, "in_voltage_rf_bandwidth", v);
		}
		if (freq > 0) {
			snprintf(v, sizeof(v), "%.0f", freq);
			iio_set(phy, "out_altvoltage0_RX_LO_frequency", v);
		}
		if (isnan(gain)) {
			iio_set(phy, "in_voltage0_gain_control_mode", "slow_attack");
		} else {
			iio_set(phy, "in_voltage0_gain_control_mode", "manual");
			snprintf(v, sizeof(v), "%.0f", gain);
			iio_set(phy, "in_voltage0_hardwaregain", v);
		}
	}

	wr(regs, R_CTRL, CTRL_RESET);
	usleep(10);
	wr(regs, R_MODCOD, (rate_idx + 1) | (pilots << 8));
	wr(regs, R_GOLD_XA, gold_x(gold));
	wr(regs, R_GOLD_XB, gold_x(gold + 131072));
	wr(regs, R_DDC_FINC, (uint32_t)(int64_t)llround(-offset / fs * 4294967296.0));
	wr(regs, R_DDC_CFG, dc.D | (ntaps << 8) | (dc.shift << 24));
	if (rd(regs, R_VERSION) >= VERSION_CIC)
		wr(regs, R_DDC_CIC, (dc.R > 1 ? dc.R : 0) | (dc.cic_shift << 8));
	wr(regs, R_DDC_CADDR, 0);
	for (int i = 0; i < ntaps; i++)
		wr(regs, R_DDC_CDATA, (uint32_t)ddc[i] & 0x3FFFF);
	wr(regs, R_RRC_CADDR, 0);
	for (int k = 0; k < RRC_NPH; k++)
		for (int j = 0; j < 32; j++)
			wr(regs, R_RRC_CDATA, (uint32_t)rrc[k][j] & 0x3FFFF);
	wr(regs, R_TIMING_GAINS, TIMING_KP | (TIMING_KI << 16));
	wr(regs, R_FS_THRESH, FS_THRESH);
	wr(regs, R_PLL_KP, PLL_KP);
	wr(regs, R_PLL_KI, PLL_KI);
	wr(regs, R_LDPC_MAX_IT, max_it);
	if (pls_h >= 0)
		wr(regs, R_PLS_CFG, pls_h | (262u << 16));

	wr(dmac, DMAC_CONTROL, 0);
	wr(dmac, DMAC_IRQ_MASK, 0x3);
	wr(dmac, DMAC_CONTROL, 1);

	/* soft reset again so the core starts with the new configuration */
	wr(regs, R_CTRL, CTRL_RESET);
	usleep(10);
	wr(regs, R_CTRL, CTRL_ENABLE);

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	if (det) {
		int r = detect(sr, offset, det_count, det_timeout);
		wr(regs, R_CTRL, 0);
		return r;
	}

	/* ---- output */
	struct tsout out = { 0 };
	if (output) {
		out.fd = strcmp(output, "-") ? open(output, O_WRONLY | O_CREAT | O_TRUNC, 0644) : 1;
		if (out.fd < 0)
			die(output);
	} else {
		char host[128];
		const char *colon = strrchr(udp, ':');
		if (!colon || colon == udp || (size_t)(colon - udp) >= sizeof(host))
			usage(argv[0]);
		memcpy(host, udp, colon - udp);
		host[colon - udp] = 0;
		struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_DGRAM }, *ai;
		if (getaddrinfo(host, colon + 1, &hints, &ai)) {
			fprintf(stderr, "cannot resolve %s\n", udp);
			return 1;
		}
		out.fd = socket(ai->ai_family, SOCK_DGRAM, 0);
		if (out.fd < 0)
			die("socket");
		memcpy(&out.dst, ai->ai_addr, ai->ai_addrlen);
		out.dstlen = ai->ai_addrlen;
		out.is_udp = 1;
		freeaddrinfo(ai);
	}

	fprintf(stderr, "dvbs2-rx: %s%s, %.0f S/s, ADC %.0f S/s (decimation %d = CIC %d x FIR %d, %d taps), "
		"roll-off %.2f, Gold %u, offset %.0f Hz\n", modcods[rate_idx], pilots ? " pilots" : "", sr, fs,
		dec, dc.R, dc.D, ntaps, rolloff, gold, offset);

	struct bbdeframer d;
	bbdeframer_init(&d);
	struct xfer q[NQUEUE];
	unsigned qh = 0, qn = 0, slot = 0;
	unsigned long bad = 0, lost = 0;
	int have_cnt = 0;
	uint16_t next_cnt = 0;
	struct timespec last, now;
	time_t last_snap = 0;
	clock_gettime(CLOCK_MONOTONIC, &last);

	while (!stop) {
		/* keep the DMA queue full */
		while (qn < NQUEUE && dma_submit(slot, &q[(qh + qn) % NQUEUE])) {
			slot = (slot + 1) % NSLOTS;
			qn++;
		}
		/* oldest transfer done? */
		if (qn && (rd(dmac, DMAC_DONE) >> q[qh].id) & 1) {
			/* The buffer is mapped uncached (device memory): no unaligned
			 * accesses there, so copy the packet out with aligned words. */
			static uint64_t pkt[PKT_BYTES / 8];
			const volatile uint64_t *src = (const volatile uint64_t *)(dmabuf + q[qh].slot * PKT_BYTES);
			uint64_t h = src[0];
			unsigned len = h & 0xFFFF, cnt = (h >> 16) & 0xFFFF;
			unsigned nw = len + 8 <= PKT_BYTES ? (len + 8 + 7) / 8 : 1;
			for (unsigned i = 0; i < nw; i++)
				pkt[i] = src[i];
			const uint8_t *p = (const uint8_t *)pkt;
			int ok = (h >> 49) & 1;          /* BCH: codeword (LDPC converged: bit 48) */
			if ((h >> 56) != 0xB5 || len != dvbs2_kbch_bytes(rate_idx)) {
				bad++;
				bbdeframer_gap(&d);
			} else {
				if (have_cnt && cnt != next_cnt) {
					lost += (uint16_t)(cnt - next_cnt);
					bbdeframer_gap(&d);   /* frames dropped in the PL */
				}
				have_cnt = 1;
				next_cnt = cnt + 1;
				if (!ok) {
					bad++;
					bbdeframer_gap(&d);
				} else {
					bbdeframer_frame(&d, p + 8, len, ts_sink, &out);
				}
			}
			ts_flush(&out);
			qh = (qh + 1) % NQUEUE;
			qn--;
			continue;
		}
		usleep(2000);
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (!quiet && (now.tv_sec - last.tv_sec) * 1000 + (now.tv_nsec - last.tv_nsec) / 1000000 >= 1000) {
			status(rate_idx, pilots, sr, out.packets, &d, bad);
			last = now;
		}
		if (constel && (now.tv_sec != last_snap)) {
			snapshot(constel, constel_raw);
			last_snap = now.tv_sec;
		}
	}
	if (!quiet)
		fprintf(stderr, "\n");
	fprintf(stderr, "dvbs2-rx: %lu TS packets, %lu BBFRAMEs, %lu bad/failed, %lu lost, %lu CRC errors\n",
		out.packets, d.frames, bad, lost, d.crc_errors);
	wr(regs, R_CTRL, 0);
	wr(dmac, DMAC_CONTROL, 0);
	return 0;
}
