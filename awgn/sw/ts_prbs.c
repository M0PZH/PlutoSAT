/*
 * ts-prbs: test transport stream for loopback tests.
 *
 *   ts-prbs gen [-n PACKETS]          write packets to stdout (forever by default)
 *   ts-prbs check [-t SEC] [-w SEC]   read packets from stdin; after the warm-up
 *                                     (-w, default 2 s from the first good packet)
 *                                     count for -t seconds (default: until EOF), then
 *                                     print "ok N bad N lost N per X"
 *
 * Packet: 0x47, PID 0x0100, CC, then a 32-bit big-endian sequence number and
 * 180 bytes from an xorshift32 seeded by it. Lost packets are counted from
 * gaps in the sequence numbers; other PIDs (null packets) are ignored.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define TS 188
#define PID 0x0100

static void payload(uint32_t seq, uint8_t *p)
{
	uint32_t x = seq * 2654435761u ^ 0xA5A5A5A5u;
	if (!x)
		x = 1;
	p[0] = seq >> 24; p[1] = seq >> 16; p[2] = seq >> 8; p[3] = seq;
	for (int i = 4; i < 184; i++) {
		x ^= x << 13; x ^= x >> 17; x ^= x << 5;
		p[i] = x;
	}
}

static void make(uint32_t seq, uint8_t *pkt)
{
	pkt[0] = 0x47;
	pkt[1] = PID >> 8;
	pkt[2] = PID & 0xFF;
	pkt[3] = 0x10 | (seq & 15);
	payload(seq, pkt + 4);
}

static int write_all(int fd, const uint8_t *p, size_t n)
{
	while (n) {
		ssize_t w = write(fd, p, n);
		if (w < 0) {
			if (errno == EINTR)
				continue;
			return -1;
		}
		p += w;
		n -= w;
	}
	return 0;
}

static double now(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }

static int gen(long n)
{
	uint8_t buf[TS * 64];
	signal(SIGPIPE, SIG_IGN);
	for (uint32_t seq = 0; n < 0 || seq < (uint32_t)n;) {
		int k = 0;
		for (; k < 64 && (n < 0 || seq < (uint32_t)n); k++, seq++)
			make(seq, buf + TS * k);
		if (write_all(1, buf, TS * k))
			return errno == EPIPE ? 0 : 1;
	}
	return 0;
}

static int check(double secs, double warm)
{
	uint8_t pkt[TS], ref[184];
	unsigned long ok = 0, bad = 0, lost = 0;
	uint32_t last = 0;
	int have = 0, counting = 0;
	unsigned long bad_since = 0;               /* bad packets since the last good one */
	size_t fill = 0;
	double t0 = 0, tc = 0;

	/* no SA_RESTART: the alarm must interrupt a read that waits forever */
	struct sigaction sa = { .sa_handler = on_signal };
	sigemptyset(&sa.sa_mask);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGALRM, &sa, NULL);
	if (secs > 0)
		alarm((unsigned)(warm + secs + 10));    /* nothing arrives: give up */
	while (!stop) {
		ssize_t r = read(0, pkt + fill, TS - fill);
		if (r < 0) {
			if (errno == EINTR && !stop)
				continue;
			break;
		}
		if (r == 0)
			break;
		fill += r;
		if (fill < TS)
			continue;
		fill = 0;
		if (pkt[0] != 0x47) {               /* resync: find the next sync byte */
			uint8_t *s = memchr(pkt + 1, 0x47, TS - 1);
			if (counting)
				bad++;
			bad_since++;
			if (s) {
				fill = TS - (s - pkt);
				memmove(pkt, s, fill);
			}
			continue;
		}
		if ((((pkt[1] & 0x1F) << 8) | pkt[2]) != PID)
			continue;
		uint32_t seq = (uint32_t)pkt[4] << 24 | pkt[5] << 16 | pkt[6] << 8 | pkt[7];
		payload(seq, ref);
		int good = !memcmp(ref, pkt + 4, 184);
		double t = now();
		if (!counting) {
			if (good && !t0)
				t0 = t;
			if (t0 && t - t0 >= warm) {
				counting = 1;
				tc = t;
				if (secs > 0)
					alarm((unsigned)(secs + 0.999));
			}
		}
		if (!good) {
			if (counting)
				bad++;
			bad_since++;
			continue;
		}
		if (counting && have && seq - last > 1 && seq - last < 1u << 30) {
			unsigned long gap = seq - last - 1;
			lost += gap > bad_since ? gap - bad_since : 0;
		}
		bad_since = 0;
		if (counting)
			ok++;
		last = seq;
		have = 1;
		if (counting && secs > 0 && t - tc >= secs)
			break;
	}
	unsigned long tot = ok + bad + lost;
	printf("ok %lu bad %lu lost %lu per %.3g\n", ok, bad, lost, tot ? (double)(bad + lost) / tot : 1.0);
	return counting ? 0 : 1;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: ts-prbs gen [-n PACKETS] | check [-t SEC] [-w SEC]\n");
		return 2;
	}
	const char *mode = argv[1];
	long n = -1;
	double secs = 0, warm = 2;
	int opt;
	optind = 2;
	while ((opt = getopt(argc, argv, "n:t:w:")) != -1) {
		switch (opt) {
		case 'n': n = atol(optarg); break;
		case 't': secs = atof(optarg); break;
		case 'w': warm = atof(optarg); break;
		default: return 2;
		}
	}
	if (!strcmp(mode, "gen"))
		return gen(n);
	if (!strcmp(mode, "check"))
		return check(secs, warm);
	return 2;
}
