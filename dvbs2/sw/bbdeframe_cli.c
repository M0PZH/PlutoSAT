/* Host test helper: packed BBFRAMEs (Kbch/8 bytes each) on stdin -> TS on
 * stdout; statistics on stderr.
 *   bbdeframe_cli <rate_idx> [lost frame indices...] */
#include <stdio.h>
#include <stdlib.h>

#include "bbdeframe.h"

static void sink_stdout(void *ctx, const uint8_t pkt[TS_LEN])
{
	(void)ctx;
	fwrite(pkt, 1, TS_LEN, stdout);
}

int main(int argc, char **argv)
{
	struct bbdeframer d;
	uint8_t f[BBFRAME_MAX_BYTES];
	unsigned len = argc > 1 ? dvbs2_kbch_bytes(atoi(argv[1])) : 0;

	if (!len) {
		fprintf(stderr, "usage: %s <rate_idx 0..10> [lost frame indices]\n", argv[0]);
		return 1;
	}
	bbdeframer_init(&d);
	for (unsigned k = 0; fread(f, 1, len, stdin) == len; k++) {
		int lost = 0;
		for (int a = 2; a < argc; a++)
			lost |= (unsigned)atoi(argv[a]) == k;
		if (lost)
			bbdeframer_gap(&d);
		else
			bbdeframer_frame(&d, f, len, sink_stdout, NULL);
	}
	fprintf(stderr, "frames %lu bad_header %lu packets %lu crc_errors %lu resyncs %lu\n",
		d.frames, d.bad_header, d.packets, d.crc_errors, d.resyncs);
	return 0;
}
