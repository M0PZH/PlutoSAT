/* Host test helper: TS on stdin -> packed BBFRAMEs on stdout.
 *   bbframe_cli <rate_idx> <rolloff>
 * Only complete BBFRAMEs are written (like gr-dtv's dvb_bbheader_bb). */
#include <stdio.h>
#include <stdlib.h>

#include "bbframe.h"

static int src_stdin(void *ctx, uint8_t pkt[TS_LEN])
{
	(void)ctx;
	return fread(pkt, 1, TS_LEN, stdin) == TS_LEN;
}

int main(int argc, char **argv)
{
	struct bbframer b;
	uint8_t out[BBFRAME_MAX_BYTES];

	if (argc < 3 || bbframer_init(&b, atoi(argv[1]), atof(argv[2]))) {
		fprintf(stderr, "usage: %s <rate_idx 0..10> <rolloff>\n", argv[0]);
		return 1;
	}
	for (;;) {
		unsigned dfl = bbframer_build(&b, src_stdin, NULL, out);
		if (dfl != b.kbch - 80)   /* partial frame: input ended */
			break;
		fwrite(out, 1, b.kbch / 8, stdout);
	}
	return 0;
}
