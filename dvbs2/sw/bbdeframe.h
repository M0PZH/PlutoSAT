/* DVB-S2 TS mode de-adaptation (inverse of bbframe.h): BBFRAME -> 188-byte
 * TS packets. The sync byte of each user packet carries the CRC-8 of the
 * previous one; packets may span BBFRAMEs. */
#ifndef DVBS2_BBDEFRAME_H
#define DVBS2_BBDEFRAME_H

#include <stddef.h>
#include <stdint.h>

#include "bbframe.h"

struct bbdeframer {
	uint8_t pkt[TS_LEN];    /* packet being assembled */
	unsigned pkt_len;       /* bytes in pkt (0 = none) */
	uint8_t last_crc;       /* CRC-8 of the last complete packet */
	int have_last;          /* last_crc valid (no gap since) */
	/* statistics */
	unsigned long frames, bad_header, packets, crc_errors, resyncs;
};

typedef void (*ts_sink_fn)(void *ctx, const uint8_t pkt[TS_LEN]);

void bbdeframer_init(struct bbdeframer *d);

/* A BBFRAME was lost (LDPC failure, dropped packet): discard the packet in
 * progress and the CRC chain. */
void bbdeframer_gap(struct bbdeframer *d);

/* Process one BBFRAME of len bytes (Kbch/8). Complete TS packets (sync byte
 * restored to 0x47) go to sink. Returns the number of packets, or -1 for a
 * bad BBHEADER (CRC, MATYPE not TS, UPL != 1504, DFL too long). */
int bbdeframer_frame(struct bbdeframer *d, const uint8_t *frame, unsigned len,
		     ts_sink_fn sink, void *ctx);

#endif
