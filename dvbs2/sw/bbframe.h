/* DVB-S2 mode adaptation for a single MPEG-TS stream (EN 302 307-1 5.1):
 * BBHEADER + data field, sync bytes replaced by the CRC-8 of the previous
 * user packet, packets split freely across BBFRAMEs. */
#ifndef DVBS2_BBFRAME_H
#define DVBS2_BBFRAME_H

#include <stddef.h>
#include <stdint.h>

#define TS_LEN 188
#define BBHEADER_BYTES 10
#define BBFRAME_MAX_BYTES (58192 / 8)

struct bbframer {
	unsigned kbch;          /* BBFRAME length in bits */
	uint8_t matype1;
	uint8_t last_crc;       /* CRC-8 of the previous user packet */
	uint8_t pkt[TS_LEN];    /* partially transmitted packet */
	unsigned pkt_pos;       /* bytes of pkt already sent (0 = none pending) */
	int have_pkt;
};

/* rate_idx 0..10 = QPSK 1/4 .. 9/10; rolloff 0.35, 0.25 or 0.20 */
int bbframer_init(struct bbframer *b, int rate_idx, double rolloff);

/* Kbch/8 for a rate index, 0 if invalid */
unsigned dvbs2_kbch_bytes(int rate_idx);

/* Callback supplying the next 188-byte TS packet; returns 0 when none is
 * available right now. */
typedef int (*ts_source_fn)(void *ctx, uint8_t pkt[TS_LEN]);

/* Build one BBFRAME into out (Kbch/8 bytes). Uses as many packets as the
 * source provides; if it runs dry the data field is shortened (DFL) and the
 * rest padded with zeros. Returns the DFL in bits (0: nothing to send, out
 * untouched). */
unsigned bbframer_build(struct bbframer *b, ts_source_fn src, void *ctx, uint8_t *out);

uint8_t dvbs2_crc8(const uint8_t *p, size_t n);

#endif
