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

/* rate_idx 0..10 = code rate 1/4 .. 9/10 (normal FECFRAMEs); rolloff 0.35,
 * 0.25 or 0.20 */
int bbframer_init(struct bbframer *b, int rate_idx, double rolloff);
/* the same for a normal (short_frame 0) or short (1) FECFRAME; short frames
 * have no 9/10 */
int bbframer_init_x(struct bbframer *b, int rate_idx, int short_frame, double rolloff);

/* Kbch/8 for a rate index, 0 if invalid */
unsigned dvbs2_kbch_bytes(int rate_idx);
unsigned dvbs2_kbch_bytes_x(int rate_idx, int short_frame);

/* MODCODs: QPSK 1..11 (rate_idx = MODCOD - 1), 8PSK 12..17 (3/5, 2/3, 3/4,
 * 5/6, 8/9, 9/10) */
#define DVBS2_NMODCOD 17
/* "qpsk1/4" .. "qpsk9/10", "8psk3/5" .. "8psk9/10" -> MODCOD, 0 if unknown */
int dvbs2_modcod_parse(const char *name);
const char *dvbs2_modcod_name(int modcod);          /* "?" if invalid */
int dvbs2_modcod_rate(int modcod);                  /* rate_idx, -1 if invalid */
int dvbs2_modcod_psk8(int modcod);
/* the code exists (short frames: no 9/10) */
int dvbs2_modcod_valid(int modcod, int short_frame);
/* symbols of a data PLFRAME (header, slots, pilot blocks) */
unsigned dvbs2_plframe_symbols(int modcod, int pilots, int short_frame);

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
