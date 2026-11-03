/* DVB-S2 TS mode adaptation, see bbframe.h. */
#include "bbframe.h"

#include <string.h>

static const unsigned kbch_tab[11] = {
	16008, 21408, 25728, 32208, 38688, 43040, 48408, 51648, 53840, 57472, 58192,
};

unsigned dvbs2_kbch_bytes(int rate_idx)
{
	if (rate_idx < 0 || rate_idx > 10)
		return 0;
	return kbch_tab[rate_idx] / 8;
}

/* g(x) = x^8 + x^7 + x^6 + x^4 + x^2 + 1, MSB first, init 0 */
uint8_t dvbs2_crc8(const uint8_t *p, size_t n)
{
	uint8_t crc = 0;

	while (n--) {
		crc ^= *p++;
		for (int i = 0; i < 8; i++)
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
	}
	return crc;
}

int bbframer_init(struct bbframer *b, int rate_idx, double rolloff)
{
	unsigned ro;

	memset(b, 0, sizeof(*b));
	b->kbch = dvbs2_kbch_bytes(rate_idx) * 8;
	if (!b->kbch)
		return -1;
	if (rolloff > 0.3)
		ro = 0;
	else if (rolloff > 0.22)
		ro = 1;
	else
		ro = 2;
	/* TS (11), single input stream, CCM, no ISSYI, no NPD, roll-off */
	b->matype1 = 0xF0 | ro;
	return 0;
}

unsigned bbframer_build(struct bbframer *b, ts_source_fn src, void *ctx, uint8_t *out)
{
	const unsigned df_bytes = b->kbch / 8 - BBHEADER_BYTES;
	uint8_t *df = out + BBHEADER_BYTES;
	unsigned n = 0;
	unsigned syncd = 0xFFFF;   /* no UP starts in this frame */

	while (n < df_bytes) {
		if (!b->have_pkt) {
			if (!src(ctx, b->pkt))
				break;
			b->have_pkt = 1;
			b->pkt_pos = 0;
		}
		if (b->pkt_pos == 0) {
			if (syncd == 0xFFFF)
				syncd = n * 8;
			/* sync byte carries the CRC-8 of the previous packet */
			uint8_t crc_prev = b->last_crc;
			b->last_crc = dvbs2_crc8(b->pkt + 1, TS_LEN - 1);
			b->pkt[0] = crc_prev;
		}
		unsigned take = TS_LEN - b->pkt_pos;
		if (take > df_bytes - n)
			take = df_bytes - n;
		memcpy(df + n, b->pkt + b->pkt_pos, take);
		n += take;
		b->pkt_pos += take;
		if (b->pkt_pos == TS_LEN)
			b->have_pkt = 0;
	}
	if (n == 0)
		return 0;

	unsigned dfl = n * 8;
	out[0] = b->matype1;
	out[1] = 0;                 /* MATYPE-2 */
	out[2] = (TS_LEN * 8) >> 8; /* UPL */
	out[3] = (TS_LEN * 8) & 0xFF;
	out[4] = dfl >> 8;
	out[5] = dfl & 0xFF;
	out[6] = 0x47;              /* SYNC */
	out[7] = syncd >> 8;
	out[8] = syncd & 0xFF;
	out[9] = dvbs2_crc8(out, 9);
	memset(df + n, 0, df_bytes - n);
	return dfl;
}
