/* DVB-S2 TS mode adaptation, see bbframe.h. */
#include "bbframe.h"

#include <string.h>

static const unsigned kbch_tab[11] = {
	16008, 21408, 25728, 32208, 38688, 43040, 48408, 51648, 53840, 57472, 58192,
};
/* short FECFRAMEs (EN 302 307-1 Table 5b), no 9/10 */
static const unsigned kbch_short_tab[10] = {
	3072, 5232, 6312, 7032, 9552, 10632, 11712, 12432, 13152, 14232,
};

unsigned dvbs2_kbch_bytes(int rate_idx)
{
	return dvbs2_kbch_bytes_x(rate_idx, 0);
}

unsigned dvbs2_kbch_bytes_x(int rate_idx, int short_frame)
{
	if (rate_idx < 0 || rate_idx > (short_frame ? 9 : 10))
		return 0;
	return (short_frame ? kbch_short_tab : kbch_tab)[rate_idx] / 8;
}

static const char *const modcod_names[DVBS2_NMODCOD + 1] = {
	"dummy",
	"qpsk1/4", "qpsk1/3", "qpsk2/5", "qpsk1/2", "qpsk3/5", "qpsk2/3",
	"qpsk3/4", "qpsk4/5", "qpsk5/6", "qpsk8/9", "qpsk9/10",
	"8psk3/5", "8psk2/3", "8psk3/4", "8psk5/6", "8psk8/9", "8psk9/10",
};
static const int psk8_rate[6] = { 4, 5, 6, 8, 9, 10 };

int dvbs2_modcod_parse(const char *name)
{
	for (int m = 1; m <= DVBS2_NMODCOD; m++)
		if (!strcmp(name, modcod_names[m]))
			return m;
	return 0;
}

const char *dvbs2_modcod_name(int modcod)
{
	return (modcod >= 1 && modcod <= DVBS2_NMODCOD) ? modcod_names[modcod] : "?";
}

int dvbs2_modcod_rate(int modcod)
{
	if (modcod >= 1 && modcod <= 11)
		return modcod - 1;
	if (modcod >= 12 && modcod <= 17)
		return psk8_rate[modcod - 12];
	return -1;
}

int dvbs2_modcod_psk8(int modcod)
{
	return modcod >= 12 && modcod <= 17;
}

int dvbs2_modcod_valid(int modcod, int short_frame)
{
	int r = dvbs2_modcod_rate(modcod);
	return r >= 0 && !(short_frame && r == 10);
}

unsigned dvbs2_plframe_symbols(int modcod, int pilots, int short_frame)
{
	unsigned slots = (short_frame ? 16200 : 64800) / (dvbs2_modcod_psk8(modcod) ? 3 : 2) / 90;
	return 90 + 90 * slots + (pilots ? 36 * ((slots - 1) / 16) : 0);
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
	return bbframer_init_x(b, rate_idx, 0, rolloff);
}

int bbframer_init_x(struct bbframer *b, int rate_idx, int short_frame, double rolloff)
{
	unsigned ro;

	memset(b, 0, sizeof(*b));
	b->kbch = dvbs2_kbch_bytes_x(rate_idx, short_frame) * 8;
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
