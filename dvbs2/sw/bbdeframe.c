/* DVB-S2 TS mode de-adaptation, see bbdeframe.h. */
#include "bbdeframe.h"

#include <string.h>

void bbdeframer_init(struct bbdeframer *d)
{
	memset(d, 0, sizeof(*d));
}

void bbdeframer_gap(struct bbdeframer *d)
{
	if (d->pkt_len || d->have_last)
		d->resyncs++;
	d->pkt_len = 0;
	d->have_last = 0;
}

/* a packet's first byte is the CRC of the previous one: check, then emit */
static void finish(struct bbdeframer *d, ts_sink_fn sink, void *ctx)
{
	uint8_t crc = dvbs2_crc8(d->pkt + 1, TS_LEN - 1);
	if (d->have_last && d->pkt[0] != d->last_crc)
		d->crc_errors++;
	d->last_crc = crc;
	d->have_last = 1;
	d->pkt[0] = 0x47;
	sink(ctx, d->pkt);
	d->packets++;
	d->pkt_len = 0;
}

int bbdeframer_frame(struct bbdeframer *d, const uint8_t *f, unsigned len,
		     ts_sink_fn sink, void *ctx)
{
	unsigned before = d->packets;
	d->frames++;
	if (len < BBHEADER_BYTES || dvbs2_crc8(f, 9) != f[9] || (f[0] >> 6) != 3 ||
	    ((f[2] << 8) | f[3]) != TS_LEN * 8) {
		d->bad_header++;
		bbdeframer_gap(d);
		return -1;
	}
	unsigned dfl = (f[4] << 8) | f[5];
	unsigned syncd = (f[7] << 8) | f[8];
	if (dfl % 8 || dfl / 8 > len - BBHEADER_BYTES || (syncd != 0xFFFF && (syncd % 8 || syncd > dfl))) {
		d->bad_header++;
		bbdeframer_gap(d);
		return -1;
	}
	const uint8_t *df = f + BBHEADER_BYTES;
	unsigned n = dfl / 8, pos = 0;

	/* bytes before the first packet start continue the packet in progress */
	unsigned cont = syncd == 0xFFFF ? n : syncd / 8;
	if (d->pkt_len) {
		if (d->pkt_len + cont > TS_LEN || (syncd != 0xFFFF && d->pkt_len + cont != TS_LEN)) {
			bbdeframer_gap(d);          /* does not fit: a frame went missing */
		} else {
			memcpy(d->pkt + d->pkt_len, df, cont);
			d->pkt_len += cont;
			if (d->pkt_len == TS_LEN)
				finish(d, sink, ctx);
		}
	}
	pos = cont;
	while (pos < n) {
		unsigned take = n - pos < TS_LEN ? n - pos : TS_LEN;
		memcpy(d->pkt, df + pos, take);
		d->pkt_len = take;
		pos += take;
		if (take == TS_LEN)
			finish(d, sink, ctx);
	}
	return (int)(d->packets - before);
}
