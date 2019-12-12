/* Ring buffer implementation */

#include "ringBuffer.h"

void ringAddByte(volatile ring_generic* buf, uint8_t data) {
	buf->data[buf->ptr_e] = data;
	buf->ptr_e = (buf->ptr_e + 1) & buf->max;
	buf->empty = FALSE;
}

uint8_t ringRemoveByte(volatile ring_generic* buf) {
	uint8_t result;
	if (ringLength(*buf) == 0) return 0;
	result = buf->data[buf->ptr_b];
	buf->ptr_b = (buf->ptr_b + 1) & buf->max;
	if (buf->ptr_b == buf->ptr_e) buf->empty = TRUE;
	return result;
}

void ringRemoveFrame(volatile ring_generic* buf, uint8_t num) {
	uint8_t x;
	x = ringLength(*buf);
	if (x > num) x = num;
	buf->ptr_b = (buf->ptr_b + x) & buf->max;
	if (buf->ptr_b == buf->ptr_e) buf->empty = TRUE;
}

uint8_t ringReadByte(volatile ring_generic* buf, uint8_t offset) {
	uint8_t pos;
	pos = (buf->ptr_b + offset) & buf->max;
	return buf->data[pos];
}

void ringSerialize(volatile ring_generic* buf, uint8_t* out, uint8_t start, uint8_t length) {
	int i;
	for (i = 0; i < length; i++)
		out[i] = buf->data[(start + i) & buf->max];
}

void ringRemoveFromMiddle(volatile ring_generic* buf, uint8_t start, uint8_t length) {
	int i;
	for (i = start; i != ((buf->ptr_e - length) & buf->max); i++)
		buf->data[i & buf->max] = buf->data[(i + length) & buf->max];
	buf->ptr_e = i & buf->max;
	if (buf->ptr_b == buf->ptr_e) buf->empty = TRUE;
}

void ringAddToStart(volatile ring_generic* buf, uint8_t* data, uint8_t len) {
	int i;

	// check full buffer
	if (ringLength(*buf) < len) return;

	buf->ptr_b = (buf->ptr_b - len) & buf->max;
	for (i = 0; i < len; i++)
		buf->data[(buf->ptr_b + i) & buf->max] = data[i];
	if (len > 0) { buf->empty = FALSE; }
}

void ringClear(volatile ring_generic* buf) {
	buf->ptr_b = buf->ptr_e;
	buf->empty = TRUE;
}
