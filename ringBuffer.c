/*
 * Ring buffer implementation
 * (c) Michal Petrilak, Jan Horacek 2015
 * Version: 1.0.1
 */

#include "ringBuffer.h"

void ringAddByte(ring_generic *buf, BYTE data)
{
    buf->data[buf->ptr_e] = data;
    buf->ptr_e = (buf->ptr_e + 1) & buf->max;
    buf->empty = FALSE;
}

BYTE ringRemoveByte(ring_generic* buf)
{
    BYTE result;
    if (ringLength(buf) == 0) return 0;
    result = buf->data[buf->ptr_b];
    buf->ptr_b = (buf->ptr_b + 1) & buf->max;
    if (buf->ptr_b == buf->ptr_e) buf->empty = TRUE;
    return result;
}

void ringRemoveFrame(ring_generic* buf, BYTE num)
{
    BYTE x;
    x = ringLength(buf);
    if (x > num) x = num;
    buf->ptr_b = (buf->ptr_b + x) & buf->max;
    if (buf->ptr_b == buf->ptr_e) buf->empty = TRUE;
}

BYTE ringReadByte(ring_generic* buf, BYTE offset)
{
    BYTE pos;
    pos = (buf->ptr_b + offset) & buf->max;
    return buf->data[pos];
}

BYTE ringLength(ring_generic* buf)
{
    return ((buf->ptr_e - buf->ptr_b) & buf->max) + (!!ringFull(buf) * (buf->max+1));
}

BOOL ringFull(ring_generic* buf)
{
    return (buf->ptr_b == buf->ptr_e) && (!buf->empty);
}

BOOL ringEmpty(ring_generic* buf)
{
    return (buf->ptr_b == buf->ptr_e) && (buf->empty);
}

BYTE ringFreeSpace(ring_generic* buf)
{
    return (((signed char)buf->ptr_b - buf->ptr_e) & buf->max) + (!!buf->empty * (buf->max+1));
}

BYTE ringDistance(ring_generic* buf, BYTE first, BYTE second)
{
    return ((second-first) & buf->max);
}

void ringSerialize(ring_generic* buf, BYTE* out, BYTE start, BYTE length)
{
    int i;
    for (i = 0; i < length; i++)
        out[i] = buf->data[(start+i) & buf->max];
}

void ringRemoveFromMiddle(ring_generic* buf, BYTE start, BYTE length)
{
    int i;
    for (i = start; i != ((buf->ptr_e-length)&buf->max); i++)
        buf->data[i%buf->max] = buf->data[(i+length)&buf->max];
    buf->ptr_e = i&buf->max;
    if (buf->ptr_b == buf->ptr_e) buf->empty = TRUE;
}

void ringAddToStart(ring_generic* buf, BYTE* data, BYTE len)
{
    int i;
    buf->ptr_b = (buf->ptr_b-len)&buf->max;
    for (i = 0; i < len; i++) {
        buf->data[(buf->ptr_b+i)&buf->max] = data[i];
        
        // check full buffer
        if (((buf->ptr_b+i)&buf->max) == buf->ptr_e) buf->ptr_e = buf->ptr_b;
    }
    if (len > 0) buf->empty = FALSE;
}

void ringClear(ring_generic* buf)
{
    buf->ptr_b = buf->ptr_e;
    buf->empty = TRUE;
}
