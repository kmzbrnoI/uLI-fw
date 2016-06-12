/*
 * Ring buffer header file.
 * (c) Michal Petrilak, Jan Horacek 2016
 * Version: 1.0.1
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include "GenericTypeDefs.h"

typedef struct {
    BYTE max;      // Maximmum index (buffer of 8 items has max 7)
    BYTE ptr_b;    // pointer to begin (for 8 items 0..7)
    BYTE ptr_e;    // pointer to end (for 8 items 0..7)
    BYTE data[32]; // data
    BOOL empty;    // wheter buffer is empty
} ring_generic;

/* Warning: ring buffer suffers from several problems, which
 * are very important to understand when working with ring buffer.
 * Read notes below.
 */

/* ptr_b points to first byte
 * ptr_e points to byte after last byte
 * This specially implies that is it NOT POSSIBLE to differentiate empty and full buffer.
 * This is why buffer contains special \empty flag.
 */

/* Common situations:
 * full buffer: ptr_b == ptr_e && !empty
 * empty buffer: ptr_b == ptr_e && empty
 * Empty flag must be set when manipulating with ring buffer!
 */

void ringAddByte(ring_generic *buf, BYTE dat);
BYTE ringRemoveByte(ring_generic *buf);
void ringRemoveFrame(ring_generic* buf, BYTE num);
BYTE ringReadByte(ring_generic* buf, BYTE offset);
BYTE ringLength(ring_generic* buf);
BOOL ringFull(ring_generic* buf);
BOOL ringEmpty(ring_generic* buf);
BYTE ringDistance(ring_generic* buf, BYTE first, BYTE second);
void ringSerialize(ring_generic* buf, BYTE* out, BYTE start, BYTE length);
void ringRemoveFromMiddle(ring_generic* buf, BYTE start, BYTE length);
void ringClear(ring_generic* buf);
BYTE ringFreeSpace(ring_generic* buf);
void ringAddToStart(ring_generic* buf, BYTE* data, BYTE len);
    // this function probably misbihaves, 

//#define ringBufferAlloc(name, size) typedef struct { BYTE max; BYTE ptr_b; BYTE ptr_e; BYTE data[size]; } ## T ## name ; T ## name name;
#define ringBufferInit(name, size) name ## . ## max = (size-1); name ## . ## ptr_b = 0; name ## . ## ptr_e = 0; name ## . ## empty = TRUE;

#endif

