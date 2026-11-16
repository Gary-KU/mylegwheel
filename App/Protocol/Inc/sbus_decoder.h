#ifndef SBUS_DECODER_H
#define SBUS_DECODER_H

#include <stdint.h>

#define SBUS_CHANNEL_COUNT 16U
#define SBUS_FRAME_BYTES 25U

typedef struct {
    uint16_t channel[SBUS_CHANNEL_COUNT]; /* Raw 11-bit values: 0..2047. */
    uint8_t channel17;
    uint8_t channel18;
    uint8_t frame_lost;
    uint8_t failsafe;
} SbusFrame;

typedef struct {
    uint8_t bytes[SBUS_FRAME_BYTES];
    uint8_t used;
    uint32_t bad_frames;
} SbusDecoder;

void SBUS_DecoderInit(SbusDecoder *decoder);
/* Handles fragments, back-to-back frames and resynchronization. Returns 1
 * only after a complete, structurally valid frame was written to output.
 */
uint8_t SBUS_DecoderFeed(SbusDecoder *decoder, uint8_t byte, SbusFrame *output);

#endif /* SBUS_DECODER_H */
