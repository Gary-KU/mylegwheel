#include "sbus_decoder.h"
#include <string.h>

void SBUS_DecoderInit(SbusDecoder *decoder)
{
    memset(decoder, 0, sizeof(*decoder));
}

static uint8_t SBUS_FooterValid(uint8_t footer)
{
    /* Standard SBUS, plus the four SBUS2 frame/telemetry slot identifiers.
     * Sources: bolderflight/sbus and PX4 src/lib/rc/sbus.cpp.
     */
    return footer == 0x00U || footer == 0x04U || footer == 0x14U ||
           footer == 0x24U || footer == 0x34U;
}

uint8_t SBUS_DecoderFeed(SbusDecoder *decoder, uint8_t byte, SbusFrame *output)
{
    uint32_t channel;
    if (decoder->used == 0U && byte != 0x0FU) return 0U;
    decoder->bytes[decoder->used++] = byte;
    if (decoder->used < SBUS_FRAME_BYTES) return 0U;
    if (SBUS_FooterValid(decoder->bytes[24]) &&
        (decoder->bytes[23] & 0xF0U) == 0U) {
        /* 16 路模拟通道按小端位流连续打包，每路 11 bit，可能跨越三个字节。 */
        for (channel = 0U; channel < SBUS_CHANNEL_COUNT; ++channel) {
            uint32_t bit = channel * 11U;
            uint32_t index = 1U + bit / 8U;
            uint32_t word = decoder->bytes[index] |
                ((uint32_t)decoder->bytes[index + 1U] << 8);
            if (index + 2U <= 22U) {
                word |= (uint32_t)decoder->bytes[index + 2U] << 16;
            }
            output->channel[channel] = (uint16_t)((word >> (bit % 8U)) & 0x7FFU);
        }
        output->channel17 = decoder->bytes[23] & 1U;
        output->channel18 = (decoder->bytes[23] >> 1) & 1U;
        output->frame_lost = (decoder->bytes[23] >> 2) & 1U;
        output->failsafe = (decoder->bytes[23] >> 3) & 1U;
        decoder->used = 0U;
        return 1U;
    }
    ++decoder->bad_frames;
    /* 坏帧内部可能已含下一帧头，保留其后字节，避免丢字节后持续错位。 */
    for (channel = 1U; channel < SBUS_FRAME_BYTES; ++channel) {
        if (decoder->bytes[channel] == 0x0FU) {
            decoder->used = (uint8_t)(SBUS_FRAME_BYTES - channel);
            memmove(decoder->bytes, decoder->bytes + channel, decoder->used);
            return 0U;
        }
    }
    decoder->used = 0U;
    return 0U;
}
