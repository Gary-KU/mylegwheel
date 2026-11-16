#include "robot_telemetry.h"
#include <string.h>
#include <math.h>
typedef char RobotTelemetryFloatMustBe32Bits[sizeof(float)==4 ? 1 : -1];
static void put_u32(uint8_t *out, uint32_t value)
{
    for (unsigned i=0;i<4;++i) out[i]=(uint8_t)(value>>(8U*i));
}
uint16_t RobotTelemetry_CRC16(const uint8_t *bytes, uint16_t length)
{
    uint16_t crc=0xFFFFU;
    for (unsigned i=0;i<length;++i) {
        crc^=(uint16_t)bytes[i]<<8;
        for (unsigned bit=0;bit<8;++bit)
            crc=(uint16_t)((crc&0x8000U) ? ((uint32_t)crc<<1)^0x1021U : (uint32_t)crc<<1);
    }
    return crc;
}
uint16_t RobotTelemetry_Encode(uint8_t *frame, uint16_t capacity,
                              const RobotTelemetrySample *sample)
{
    if (!frame || !sample || capacity<ROBOT_TELEMETRY_FRAME_BYTES) return 0;
    frame[0]='W';frame[1]='L';frame[2]=1;frame[3]=1;
    frame[4]=ROBOT_TELEMETRY_PAYLOAD_BYTES;frame[5]=0;
    unsigned at=6;
    for (unsigned i=0;i<ROBOT_TELEMETRY_META_WORDS;++i,at+=4)
        put_u32(frame+at,sample->meta[i]);
    for (unsigned i=0;i<ROBOT_TELEMETRY_CHANNELS;++i,at+=4) {
        uint32_t bits;
        memcpy(&bits,&sample->value[i],4);
        put_u32(frame+at,bits);
    }
    uint16_t crc=RobotTelemetry_CRC16(frame,(uint16_t)at);
    frame[at++]=(uint8_t)crc;frame[at++]=(uint8_t)(crc>>8);
    return (uint16_t)at;
}
uint16_t RobotTelemetry_EncodeAnoPair(uint8_t *frame, uint16_t capacity,
                                    const RobotTelemetrySample *sample, uint32_t pair)
{
    static const uint8_t indices[12][2]={
        {0,3},{0,1},{3,4},{6,7},{8,9},{10,11},
        {12,13},{16,17},{14,15},{5,2},{18,19},{20,21}};
    /* rad -> degrees*100; m -> mm*100; N*m -> mN*m*100; V/N -> *100. */
    static const float factors[12][2]={
        {5729.5779513f,100000.f},{5729.5779513f,5729.5779513f},
        {100000.f,100000.f},{5729.5779513f,5729.5779513f},
        {100000.f,100000.f},{100000.f,100000.f},
        {100000.f,100000.f},{100.f,100.f},{100.f,100.f},
        {100000.f,5729.5779513f},{100.f,100.f},{100.f,100.f}};
    if (!frame || !sample || capacity<22U || pair>=ROBOT_TELEMETRY_ANO_PAIRS) return 0;
    int32_t channel[2];
    for (unsigned i=0;i<2;++i) {
        double value;
        if (pair<12U) value=(double)sample->value[indices[pair][i]]*factors[pair][i];
        else if (pair==12U) value=(double)(i ? ((sample->meta[7]>>8)&31U) : sample->meta[4])*100.0;
        else if (pair==13U) value=i ? ((sample->meta[7]&4U) ? 100.0:0.0) :
            ((double)sample->value[3]-(double)sample->value[4])*100000.0;
        else if (pair==14U) value=(double)sample->swing_command_nm[i]*100000.0;
        else if (pair==15U) value=i ? (double)sample->value[13]*100000.0 : (double)sample->meta[4]*100.0;
        else if (pair<18U) value=(double)(i ? sample->leg_angle_rate_rad_s[pair-16U] :
            sample->value[6U+pair-16U])*5729.5779513f;
        else if (pair==18U) value=(double)sample->value[i ? 2U:0U]*5729.5779513f;
        else if (pair==19U) value=i ? (double)sample->value[13]*100000.0 : (double)sample->value[0]*5729.5779513f;
        else if (pair==20U) value=i ? (double)sample->value[5]*100000.0 :
            ((double)sample->value[3]-(double)sample->value[4])*100000.0;
        else if (pair==21U) value=(double)sample->value[i ? 17U:5U]*100000.0; /* mm/s, mV */
        else if (pair==22U) value=i ? (double)sample->value[0]*5729.5779513f :
            (double)sample->wheel_torque_command_nm*100000.0; /* mN*m, degrees */
        else if (pair==23U) value=(double)(i ? sample->value[16] :
            sample->wheel_torque_command_nm)*100000.0; /* mN*m, left command mV */
        else value=(double)sample->meta[i ? 5U:4U]*100.0; /* phase, last fault reason */
        if (!isfinite(value) || value>2147483647.0 || value<(-2147483647.0-1.0)) return 0;
        channel[i]=(int32_t)value;
    }
    memset(frame,0,22);
    frame[0]=0xAA;frame[1]=0xFF;frame[2]=0xF1;frame[3]=16;
    put_u32(frame+4,(uint32_t)channel[0]);put_u32(frame+8,(uint32_t)channel[1]);
    uint8_t sc=0,ac=0;
    for (unsigned i=0;i<20;++i) { sc=(uint8_t)(sc+frame[i]);ac=(uint8_t)(ac+sc); }
    frame[20]=sc;frame[21]=ac;
    return 22;
}
uint16_t RobotTelemetry_EncodeAnoStatus(uint8_t *frame, uint16_t capacity, uint32_t status)
{
    if (!frame || capacity<10U) return 0;
    frame[0]=0xAA;frame[1]=0xFF;frame[2]=0xF2;frame[3]=4;
    put_u32(frame+4,status);
    uint8_t sc=0,ac=0;
    for (unsigned i=0;i<8;++i) {sc=(uint8_t)(sc+frame[i]);ac=(uint8_t)(ac+sc);}
    frame[8]=sc;frame[9]=ac;return 10;
}
