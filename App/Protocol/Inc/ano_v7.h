/**
  ******************************************************************************
  * @file    ano_v7.h
  * @brief   匿名上位机 V7 (ANO Tech) 通信协议接口
  *
  * 帧格式:
  *   帧头0  帧头1  功能字  长度    数据           校验SC  校验AC
  *   0xAA   0xFF   FUN    LEN   DATA[LEN]        SC     AC
  *   SC = 从帧头开始所有字节累加和 (含 0xAA 0xFF)
  *   AC = 每次累加 SC 后的二次累加和
  ******************************************************************************
  */
#ifndef ANO_V7_H
#define ANO_V7_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 帧头 */
#define ANO_HEADER1          0xAA
#define ANO_HEADER2          0xFF

/* 功能字 FUN */
#define ANO_FUN_DATA         0xF1    /* 数据发送（4个 int32） */
#define ANO_FUN_PARAM_RD     0xE1    /* 参数读取请求（上位机->设备） */
#define ANO_FUN_PARAM_WR     0xE2    /* 参数发送/写入 */
#define ANO_FUN_CHECK        0x00    /* 校验回传 */

/**
  * @brief 通用发送：自动组帧 (0xAA 0xFF FUN LEN DATA SC AC) 并经 USB CDC 发出
  * @param fun  功能字
  * @param data 数据区首地址
  * @param len  数据区字节数 (<=50)
  */
void ano_v7_send(uint8_t fun, const uint8_t *data, uint8_t len);

/**
  * @brief 发送数据帧 (FUN=0xF1)，4 个 int32（小端）
  */
void ano_v7_send_data(int32_t a, int32_t b, int32_t c, int32_t d);

/**
  * @brief 发送数据帧 (FUN=0xF1)，2 个 int32（小端）
  */
void ano_v7_send_data2(int32_t a, int32_t b);

/**
  * @brief 发送参数帧 (FUN=0xE2)：id(2字节) + value(4字节)
  */
void ano_v7_send_param(uint16_t id, int32_t value);

/**
  * @brief 发送校验回传帧 (FUN=0x00)
  */
void ano_v7_send_check(uint8_t id, uint8_t sc, uint8_t ac);

/**
  * @brief 接收字节解析（在 USB CDC 接收回调里逐字节调用）
  *        自动响应上位机的「读取参数」请求
  */
void ano_v7_parse_byte(uint8_t data);

/**
  * @brief 演示：每 10ms 发一帧测试数据（可自行替换/删除）
  */
void ano_v7_demo(void);

#ifdef __cplusplus
}
#endif

#endif /* ANO_V7_H */
