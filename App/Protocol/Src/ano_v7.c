/**
  ******************************************************************************
  * @file    ano_v7.c
  * @brief   匿名上位机 V7 (ANO Tech) 通信协议实现
  *
  * 帧格式: 0xAA 0xFF FUN LEN DATA[LEN] SC AC
  *   SC = 从 0xAA 开始累加所有字节的和
  *   AC = 每次累加 SC 的二次累加和
  ******************************************************************************
  */
#include "ano_v7.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* 发送缓冲区 */
/* Each caller builds its own frame; CDC copies it before returning. */

/* 接收缓冲区与状态机 */
static uint8_t ano_rx_buf[64];
static uint8_t ano_rx_sta = 0;
static uint8_t ano_rx_len = 0;
static uint8_t ano_rx_cnt = 0;

/* 内部：组帧并发送 */
void ano_v7_send(uint8_t fun, const uint8_t *data, uint8_t len)
{
    uint8_t ano_tx_buf[64];
    uint8_t cnt = 0;
    uint8_t sc = 0, ac = 0;

    if (len > 50U)
    {
        len = 50U;
    }

    ano_tx_buf[cnt++] = ANO_HEADER1;    /* 0xAA */
    ano_tx_buf[cnt++] = ANO_HEADER2;    /* 0xFF */
    ano_tx_buf[cnt++] = fun;
    ano_tx_buf[cnt++] = len;
    for (uint8_t i = 0; i < len; i++)
    {
        ano_tx_buf[cnt++] = data[i];
    }

    /* 双重校验：SC 累加和，AC 二次累加和（均包含帧头） */
    for (uint8_t i = 0; i < cnt; i++)
    {
        sc = (uint8_t)(sc + ano_tx_buf[i]);
        ac = (uint8_t)(ac + sc);
    }
    ano_tx_buf[cnt++] = sc;
    ano_tx_buf[cnt++] = ac;

    CDC_Transmit_HS(ano_tx_buf, cnt);
}

/* 发送 4 个 int32 数据帧 */
void ano_v7_send_data(int32_t a, int32_t b, int32_t c, int32_t d)
{
    uint8_t data[16];

    data[0]  = (uint8_t)(a & 0xFF);
    data[1]  = (uint8_t)((a >> 8) & 0xFF);
    data[2]  = (uint8_t)((a >> 16) & 0xFF);
    data[3]  = (uint8_t)((a >> 24) & 0xFF);

    data[4]  = (uint8_t)(b & 0xFF);
    data[5]  = (uint8_t)((b >> 8) & 0xFF);
    data[6]  = (uint8_t)((b >> 16) & 0xFF);
    data[7]  = (uint8_t)((b >> 24) & 0xFF);

    data[8]  = (uint8_t)(c & 0xFF);
    data[9]  = (uint8_t)((c >> 8) & 0xFF);
    data[10] = (uint8_t)((c >> 16) & 0xFF);
    data[11] = (uint8_t)((c >> 24) & 0xFF);

    data[12] = (uint8_t)(d & 0xFF);
    data[13] = (uint8_t)((d >> 8) & 0xFF);
    data[14] = (uint8_t)((d >> 16) & 0xFF);
    data[15] = (uint8_t)((d >> 24) & 0xFF);

    ano_v7_send(ANO_FUN_DATA, data, 16);
}

/* 发送 2 个 int32 数据帧 */
void ano_v7_send_data2(int32_t a, int32_t b)
{
    uint8_t data[8];

    data[0] = (uint8_t)(a & 0xFF);
    data[1] = (uint8_t)((a >> 8) & 0xFF);
    data[2] = (uint8_t)((a >> 16) & 0xFF);
    data[3] = (uint8_t)((a >> 24) & 0xFF);

    data[4] = (uint8_t)(b & 0xFF);
    data[5] = (uint8_t)((b >> 8) & 0xFF);
    data[6] = (uint8_t)((b >> 16) & 0xFF);
    data[7] = (uint8_t)((b >> 24) & 0xFF);

    ano_v7_send(ANO_FUN_DATA, data, 8);
}

/* 发送参数帧 */
void ano_v7_send_param(uint16_t id, int32_t value)
{
    uint8_t data[6];

    data[0] = (uint8_t)(id & 0xFF);
    data[1] = (uint8_t)((id >> 8) & 0xFF);
    data[2] = (uint8_t)(value & 0xFF);
    data[3] = (uint8_t)((value >> 8) & 0xFF);
    data[4] = (uint8_t)((value >> 16) & 0xFF);
    data[5] = (uint8_t)((value >> 24) & 0xFF);

    ano_v7_send(ANO_FUN_PARAM_WR, data, 6);
}

/* 发送校验回传帧 */
void ano_v7_send_check(uint8_t id, uint8_t sc, uint8_t ac)
{
    uint8_t data[3];

    data[0] = id;
    data[1] = sc;
    data[2] = ac;

    ano_v7_send(ANO_FUN_CHECK, data, 3);
}

/* 内部：解析一帧并响应参数请求 */
static void ano_v7_parse(uint8_t *p, uint8_t body_len)
{
    uint8_t sc = 0, ac = 0;

    /* 校验 body[0..body_len-1]，SC 在 [body_len]，AC 在 [body_len+1] */
    for (uint8_t i = 0; i < body_len; i++)
    {
        sc = (uint8_t)(sc + p[i]);
        ac = (uint8_t)(ac + sc);
    }
    if (sc != p[body_len] || ac != p[body_len + 1])
    {
        return;     /* 校验失败，丢弃 */
    }

    uint8_t fun = p[2];

    if (fun == ANO_FUN_PARAM_RD && body_len >= 6U)        /* 上位机读取参数 */
    {
        uint16_t id = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
        /* 这里固定回 0；需要真实参数时替换成你的参数值 */
        ano_v7_send_param(id, 0);
    }
    else if (fun == ANO_FUN_PARAM_WR && body_len >= 10U)   /* 上位机写入参数 */
    {
        /* 可在此保存参数： id = p[4]|p[5]<<8, value = p[6..9] */
        ano_v7_send_check(fun, p[body_len], p[body_len + 1]);
    }
}

/* 接收状态机：逐字节喂入 */
void ano_v7_parse_byte(uint8_t data)
{
    switch (ano_rx_sta)
    {
        case 0:     /* 等帧头1 */
            if (data == ANO_HEADER1)
            {
                ano_rx_buf[0] = data;
                ano_rx_sta = 1;
            }
            break;

        case 1:     /* 帧头2 */
            if (data == ANO_HEADER2)
            {
                ano_rx_buf[1] = data;
                ano_rx_sta = 2;
            }
            else
            {
                ano_rx_sta = 0;
            }
            break;

        case 2:     /* 功能字 */
            ano_rx_buf[2] = data;
            ano_rx_sta = 3;
            break;

        case 3:     /* 长度 */
            ano_rx_buf[3] = data;
            ano_rx_len = data;
            ano_rx_cnt = 0;
            if (ano_rx_len > sizeof(ano_rx_buf) - 6U) {
                ano_rx_sta = 0;
            } else {
                ano_rx_sta = ano_rx_len == 0U ? 5U : 4U;
            }
            break;

        case 4:     /* 数据 */
            ano_rx_buf[4 + ano_rx_cnt++] = data;
            if (ano_rx_cnt >= ano_rx_len)
            {
                ano_rx_sta = 5;
            }
            break;

        case 5:     /* SC */
            ano_rx_buf[4 + ano_rx_cnt++] = data;
            ano_rx_sta = 6;
            break;

        case 6:     /* AC，完成一帧 */
            ano_rx_buf[4 + ano_rx_cnt++] = data;
            ano_rx_sta = 0;
            ano_v7_parse(ano_rx_buf, 4 + ano_rx_len);
            break;

        default:
            ano_rx_sta = 0;
            break;
    }
}

/* 演示：每 10ms 发一帧 4 个 int32 测试数据 */
void ano_v7_demo(void)
{
    static uint32_t last_tick = 0;
    static uint32_t cnt = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_tick) >= 10U)
    {
        last_tick = now;
        cnt++;

        int32_t a = (int32_t)cnt;
        int32_t b = (int32_t)(cnt % 1000U);
        int32_t c = (int32_t)(cnt / 1000U);
        int32_t d = 0;

        ano_v7_send_data(a, b, c, d);
    }
}
