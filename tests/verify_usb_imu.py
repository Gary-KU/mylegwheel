"""Compile actual CDC/ANO sources with USB mocks; run via Python + gcc."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
stub = r'''
#ifndef USB_TEST_STUB_H
#define USB_TEST_STUB_H
#include <stdint.h>
#include <stddef.h>
#define __USBD_CDC_IF_H__
#define APP_RX_DATA_SIZE 2048
#define APP_TX_DATA_SIZE 2048
#define UNUSED(x) (void)(x)
#define USBD_OK 0
#define USBD_BUSY 1
#define USBD_FAIL 2
#define USBD_STATE_CONFIGURED 3
enum {CDC_SEND_ENCAPSULATED_COMMAND, CDC_GET_ENCAPSULATED_RESPONSE,
CDC_SET_COMM_FEATURE, CDC_GET_COMM_FEATURE, CDC_CLEAR_COMM_FEATURE,
CDC_SET_LINE_CODING, CDC_GET_LINE_CODING, CDC_SET_CONTROL_LINE_STATE, CDC_SEND_BREAK};
typedef struct {uint32_t TxState;} USBD_CDC_HandleTypeDef;
typedef struct {void *pClassData; uint8_t dev_state;} USBD_HandleTypeDef;
typedef struct {
int8_t (*Init)(void); int8_t (*DeInit)(void);
int8_t (*Control)(uint8_t,uint8_t*,uint16_t);
int8_t (*Receive)(uint8_t*,uint32_t*);
int8_t (*TransmitCplt)(uint8_t*,uint32_t*,uint8_t);
} USBD_CDC_ItfTypeDef;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
uint8_t CDC_Transmit_HS(uint8_t*,uint16_t);
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef*,uint8_t*,uint16_t);
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef*,uint8_t*);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef*);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef*);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
uint32_t HAL_GetTick(void);
#endif
'''
harness = r'''
#include "main.h"
#include "ano_v7.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
USBD_HandleTypeDef hUsbDeviceHS;
static USBD_CDC_HandleTypeDef cdc;
static uint8_t *tx;
static uint16_t length;
static uint32_t mask;
uint32_t __get_PRIMASK(void) {return mask;}
void __disable_irq(void) {mask=1;}
void __set_PRIMASK(uint32_t m) {mask=m;}
uint32_t HAL_GetTick(void) {return 0;}
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *h,uint8_t *b,uint16_t n)
{(void)h; tx=b; length=n; return 0;}
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *h,uint8_t *b)
{(void)h;(void)b;return 0;}
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *h) {(void)h;return 0;}
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *h)
{((USBD_CDC_HandleTypeDef*)h->pClassData)->TxState=1;return 0;}
static void feed(uint8_t *p,unsigned n)
{unsigned i;for(i=0;i<n;++i)ano_v7_parse_byte(p[i]);}
int main(void)
{
    uint8_t buf[64]={1,2,3}, saved[64], line[7]={0};
    uint8_t sc=0,ac=0;
    unsigned i;
    assert(CDC_Transmit_HS(buf,3)==USBD_FAIL); /* no USB enumeration */
    hUsbDeviceHS.dev_state=USBD_STATE_CONFIGURED;
    assert(CDC_Transmit_HS(buf,3)==USBD_FAIL); /* null class */
    hUsbDeviceHS.pClassData=&cdc;
    assert(CDC_Transmit_HS(buf,3)==USBD_OK && mask==0);
    buf[0]=99;
    assert(tx[0]==1 && tx!=buf); /* async buffer lifetime */
    assert(CDC_Transmit_HS(buf,3)==USBD_BUSY && tx[0]==1);
    cdc.TxState=0; mask=1;
    ano_v7_send_data(-100,200,-300,4000);
    assert(mask==1 && length==22);
    assert(tx[0]==0xAA && tx[1]==0xFF && tx[2]==0xF1 && tx[3]==16);
    assert(tx[4]==0x9C && tx[5]==0xFF && tx[6]==0xFF && tx[7]==0xFF);
    assert(tx[16]==0xA0 && tx[17]==0x0F);
    for(i=0;i<20;++i){sc=(uint8_t)(sc+tx[i]);ac=(uint8_t)(ac+sc);}
    assert(tx[20]==sc && tx[21]==ac);
    memcpy(saved,tx,22);
    ano_v7_send_data(0,0,0,0);
    assert(memcmp(saved,tx,22)==0); /* busy caller cannot corrupt frame */
    USBD_Interface_fops_HS.Control(CDC_GET_LINE_CODING,line,7);
    assert(line[0]==0 && line[1]==0xC2 && line[2]==1 && line[6]==8);
    line[0]=0x80;line[1]=0x25;line[2]=0;
    USBD_Interface_fops_HS.Control(CDC_SET_LINE_CODING,line,7);
    memset(line,0,7);
    USBD_Interface_fops_HS.Control(CDC_GET_LINE_CODING,line,7);
    assert(line[0]==0x80 && line[1]==0x25);
    /* Oversized length, zero-payload packet, then a valid parameter query. */
    {uint8_t bad[]={0xAA,0xFF,0xE1,255};feed(bad,sizeof(bad));}
    {uint8_t empty[]={0xAA,0xFF,0xE1,0,0x8A,0xBB};feed(empty,sizeof(empty));}
    cdc.TxState=0;
    {uint8_t query[]={0xAA,0xFF,0xE1,2,1,0,0,0};
     sc=ac=0;for(i=0;i<6;++i){sc+=query[i];ac+=sc;}
     query[6]=sc;query[7]=ac;feed(query,sizeof(query));}
    assert(cdc.TxState==1 && tx[2]==0xE2 && length==12 && tx[4]==1);
    hUsbDeviceHS.pClassData=NULL; hUsbDeviceHS.dev_state=0;
    assert(CDC_Transmit_HS(buf,3)==USBD_FAIL); /* unplug */
    puts("PASS: CDC disconnect/busy/copy/IRQ state, ANO F1 payload/checksum, line coding, parser recovery");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix="usb_imu_test_") as folder:
    temp = Path(folder)
    (temp / "main.h").write_text(stub)
    (temp / "harness.c").write_text(harness)
    exe = temp / "verify.exe"
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-include", str(temp / "main.h"), "-I", str(temp),
                    "-I", str(root / "Core/Inc"), "-I", str(root / "USB_DEVICE/App"),
                    "-I", str(root / "App/Protocol/Inc"),
                    str(root / "App/Protocol/Src/ano_v7.c"),
                    str(root / "USB_DEVICE/App/usbd_cdc_if.c"),
                    str(temp / "harness.c"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
