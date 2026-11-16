#include "lqr_startup_record.h"
#include "main.h"
#include <stddef.h>
#include <string.h>
#if FLASH_NB_32BITWORD_IN_FLASHWORD != 8U
#error Record storage requires this STM32H723 32-byte Flash word layout
#endif
int LqrRecordStorage_Load(LqrRecordHeader *header)
{
    const LqrRecordData *flash=(const LqrRecordData *)LQR_RECORD_FLASH_ADDRESS;
    if(flash->header.magic!=LQR_RECORD_MAGIC ||
       flash->header.commit!=LQR_RECORD_COMMIT || !flash->header.count) return 0;
    /* 历史版本的行宽和数据起点不同，必须按保存版本计算 CRC 和边界。 */
    uint32_t row_bytes,capacity,payload_offset,configuration_bytes=0;
    if(flash->header.version==1U || flash->header.version==2U) {
        row_bytes=160U;capacity=1536U;payload_offset=128U;
    } else if(flash->header.version==3U) {
        row_bytes=288U;capacity=896U;payload_offset=384U;configuration_bytes=256U;
    } else if(flash->header.version==4U) {
        row_bytes=320U;capacity=800U;payload_offset=384U;configuration_bytes=256U;
    } else if(flash->header.version==LQR_RECORD_VERSION) {
        row_bytes=sizeof(LqrRecordRow);capacity=LQR_RECORD_ROWS;
        payload_offset=offsetof(LqrRecordData,rows);configuration_bytes=sizeof(LqrRecordConfiguration);
    } else return 0;
    if(flash->header.row_bytes!=row_bytes || flash->header.capacity!=capacity ||
       flash->header.count>capacity) return 0;
    uint32_t crc=LqrStartupRecord_CRC32(UINT32_MAX,&flash->header,offsetof(LqrRecordHeader,crc32));
    if(flash->header.version>=3U)
        crc=LqrStartupRecord_CRC32(crc,(const uint8_t *)flash+128U,configuration_bytes);
    crc=LqrStartupRecord_CRC32(crc,(const uint8_t *)flash+payload_offset,flash->header.count*row_bytes);
    if((crc^UINT32_MAX)!=flash->header.crc32) return 0;
    *header=flash->header;return 1;
}
static int program(uint32_t address,const void *source,uint32_t bytes,uint32_t *error)
{
    for(uint32_t offset=0;offset<bytes;offset+=32U) {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,address+offset,
                             (uintptr_t)((const uint8_t *)source+offset))!=HAL_OK) {
            *error=HAL_FLASH_GetError();return 0;
        }
    }
    return 1;
}
int LqrRecordStorage_Save(const volatile LqrRecordData *record,uint32_t *error)
{
    /* Flash 擦写会阻塞执行；这里要求存储占用标记有效，并关闭下列机器人控制入口。 */
    if(!record || !error || !g_lqr_record.storage_busy || g_robot_control.enable ||
       g_dual_hold.enable || g_balance.enable || g_wheel_test.enable || g_wheel_test.zero_request ||
       (g_robot_control.remote_enable && g_robot_control.remote_switch_on) ||
       record->header.version!=LQR_RECORD_VERSION || record->header.row_bytes!=sizeof(LqrRecordRow) ||
       record->header.capacity!=LQR_RECORD_ROWS || !record->header.count || record->header.count>LQR_RECORD_ROWS) return 0;
    *error=0;
    if(HAL_FLASH_Unlock()!=HAL_OK) { *error=HAL_FLASH_GetError();return 0; }
    FLASH_EraseInitTypeDef erase={0};uint32_t sector_error=UINT32_MAX;
    erase.TypeErase=FLASH_TYPEERASE_SECTORS;erase.Banks=FLASH_BANK_1;
    erase.Sector=FLASH_SECTOR_6;erase.NbSectors=2U;erase.VoltageRange=FLASH_VOLTAGE_RANGE_3;
    int ok=HAL_FLASHEx_Erase(&erase,&sector_error)==HAL_OK;
    if(!ok) *error=HAL_FLASH_GetError();
    if(ok) ok=program(LQR_RECORD_FLASH_ADDRESS+offsetof(LqrRecordData,rows),(const void *)record->rows,
                     record->header.count*sizeof(LqrRecordRow),error);
    if(ok) ok=program(LQR_RECORD_FLASH_ADDRESS+sizeof(LqrRecordHeader),
                     (const void *)&record->configuration,sizeof(LqrRecordConfiguration),error);
    if(ok) ok=program(LQR_RECORD_FLASH_ADDRESS,(const void *)&record->header,96U,error);
    /* 最后才写含 CRC 和 commit 的 32 字节 Flash 字，掉电半写记录不会被接收。 */
    if(ok) ok=program(LQR_RECORD_FLASH_ADDRESS+96U,
                     (const uint8_t *)&record->header+96U,32U,error);
    (void)HAL_FLASH_Lock();
    if(ok) {
        LqrRecordHeader check;
        ok=LqrRecordStorage_Load(&check) && check.crc32==record->header.crc32;
        if(!ok) *error=0x80000000U;
    }
    return ok;
}
