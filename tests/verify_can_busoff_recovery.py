"""Run the actual CAN task's stopped-only bus-off state machine with HAL mocks."""
import ast
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'App/Tasks/Src/can_task.c').read_text()
tree = ast.parse((root / 'tests/verify_can_feedback.py').read_text())
runtime = next(ast.literal_eval(n.value) for n in tree.body
               if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'runtime' for t in n.targets))
runtime = runtime.replace('typedef struct { uint32_t ErrorCode; } FDCAN_HandleTypeDef;', '''
typedef struct { uint32_t CCCR, IR, TXBRP; } FDCAN_GlobalTypeDef;
typedef struct { uint32_t ErrorCode, State; FDCAN_GlobalTypeDef *Instance; } FDCAN_HandleTypeDef;
#define HAL_FDCAN_STATE_READY 1U
#define HAL_FDCAN_STATE_BUSY 2U
#define HAL_FDCAN_STATE_ERROR 3U
''')
runtime = runtime.replace('#endif', '''
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *, uint32_t);
void test_clear_flags(FDCAN_HandleTypeDef *, uint32_t);
#define __HAL_FDCAN_CLEAR_FLAG(h,f) test_clear_flags(h,f)
#endif
''')
headers = (root / 'Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h').read_text()
headers += '\n' + (root / 'Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h723xx.h').read_text()
names = sorted(set(re.findall(r'\bFDCAN_[A-Z][A-Z0-9_]+\b', source)))
names.append('HAL_FDCAN_ERROR_NONE')
defines, seen = [], set()
for name in names:
    if name in seen:
        continue
    seen.add(name)
    match = re.search(r'^#define\s+' + name + r'\s+([^\r\n]+)', headers, re.M)
    assert match, name
    defines.append('#define ' + name + ' ' + match[1])
    names.extend(re.findall(r'\bFDCAN_[A-Za-z0-9_]+\b', match[1].split('/*')[0]))
runtime = runtime.replace('#endif', '\n'.join(defines) + '\n#endif')

harness = r'''
#include "test_runtime.h"
#include "can_task.h"
#include <stdio.h>
#include <string.h>
unsigned critical_depth;
static FDCAN_GlobalTypeDef hw;
FDCAN_HandleTypeDef hfdcan1;
static uint32_t now, queued, stop_calls, start_calls, notice, invalidations;
static uint32_t fault_notices, aborts, decoded, stopped, refill;
static uint32_t fail_stop, fail_start, fail_notice, fail_read;
static FDCAN_ProtocolStatusTypeDef protocol;
uint32_t HAL_GetTick(void) { return now; }
void Error_Handler(void) { assert(!"unexpected fatal HAL error"); }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b,uint32_t c,uint32_t d) {
    assert(h==&hfdcan1 && a==FDCAN_ACCEPT_IN_RX_FIFO0 && b==FDCAN_REJECT);
    assert(c==FDCAN_REJECT_REMOTE && d==FDCAN_REJECT_REMOTE); return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *h,uint32_t m,uint32_t line) {
    (void)m; assert(h==&hfdcan1 && line==FDCAN_INTERRUPT_LINE0); return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,uint32_t mask,uint32_t tx) {
    assert(h==&hfdcan1 && tx==0); if(fail_notice) return HAL_ERROR; notice=mask; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *h,uint32_t mask) {
    assert(h==&hfdcan1); notice&=~mask; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h) {
    assert(h==&hfdcan1 && h->State==HAL_FDCAN_STATE_BUSY && CAN_CommunicationBlocked());
    ++stop_calls; h->Instance->CCCR|=FDCAN_CCCR_INIT;
    if(fail_stop) {h->State=HAL_FDCAN_STATE_ERROR; h->ErrorCode=123; return HAL_ERROR;}
    h->State=HAL_FDCAN_STATE_READY; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) {
    assert(h==&hfdcan1 && h->State==HAL_FDCAN_STATE_READY && CAN_CommunicationBlocked());
    ++start_calls; if(fail_start) return HAL_ERROR;
    h->State=HAL_FDCAN_STATE_BUSY; h->Instance->CCCR&=~FDCAN_CCCR_INIT; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *p) {
    assert(h==&hfdcan1); *p=protocol; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h,FDCAN_ErrorCountersTypeDef *p) {
    assert(h==&hfdcan1); *p=(FDCAN_ErrorCountersTypeDef){0}; return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h,uint32_t fifo) {
    assert(h==&hfdcan1 && fifo==FDCAN_RX_FIFO0); return queued;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h,uint32_t fifo,FDCAN_RxHeaderTypeDef *p,uint8_t *d) {
    assert(h==&hfdcan1 && h->State==HAL_FDCAN_STATE_BUSY && fifo==FDCAN_RX_FIFO0 && queued);
    if(fail_read) return HAL_ERROR;
    --queued; if(refill) ++queued;
    *p=(FDCAN_RxHeaderTypeDef){0x181,FDCAN_STANDARD_ID,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_8,FDCAN_CLASSIC_CAN};
    memset(d,0,64); d[0]=0xa1; return HAL_OK;
}
void test_clear_flags(FDCAN_HandleTypeDef *h,uint32_t mask) { assert(h==&hfdcan1); h->Instance->IR&=~mask; }
void vTaskNotifyGiveFromISR(TaskHandle_t t,BaseType_t *wake) {(void)t; *wake=pdFALSE;}
uint32_t ulTaskNotifyTake(BaseType_t c,TickType_t t) {(void)c;(void)t;return 0;}
void vTaskDelay(TickType_t t) {(void)t;}
BaseType_t xTaskCreate(void (*f)(void *),const char *n,unsigned s,void *a,unsigned p,TaskHandle_t *h) {
    (void)f;(void)n;(void)s;(void)a;(void)p;*h=NULL;return pdPASS;
}
void JointCalibration_Apply(MotorFeedback *f) {(void)f;}
void MotorFoc_Init(void) {}
void MotorFoc_InvalidateFromISR(void) {++invalidations;}
uint32_t MotorFoc_OnFrame(uint32_t id,const uint8_t d[8],uint32_t stamp) {
    (void)id;(void)d;(void)stamp;++decoded;return 1;
}
uint8_t MotorFoc_GetFeedback(uint8_t id,MotorFeedback *f) {memset(f,0,sizeof(*f));f->motor_id=id;f->online=1;return 1;}
uint8_t MotorFoc_GetFeedbackWithHealth(uint8_t id,MotorFeedback *f,MotorFeedbackHealth *health) {
    if(health) {memset(health,0,sizeof(*health));health->valid=1;health->motor_id=id;}
    return MotorFoc_GetFeedback(id,f);
}
void MotorCommand_OnTransportFault(void) {++fault_notices;}
uint32_t MotorCommand_CommunicationStopped(void) {return stopped;}
void FdcanDeadline_AbortAll(void) {++aborts;}
#include "CAN_SOURCE"

static void reset(uint32_t stamp) {
    now=stamp; queued=stop_calls=start_calls=notice=invalidations=0;
    fault_notices=aborts=decoded=stopped=refill=0;
    fail_stop=fail_start=fail_notice=fail_read=0;
    protocol=(FDCAN_ProtocolStatusTypeDef){0}; hw=(FDCAN_GlobalTypeDef){0};
    hfdcan1=(FDCAN_HandleTypeDef){0,HAL_FDCAN_STATE_BUSY,&hw};
    CAN_Task_Init(); assert(!CAN_CommunicationBlocked());
}
static void busoff(void) {
    protocol.BusOff=1; hw.CCCR|=FDCAN_CCCR_INIT;
    HAL_FDCAN_ErrorStatusCallback(&hfdcan1,FDCAN_IT_BUS_OFF);
    assert(CAN_CommunicationBlocked());
}
static void begin_attempt(void) {
    stopped=1; assert(CAN_ServiceRecovery());
    now+=199; assert(CAN_ServiceRecovery()); assert(start_calls==0);
    ++now; assert(CAN_ServiceRecovery()); assert(start_calls==1);
    assert(g_motor_can.recovery_state==2 && !notice);
}
int main(void) {
    MotorFeedback f;
    reset(100); busoff();
    assert(CAN_ServiceRecovery() && fault_notices==1 && aborts==1);
    now+=1000; assert(CAN_ServiceRecovery() && !start_calls); /* owner still active */
    queued=4; begin_attempt();
    assert(stop_calls==1 && g_motor_can.recovery_discarded_frames==4 && !decoded);
    assert(!MotorCAN_GetFeedback(1,&f) && !f.online);
    MotorFeedbackHealth health;
    assert(!MotorCAN_GetFeedbackWithHealth(1,&f,&health) && !f.online && health.valid);
    assert(health.reject_mask==MOTOR_FEEDBACK_REJECT_COMMUNICATION &&
           health.communication_blocked && health.recovery_state==g_motor_can.recovery_state);
    FDCAN_RxHeaderTypeDef legacy={0x104,FDCAN_STANDARD_ID,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_8,FDCAN_CLASSIC_CAN};
    uint8_t old_legacy[8]={0};
    CAN_ProcessFeedback(&legacy,old_legacy,receive_epoch);
    assert(!g_motor_can.motor[3].rx_count && !MotorCAN_GetFeedback(4,&f));
    ++now; assert(CAN_ServiceRecovery()); /* Start returned, BO still active */
    protocol.BusOff=0; hw.CCCR|=FDCAN_CCCR_INIT;
    assert(CAN_ServiceRecovery()); /* BO alone is insufficient */
    hw.CCCR=0; queued=3; unsigned before=invalidations;
    assert(!CAN_ServiceRecovery());
    assert(!CAN_CommunicationBlocked() && invalidations==before+1 && notice);
    assert(g_motor_can.recovery_discarded_frames==7 && !decoded && fault_notices==1);
    assert(g_motor_can.recovery_completed_ms==now && start_calls==1);

    /* A FIFO frame read before the fault cannot be relabelled with the new
     * A1 epoch, even though its version/payload are otherwise valid. */
    FDCAN_RxHeaderTypeDef h={0x181,FDCAN_STANDARD_ID,FDCAN_DATA_FRAME,FDCAN_DLC_BYTES_8,FDCAN_CLASSIC_CAN};
    uint8_t d[8]={0xa1};
    before=receive_epoch; ++receive_epoch;
    CAN_ProcessFeedback(&h,d,before); assert(decoded==0);
    CAN_ProcessFeedback(&h,d,receive_epoch); assert(decoded==1);

    reset(2000); busoff(); begin_attempt();
    now+=999; assert(CAN_ServiceRecovery() && start_calls==1);
    ++now; assert(CAN_ServiceRecovery() && g_motor_can.recovery_state==1);
    assert(g_motor_can.recovery_failures==1); /* finite hardware wait */
    assert(CAN_ServiceRecovery()); now+=199; assert(CAN_ServiceRecovery() && start_calls==1);
    ++now; assert(CAN_ServiceRecovery() && start_calls==2); /* at most 1 attempt/s */

    reset(UINT32_MAX-100U); busoff(); begin_attempt();
    protocol.BusOff=0; hw.CCCR=0; assert(!CAN_ServiceRecovery()); /* tick wrap */

    reset(100); busoff(); stopped=1; fail_stop=1;
    assert(CAN_ServiceRecovery()); now+=200; assert(CAN_ServiceRecovery());
    assert(g_motor_can.recovery_state==3 && hfdcan1.State==HAL_FDCAN_STATE_ERROR);
    now+=5000; assert(CAN_ServiceRecovery() && stop_calls==1 && start_calls==0);

    reset(100); busoff(); stopped=1; fail_start=1;
    assert(CAN_ServiceRecovery()); now+=200; assert(CAN_ServiceRecovery());
    assert(g_motor_can.recovery_state==3 && start_calls==1 && CAN_CommunicationBlocked());

    reset(100); busoff(); begin_attempt(); protocol.BusOff=0; hw.CCCR=0; fail_notice=1;
    assert(CAN_ServiceRecovery() && g_motor_can.recovery_state==3);

    reset(100); busoff(); queued=8; refill=1; begin_attempt();
    assert(g_motor_can.recovery_discarded_frames==16); /* bounded pre-Stop drain */
    protocol.BusOff=0; hw.CCCR=0;
    before=g_motor_can.recovery_discarded_frames;
    assert(CAN_ServiceRecovery() && g_motor_can.recovery_discarded_frames==before+16);
    assert(CAN_CommunicationBlocked() && !decoded); /* busy peer RX never becomes fresh */
    now+=1000; assert(CAN_ServiceRecovery() && g_motor_can.recovery_state==1);

    reset(100); protocol.BusOff=1; CAN_UpdateStatus();
    assert(CAN_CommunicationBlocked() && invalidations==1); /* missed IRQ fallback */

    reset(100); busoff(); stopped=1; hw.TXBRP=1;
    assert(CAN_ServiceRecovery()); now+=200;
    assert(CAN_ServiceRecovery() && !start_calls && !stop_calls);
    now+=799; assert(CAN_ServiceRecovery() && g_motor_can.recovery_state==1);
    ++now; assert(CAN_ServiceRecovery() && g_motor_can.recovery_state==3);
    assert(g_motor_can.recovery_tx_cancel_timeout==1 && !start_calls);
    reset(100); busoff(); stopped=1; hw.TXBRP=3;
    assert(CAN_ServiceRecovery()); now+=200; assert(CAN_ServiceRecovery());
    hw.TXBRP=0; ++now; assert(CAN_ServiceRecovery() && start_calls==1);
    assert(critical_depth==0);
    puts("PASS: actual CAN task bus-off gate, owner cancellation, quiet wait, HAL Stop/Start states, BO+INIT completion, stale FIFO/epoch discard, bounded peer drain/retry, failure lock and tick wrap");
}
'''

with tempfile.TemporaryDirectory(prefix='mylegwheel_busoff_') as directory:
    work = Path(directory)
    (work / 'test_runtime.h').write_text(runtime)
    for name in ('bsp_fdcan.h', 'main.h', 'FreeRTOS.h', 'task.h'):
        (work / name).write_text('#include "test_runtime.h"\n')
    (work / 'motor_foc.h').write_text('''#include "can_task.h"
void MotorFoc_Init(void);
void MotorFoc_InvalidateFromISR(void);
uint32_t MotorFoc_OnFrame(uint32_t,const uint8_t [8],uint32_t);
uint8_t MotorFoc_GetFeedback(uint8_t,MotorFeedback *);
uint8_t MotorFoc_GetFeedbackWithHealth(uint8_t,MotorFeedback *,MotorFeedbackHealth *);
''')
    (work / 'motor_command.h').write_text('''#include <stdint.h>
void MotorCommand_OnTransportFault(void);
uint32_t MotorCommand_CommunicationStopped(void);
''')
    (work / 'bsp_fdcan_deadline.h').write_text('void FdcanDeadline_AbortAll(void);\n')
    (work / 'test.c').write_text(harness.replace('CAN_SOURCE', (root / 'App/Tasks/Src/can_task.c').as_posix()))
    exe = work / 'test.exe'
    subprocess.run(['gcc', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-DMYLEGWHEEL_A1_CURRENT=1', '-I', str(work),
                    '-I', str(root / 'App/Tasks/Inc'), '-I', str(root / 'App/Services/Inc'),
                    str(work / 'test.c'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
