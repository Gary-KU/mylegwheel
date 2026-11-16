"""Cross-language wire checks and actual coherent sampler/USB transport tests."""
import binascii
import ctypes as C
import math
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'scripts'))
from robot_telemetry_protocol import Decoder, encode


class Sample(C.Structure):
    _fields_ = [('meta', C.c_uint32*8), ('value', C.c_float*24), ('swing_command_nm', C.c_float*2), ('leg_angle_rate_rad_s', C.c_float*2), ('wheel_torque_command_nm', C.c_float), ('ano_pair_override', C.c_uint32)]


harness = r'''
#include "mock.h"
#include "robot_telemetry_service.h"
#include "robot_telemetry.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#undef assert
#define assert(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
volatile WheelLegDebug g_wheel_leg;
volatile RobotControlDebug g_robot_control;
volatile DualHoldDebug g_dual_hold;
volatile BalanceDebug g_balance;
volatile WheelOutputDebug g_wheel_output;
volatile JointOutputDebug g_joint_output;
volatile uint32_t imu_status;
static uint32_t now;static unsigned queue_size,pending,usb_calls;
static unsigned usb_mode,usb_length;static unsigned char slot[sizeof(RobotTelemetrySample)],usb_owned[136];
static unsigned uart_calls,uart_accept=1;
uint8_t BSP_TelemetryUART_Send(const uint8_t *bytes,uint16_t length,uint32_t baudrate) {
 assert(bytes && length && baudrate==115200);++uart_calls;return (uint8_t)uart_accept;
}
void Error_Handler(void) {assert(0);}
uint32_t HAL_GetTick(void) {return now;}
QueueHandle_t xQueueCreate(unsigned count,unsigned size) {
 assert(count==1 && size==sizeof(RobotTelemetrySample));queue_size=size;return (void*)1;
}
unsigned uxQueueMessagesWaiting(QueueHandle_t q) {assert(q);return pending;}
int xQueueOverwrite(QueueHandle_t q,const void *item) {
 assert(q);memcpy(slot,item,queue_size);pending=1;return pdPASS;
}
int xQueueReceive(QueueHandle_t q,void *item,unsigned wait) {
 assert(q && !wait);if(!pending)return 0;memcpy(item,slot,queue_size);pending=0;return pdPASS;
}
uint8_t CDC_Transmit_HS(uint8_t *bytes,uint16_t length) {
 ++usb_calls;if(usb_mode)return 1;assert(length<=sizeof(usb_owned));
 memcpy(usb_owned,bytes,length);usb_length=length;return 0;
}
static uint32_t u32(unsigned at) {
 return (uint32_t)usb_owned[at]|((uint32_t)usb_owned[at+1]<<8)|
  ((uint32_t)usb_owned[at+2]<<16)|((uint32_t)usb_owned[at+3]<<24);
}
static void check_ano(void) {
 assert(usb_length==22 && usb_owned[0]==0xaa && usb_owned[1]==0xff && usb_owned[2]==0xf1 && usb_owned[3]==16);
 uint8_t sc=0,ac=0;for(unsigned i=0;i<20;i++){sc+=usb_owned[i];ac+=sc;}
 assert(sc==usb_owned[20] && ac==usb_owned[21]);assert(u32(12)==0 && u32(16)==0);
}
int main(void) {
 WheelLegDebug s={0};s.tick_ms=100;s.imu_valid=1;
 s.leg_valid[0]=s.leg_valid[1]=s.wheel_valid[0]=s.wheel_valid[1]=1;
 s.state_valid[0]=s.state_valid[1]=1;s.body_pitch_rad=-.125f;s.body_pitch_rate_rad_s=.3f;
 for(unsigned side=0;side<2;side++) {s.state[side][2]=.125f;s.state[side][3]=.25f;
  s.leg[side].angle=1.5f;s.leg[side].length=.07f;s.angle_speed_rad_s[side]=-.5f;}
 g_robot_control.enable=1;g_robot_control.status=1;g_robot_control.startup_phase=ROBOT_STARTUP_BALANCE;
 g_robot_control.startup_ready_mask=17;
 g_balance.enable=1;g_balance.status=1;g_balance.reference[4]=-.25f;g_balance.position_locked=1;
 g_balance.torque_nm[1]=.03125f;
 g_dual_hold.enable=1;g_dual_hold.status=1;g_dual_hold.angle_reference_rad[0]=1.57f;
 g_dual_hold.pitch_torque_nm[0]=.0625f;g_dual_hold.pitch_torque_nm[1]=-.125f;
 g_joint_output.voltage_mv[0]=-3210;g_joint_output.saturated=1;
 RobotTelemetry_Capture(&s);assert(!pending);RobotTelemetry_Init();now=100;
 assert(g_robot_telemetry.route==2 && g_robot_telemetry.uart_baudrate==115200);
 assert(g_robot_telemetry.ano_pair==22 && g_robot_telemetry.ch5_select==1);
 g_robot_telemetry.ano_pair=16; /* Explicit passive leg fixture; production defaults speed/command. */
 g_robot_telemetry.route=1; /* Explicit USB fixture; production defaults to PA10. */
 RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(u32(4)==8594 && (int32_t)u32(8)==-2864); /* Coherent leg angle and angular velocity. */
 assert(g_robot_telemetry.sent==1 && g_robot_telemetry.data_valid);
 g_robot_telemetry.ano_pair=14;s.tick_ms=120;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert((int32_t)u32(4)==6250 && (int32_t)u32(8)==-12500);
 g_robot_telemetry.ano_pair=0; /* Explicit angle/position fixture for remaining cases. */
 unsigned calls=usb_calls;s.tick_ms=125;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();assert(usb_calls==calls);
 s.tick_ms=140;RobotTelemetry_Capture(&s);s.tick_ms=160;s.body_pitch_rad=.25f;RobotTelemetry_Capture(&s);
 assert(g_robot_telemetry.queue_overwrites==1);RobotTelemetry_Poll();check_ano();assert(u32(4)==1432);
 g_robot_control.last_fault.count=1;g_robot_control.last_fault.reason=ROBOT_FAULT_PARAMETERS;
 s.tick_ms=165;RobotTelemetry_Capture(&s);assert(pending); /* Urgent fault bypasses the 20ms cadence. */
 g_robot_telemetry.format=1;RobotTelemetry_Poll();
 assert(usb_length==136 && usb_owned[0]=='W' && u32(10)==165 && u32(14)==127);
 assert(u32(26)==ROBOT_FAULT_PARAMETERS && u32(30)==1 && (u32(34)&2));
 assert(((u32(34)>>8)&31)==17);
 float voltage;memcpy(&voltage,usb_owned+38+18*4,4);assert(fabsf(voltage+3.21f)<1e-5f);
 s.tick_ms=185;s.imu_valid=s.state_valid[0]=s.state_valid[1]=0;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();
 float pitch;memcpy(&pitch,usb_owned+38,4);assert(isnan(pitch));
 g_robot_telemetry.format=0;s.tick_ms=205;RobotTelemetry_Capture(&s);calls=usb_calls;
 RobotTelemetry_Poll();check_ano();assert(usb_calls==calls+1 && !g_robot_telemetry.data_valid);
 assert(u32(4)==0 && u32(8)==0);
 now=1200;RobotTelemetry_Poll();assert(usb_length==10 && usb_owned[2]==0xf2 && u32(4)==512);
 imu_status=7;now=2200;RobotTelemetry_Poll();assert(u32(4)==7);imu_status=0;
 s.imu_valid=s.state_valid[0]=s.state_valid[1]=1;s.tick_ms=2250;now=2250;
 RobotTelemetry_Capture(&s);usb_mode=1;unsigned drops=g_robot_telemetry.transport_drops;
 RobotTelemetry_Poll();assert(g_robot_telemetry.transport_drops==drops+1);
 s.tick_ms=2270;RobotTelemetry_Capture(&s);usb_mode=0;RobotTelemetry_Poll();check_ano();
 g_robot_telemetry.ano_pair=ROBOT_TELEMETRY_ANO_PAIRS;s.tick_ms=2290;RobotTelemetry_Capture(&s);calls=usb_calls;
 RobotTelemetry_Poll();assert(!g_robot_telemetry.data_valid && usb_calls==calls);
 g_robot_telemetry.ano_pair=0;g_robot_telemetry.enable=0;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();assert(!pending);
 g_robot_telemetry.enable=1;s.tick_ms=UINT32_MAX-10;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();
 calls=usb_calls;s.tick_ms=5;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();assert(usb_calls==calls);
 s.tick_ms=10;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();assert(usb_calls==calls+1);
 /* Default position pair stays at zero while disarmed, then publishes true
  * measured/target positions. Feedback remains valid during the transition. */
 g_robot_telemetry.ano_pair=2;g_robot_control.enable=g_balance.enable=g_dual_hold.enable=0;
 s.tick_ms=30;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(u32(4)==0 && u32(8)==0 && !g_robot_telemetry.data_valid);
 g_robot_control.enable=g_balance.enable=g_dual_hold.enable=1;
 g_balance.reference[2]=.10f;g_balance.parking_target_m=-.25f; /* Soft LQR reference differs from true goal. */
 s.tick_ms=50;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(u32(4)==12500 && (int32_t)u32(8)==-25000 && g_robot_telemetry.data_valid);
 g_robot_telemetry.ano_pair=0;
 assert(uart_calls==0); /* Default USB does not configure or send on PA10. */
 g_robot_telemetry.route=2;s.tick_ms=70;RobotTelemetry_Capture(&s);calls=usb_calls;
 RobotTelemetry_Poll();assert(uart_calls==1 && usb_calls==calls);
 g_robot_telemetry.route=3;usb_mode=1;s.tick_ms=90;RobotTelemetry_Capture(&s);
 drops=g_robot_telemetry.transport_drops;RobotTelemetry_Poll();assert(uart_calls==2 && g_robot_telemetry.transport_drops==drops);
 usb_mode=0;uart_accept=0;s.tick_ms=110;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();
 assert(g_robot_telemetry.transport_drops==drops);
 g_robot_telemetry.route=4;s.tick_ms=130;RobotTelemetry_Capture(&s);calls=usb_calls;
 RobotTelemetry_Poll();assert(usb_calls==calls && uart_calls==3 && g_robot_telemetry.transport_drops==drops+1);
 assert(g_robot_control.enable==1 && g_balance.enable==1 && g_dual_hold.enable==1);
 g_robot_control.enable=g_balance.enable=g_dual_hold.enable=0;
 g_robot_telemetry.route=1;g_robot_telemetry.ano_pair=16;s.leg_valid[1]=0;
 s.tick_ms=150;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(g_robot_telemetry.data_valid && u32(4)==8594 && (int32_t)u32(8)==-2864);
 g_robot_telemetry.ano_pair=17;s.tick_ms=170;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(!g_robot_telemetry.data_valid && u32(4)==0 && u32(8)==0);
 g_robot_telemetry.ano_pair=18;s.tick_ms=190;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(g_robot_telemetry.data_valid && u32(4)==1432 && u32(8)==1718);
 /* CH5 selects feedback only, using the detent captured with the sample.
  * Moving the switch between producer and consumer cannot relabel a frame. */
 g_robot_control.feedback_only=1;g_robot_control.remote_enable=1;g_robot_control.remote_link_ok=1;
 g_robot_control.remote_on_raw=306;g_robot_control.remote_center_raw=1000;
 g_robot_control.remote_off_raw=1694;g_robot_control.remote_tolerance_raw=50;
 g_robot_telemetry.ano_pair=0;g_robot_control.remote_ch5_raw=306;
 s.tick_ms=210;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(g_robot_telemetry.active_pair==16 && u32(4)==8594 && (int32_t)u32(8)==-2864);
 g_robot_control.remote_ch5_raw=1000;
 s.tick_ms=230;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(g_robot_telemetry.active_pair==17 && !g_robot_telemetry.data_valid && u32(4)==0 && u32(8)==0);
 s.leg_valid[1]=1;s.angle_speed_rad_s[1]=.25f;
 s.tick_ms=250;RobotTelemetry_Capture(&s);g_robot_control.remote_ch5_raw=1694;RobotTelemetry_Poll();check_ano();
 assert(g_robot_telemetry.active_pair==17 && g_robot_telemetry.data_valid && u32(4)==8594 && u32(8)==1432);
 s.tick_ms=270;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(g_robot_telemetry.active_pair==18 && u32(4)==1432 && u32(8)==1718);
 g_robot_control.feedback_only=0;
 s.tick_ms=290;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(g_robot_telemetry.active_pair==0); /* Ordinary control cannot repurpose CH5. */
 g_robot_control.feedback_only=1;g_robot_telemetry.ch5_select=0;g_robot_telemetry.ano_pair=16;
 s.tick_ms=310;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();assert(g_robot_telemetry.active_pair==16);
 g_robot_telemetry.ch5_select=1;g_robot_control.remote_link_ok=0;
 s.tick_ms=330;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();assert(g_robot_telemetry.active_pair==16);
 g_robot_control.remote_link_ok=1;g_robot_control.remote_ch5_raw=900;
 s.tick_ms=350;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();assert(g_robot_telemetry.active_pair==16);
 assert(!g_robot_control.enable && !g_balance.enable && !g_dual_hold.enable);
 g_robot_control.feedback_only=0;g_robot_telemetry.ano_pair=19;g_balance.torque_nm[1]=0;
 s.tick_ms=370;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(g_robot_telemetry.active_pair==19 && g_robot_telemetry.data_valid && u32(4)==1432 && u32(8)==0);
 /* New forward-motion diagnostics keep two S32 slots and true goal semantics. */
 s.state_valid[0]=s.state_valid[1]=1;
 s.state[0][2]=s.state[1][2]=.125f;
 s.state[0][3]=.125f;s.state[1][3]=.375f;
 g_wheel_output.voltage_mv[1]=-1500;
 g_robot_telemetry.ano_pair=21;s.tick_ms=350;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert((int32_t)u32(4)==25000 && (int32_t)u32(8)==-150000);
 g_balance.enable=1;g_balance.status=1;g_balance.position_locked=1;
 g_balance.parking_target_m=-.25f;g_balance.reference[2]=.10f;
 g_robot_telemetry.ano_pair=20;s.tick_ms=370;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert((int32_t)u32(4)==37500 && (int32_t)u32(8)==25000);
 g_balance.enable=0;s.tick_ms=390;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(!g_robot_telemetry.data_valid && u32(4)==0 && u32(8)==0);
 /* Left torque, attitude and voltage must retain signs and share a snapshot.
  * A stop clears the command without requiring odometry or IMU validity. */
 g_wheel_output.applied_torque_nm=-.03125f;g_wheel_output.voltage_mv[0]=6000;
 g_robot_telemetry.ano_pair=22;s.tick_ms=410;RobotTelemetry_Capture(&s);
 g_wheel_output.applied_torque_nm=.5f;RobotTelemetry_Poll();check_ano();
 assert((int32_t)u32(4)==-3125 && u32(8)==1432 && g_robot_telemetry.data_valid);
 g_wheel_output.applied_torque_nm=.03125f;g_wheel_output.voltage_mv[0]=-6000;
 g_robot_telemetry.ano_pair=23;s.imu_valid=0;s.tick_ms=430;
 RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(u32(4)==3125 && (int32_t)u32(8)==-600000 && g_robot_telemetry.data_valid);
 g_wheel_output.applied_torque_nm=0;g_wheel_output.voltage_mv[0]=0;s.tick_ms=450;
 RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(u32(4)==0 && u32(8)==0 && g_robot_telemetry.data_valid);
 g_robot_control.startup_phase=ROBOT_STARTUP_OFF;
 g_robot_telemetry.ano_pair=24;s.tick_ms=470;RobotTelemetry_Capture(&s);RobotTelemetry_Poll();check_ano();
 assert(u32(4)==0 && u32(8)==100*ROBOT_FAULT_PARAMETERS && g_robot_telemetry.data_valid);
 puts("PASS: coherent control snapshot, two ANO channels, latest-only queue, fault edge, validity, busy/disconnect, mode/rate bounds and tick wrap");
}
'''

with tempfile.TemporaryDirectory(prefix='robot_telemetry_') as directory:
    tmp = Path(directory)
    # Windows keeps a loaded DLL open; store it outside the temporary test tree.
    dll_path = Path(tempfile.gettempdir()) / 'mylegwheel_telemetry_verify.dll'
    inc = ['-I', str(root/'App/Protocol/Inc')]
    subprocess.run(['gcc','-shared','-std=c99','-Wall','-Wextra','-Werror','-O2',*inc,
                    str(root/'App/Protocol/Src/robot_telemetry.c'),'-o',str(dll_path)], check=True)
    dll=C.CDLL(str(dll_path));bytep=C.POINTER(C.c_uint8)
    dll.RobotTelemetry_Encode.argtypes=[bytep,C.c_uint16,C.POINTER(Sample)]
    dll.RobotTelemetry_EncodeAnoPair.argtypes=[bytep,C.c_uint16,C.POINTER(Sample),C.c_uint32]
    dll.RobotTelemetry_CRC16.argtypes=[bytep,C.c_uint16];dll.RobotTelemetry_CRC16.restype=C.c_uint16
    golden=(C.c_uint8*9)(*b'123456789');assert dll.RobotTelemetry_CRC16(golden,9)==0x29b1
    rng=random.Random(6);decoder=Decoder();packets=[]
    for sequence in range(100):
        sample=Sample();sample.meta[:]=[sequence,sequence*20,127,1,3,0,0,57]
        sample.value[:]=[rng.uniform(-5,5) for _ in range(24)]
        if sequence==10:sample.value[0]=math.nan
        buffer=(C.c_uint8*136)();assert dll.RobotTelemetry_Encode(buffer,136,C.byref(sample))==136
        packet=bytes(buffer);assert packet==encode(list(sample.meta),list(sample.value));packets.append(packet)
    stream=b'noise'+b''.join(packets);records=[]
    while stream:
        size=rng.randint(1,93);records.extend(decoder.feed(stream[:size]));stream=stream[size:]
    assert [r['sequence'] for r in records]==list(range(100)) and records[10]['pitch_rad'] is None
    bad=bytearray(packets[0]);bad[20]^=1
    recovery=Decoder();result=recovery.feed(bytes(bad)+packets[1]);assert len(result)==1 and result[0]['sequence']==1 and recovery.crc_errors==1
    sample=Sample();sample.value[:]=[.125*(i+1) for i in range(24)]
    sample.meta[4]=3;sample.meta[7]=(31<<8)|4
    sample.swing_command_nm[:]=[.0625,-.125]
    sample.leg_angle_rate_rad_s[:]=[.125,-.25]
    sample.wheel_torque_command_nm=-.03125;sample.meta[5]=7
    for pair in range(25):
        buffer=(C.c_uint8*136)();assert dll.RobotTelemetry_EncodeAnoPair(buffer,136,C.byref(sample),pair)==22
        packet=bytes(buffer[:22]);assert packet[:4]==b'\xaa\xff\xf1\x10'
        channels=struct.unpack('<4i',packet[4:20]);assert channels[2:]==(0,0)
        if pair==2:assert channels[:2]==(50000,62500)
        if pair==12:assert channels[:2]==(300,3100)
        if pair==13:assert channels[:2]==(-12500,100)
        if pair==14:assert channels[:2]==(6250,-12500)
        if pair==15:assert channels[:2]==(300,175000)
        if pair==16:assert channels[:2]==(5013,716)
        if pair==17:assert channels[:2]==(5729,-1432)
        if pair==18:assert channels[:2]==(716,2148)
        if pair==19:assert channels[:2]==(716,175000)
        if pair==20:assert channels[:2]==(-12500,75000)
        if pair==21:assert channels[:2]==(75000,225000)
        if pair==22:assert channels[:2]==(-3125,716)
        if pair==23:assert channels[:2]==(-3125,212500)
        if pair==24:assert channels[:2]==(300,700)
        sc=ac=0
        for byte in packet[:20]:sc=(sc+byte)&255;ac=(ac+sc)&255
        assert packet[20:]==bytes([sc,ac])
    buffer=(C.c_uint8*136)();assert dll.RobotTelemetry_EncodeAnoPair(buffer,21,C.byref(sample),0)==0
    assert dll.RobotTelemetry_EncodeAnoPair(buffer,136,C.byref(sample),25)==0
    sample.wheel_torque_command_nm=math.nan
    assert dll.RobotTelemetry_EncodeAnoPair(buffer,136,C.byref(sample),23)==0
    sample.wheel_torque_command_nm=-.03125
    sample.value[0]=math.nan;assert dll.RobotTelemetry_EncodeAnoPair(buffer,136,C.byref(sample),0)==0
    sample.value[0]=math.inf;assert dll.RobotTelemetry_EncodeAnoPair(buffer,136,C.byref(sample),0)==0
    sample.value[0]=1e30;assert dll.RobotTelemetry_EncodeAnoPair(buffer,136,C.byref(sample),0)==0
    print('PASS: C/Python framing, CRC golden vector, fragmented/noisy/corrupt streams, signed ANO pairs and invalid/overflow rejection')
    mock='''#ifndef MOCK_H
#define MOCK_H
#include <stdint.h>
#include <stddef.h>
typedef void *QueueHandle_t;
#define pdPASS 1
#define __weak __attribute__((weak))
QueueHandle_t xQueueCreate(unsigned,unsigned);
unsigned uxQueueMessagesWaiting(QueueHandle_t);
int xQueueOverwrite(QueueHandle_t,const void*);
int xQueueReceive(QueueHandle_t,void*,unsigned);
uint32_t HAL_GetTick(void);
void Error_Handler(void);
uint8_t CDC_Transmit_HS(uint8_t*,uint16_t);
#define USBD_OK 0
#endif
'''
    (tmp/'mock.h').write_text(mock)
    for name in ('main.h','FreeRTOS.h','queue.h','usbd_cdc_if.h'):(tmp/name).write_text('#include "mock.h"\n')
    (tmp/'test.c').write_text(harness)
    includes=[tmp,root/'App/Tasks/Inc',root/'App/Services/Inc',root/'App/Protocol/Inc',root/'Control/WheelLeg/Inc',root/'BSP/Peripherals/Inc']
    flags=[item for path in includes for item in ('-I',str(path))]
    exe=tmp/'test.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-O2',*flags,
                    str(tmp/'test.c'),str(root/'App/Services/Src/robot_telemetry_service.c'),
                    str(root/'App/Protocol/Src/robot_telemetry.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
