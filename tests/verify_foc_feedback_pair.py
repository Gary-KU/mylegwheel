"""Actual byte-level FOC RX -> calibrated observer pairing and fail-closed checks.

No hardware. Tests real motor_foc.c, joint_calibration.c and observer, including
FAST frames received between observer cycles. Existing current/STOP transport
regressions separately check active-current revocation and command boundaries.
"""
import ast
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
foc_source=(ROOT/"tests/verify_foc_current.py").read_text(encoding="utf-8")
ns={"__file__":str(ROOT/"tests/verify_foc_current.py")}
exec(compile(foc_source.split("harness=r'''")[0],str(ROOT/"tests/verify_foc_current.py"),"exec"),ns)
runtime=ns["runtime"]
harness=next(ast.literal_eval(n.value) for n in ast.parse(foc_source).body
             if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="harness" for t in n.targets))
prefix=harness.split("int main(void) {")[0]
fixture=r'''
#include "wheel_leg_observer.h"
#include "imu_task.h"
#include "leg_trajectory.h"
volatile WheelLegDebug g_wheel_leg;
volatile float g_wheel_leg_reference[2][6];
static WheelLegDebug observation;
static uint32_t base;
static unsigned pair_front,pair_rear;
static float pose[2];
uint8_t IMU_GetSample(ImuSample *out) {memset(out,0,sizeof(*out));return 1;}
uint8_t MotorCAN_GetFeedback(uint8_t id,MotorFeedback *out) {
 assert(critical);return MotorFoc_GetFeedback(id,out);
}
#ifdef MOTOR_FEEDBACK_HEALTH_VERSION
uint8_t MotorCAN_GetFeedbackWithHealth(uint8_t id,MotorFeedback *out,MotorFeedbackHealth *health) {
 assert(critical);return MotorFoc_GetFeedbackWithHealth(id,out,health);
}
#endif
static void fast_motor(unsigned id,float delta,int speed) {
 float angle=1.0f;
 for(unsigned j=0;j<JOINT_CALIBRATION_COUNT;++j) if(g_joint_calibration[j].motor_id==id) {
  const JointCalibration *c=&g_joint_calibration[j];
  float joint=pose[j%2?0:1];
  angle=c->encoder_reference_rad+c->ratio*(joint-c->model_reference_rad)/c->direction+delta;
 }
 angle=fmodf(angle,6.28318530718f);if(angle<0)angle+=6.28318530718f;
 uint8_t d[8]={0xa1,0x3e,0,0,0,0,0,0};
 put16(d+2,(int)lroundf(angle*1000));put16(d+4,speed);
 assert(MotorFoc_OnFrame(0x700+id,d,now));
}
static void health_all(void) {
 unsigned old=emit_fast;emit_fast=0;
 for(unsigned i=0;i<8;++i) if(MOTOR_FOC_NODE_MASK&(1U<<i)) {mode[i]=0;flags[i]=0x3e;fault[i]=0;diagnostic_flags[i]=0;feed(i,0);}
 emit_fast=old;
}
static void other_fast(void) {
 for(unsigned id=1;id<=8;++id) if((MOTOR_FOC_NODE_MASK&(1U<<(id-1))) && id!=pair_front && id!=pair_rear)
  fast_motor(id,0,0);
}
static void observe(void) {WheelLeg_Observe(&observation);g_wheel_leg=observation;assert(!critical);}
static void setup(unsigned front,unsigned rear,unsigned wrap) {
 pair_front=front;pair_rear=rear;MotorFoc_Init();
 if(wrap)now=UINT32_MAX-12U;else now+=5;
 assert(leg_inverse(.03f,.07f,1.57f,1.57f,pose));
 health_all();for(unsigned id=1;id<=8;++id)if(MOTOR_FOC_NODE_MASK&(1U<<(id-1)))fast_motor(id,0,0);
 observe(); /* A prior test may deliberately move the clock/domain. */
 now+=5;health_all();for(unsigned id=1;id<=8;++id)if(MOTOR_FOC_NODE_MASK&(1U<<(id-1)))fast_motor(id,0,0);
 observe();assert(observation.leg_valid[0] && observation.leg_valid[1]);base=now;
}
static unsigned side(void) {return pair_front==g_joint_calibration[0].motor_id?0:1;}
static void timing(unsigned newer_front,unsigned intermediate,unsigned wrap) {
 unsigned newer=newer_front?pair_front:pair_rear,older=newer_front?pair_rear:pair_front;
 now=base+4;fast_motor(older,0,1111);
 now=base+5;other_fast();observe();assert(observation.leg_valid[side()]);
 MotorFeedback saved={0};
 if(intermediate) {now=base+6;fast_motor(newer,.003f,2222);assert(MotorFoc_GetFeedback(newer,&saved));}
 now=base+11;fast_motor(newer,.006f,9000);other_fast();observe();
 MotorFeedback latest;assert(MotorFoc_GetFeedback(newer,&latest));
 assert(latest.last_rx_ms==now && latest.age_ms==0 && fabsf(latest.speed_rad_s-9)<1e-6f);
 if(!intermediate) {
  assert(!observation.leg_valid[side()] && observation.feedback_detail==WHEEL_LEG_FEEDBACK_SKEW);
  assert(observation.feedback_ms==7);return;
 }
 assert(observation.leg_valid[side()] && observation.feedback_detail==0);
 unsigned j=(side()*2)+(newer_front?0:1),other=(side()*2)+(newer_front?1:0);
 assert(observation.joint_feedback_age_ms[j]==5 && observation.joint_feedback_age_ms[other]==7);
 assert(fabsf(observation.joint_angle_rad[j]-saved.joint_angle_rad)<1e-6f);
 assert(fabsf(observation.joint_shaft_speed_rad_s[j]-2.222f)<1e-6f);
 assert(g_wheel_leg_pair_debug[side()].last_selected_rx_count[newer_front?0:1]==saved.rx_count);
 assert(g_wheel_leg_pair_debug[side()].history_pair_count);
 if(wrap)assert(now<base); /* Unsigned stamp arithmetic crosses UINT32 wrap. */
}
static void bad_fast(unsigned kind) {
 setup(5,6,0);
 now=base+4;fast_motor(6,0,0);
 now=base+5;other_fast();observe();
 now=base+6;fast_motor(5,.003f,2222);
 MotorFeedback before;assert(MotorFoc_GetFeedback(5,&before));
 now=base+10;
 uint8_t d[8]={0xa1,0x3e,0,0,0,0,0,0};put16(d+2,1219);put16(d+4,0);
 if(kind==0)d[0]=0;
 if(kind==1)put16(d+2,6284);
 if(kind==2)d[1]&=~8U;
 if(kind==3)put16(d+4,INT16_MAX);
 assert(MotorFoc_OnFrame(0x705,d,now));
 MotorFeedback bad;assert(!MotorFoc_GetFeedback(5,&bad));assert(!bad.fast_history_count);
 now=base+11;fast_motor(5,.006f,9000);other_fast();observe();
 MotorFeedback after;assert(MotorFoc_GetFeedback(5,&after));
 assert(after.fast_history_count==1 && after.fast_history_generation!=before.fast_history_generation);
 assert(!observation.leg_valid[1] && observation.feedback_detail==WHEEL_LEG_FEEDBACK_SKEW);
}
static void stale_and_epoch(void) {
 setup(5,6,0);now=base+5;fast_motor(5,0,0);other_fast();observe();assert(observation.leg_valid[1]);
 now=base+11;fast_motor(5,0,0);other_fast();observe();assert(!observation.leg_valid[1]);
 MotorFeedback f;assert(!MotorFoc_GetFeedback(6,&f) && f.age_ms==11);
 setup(5,6,0);now=base+4;fast_motor(6,0,0);now=base+5;other_fast();observe();
 now=base+6;fast_motor(5,.003f,2222);
 MotorFoc_InvalidateFromISR();
 now=base+11;health_all();fast_motor(5,.006f,9000);other_fast();observe();
 assert(!observation.leg_valid[1] && !MotorFoc_GetFeedback(6,&f));
 assert(MotorFoc_GetFeedback(5,&f) && f.fast_history_count==1);
 now=base+16;fast_motor(6,0,0);fast_motor(5,0,0);other_fast();observe();assert(observation.leg_valid[1]);
}
static void reanchor(void) {
 setup(5,6,0);now=base+4;fast_motor(6,0,0);now=base+5;other_fast();observe();
 now=base+6;fast_motor(5,.003f,2222);MotorFeedback before;assert(MotorFoc_GetFeedback(5,&before));
 g_motor_foc.calibration[4].angle_anchor_rad+=.001f;
 MotorFeedback f;assert(!MotorFoc_GetFeedback(5,&f) && !f.fast_history_count);
 now=base+11;fast_motor(5,.006f,9000);other_fast();observe();
 assert(MotorFoc_GetFeedback(5,&f) && f.fast_history_count==1);
 assert(f.fast_history_generation!=before.fast_history_generation && !observation.leg_valid[1]);
 /* A real >10ms RX gap also discards all samples of the previous unwrap. */
 uint32_t generation=f.fast_history_generation;now+=11;health_all();fast_motor(5,0,0);
 assert(MotorFoc_GetFeedback(5,&f) && f.fast_history_count==1 && f.fast_history_generation!=generation);
}
static void transient_health(void) {
 for(unsigned kind=0;kind<4;++kind) {
  setup(5,6,0);now=base+4;fast_motor(6,0,0);now=base+5;other_fast();observe();
  now=base+6;fast_motor(5,.003f,2222);now=base+10;
  unsigned old=emit_fast;emit_fast=0;
  if(kind==0)flags[4]&=~8U;
  if(kind==1)fault[4]=1;
  if(kind==2)diagnostic_flags[4]=32;
  if(kind==3)diagnostic_flags[4]=2;
  feed(4,0);flags[4]=0x3e;fault[4]=0;diagnostic_flags[4]=0;feed(4,0);emit_fast=old;
  now=base+11;fast_motor(5,.006f,9000);other_fast();observe();
  MotorFeedback f;assert(MotorFoc_GetFeedback(5,&f) && f.fast_history_count==1);
  assert(!observation.leg_valid[1] && observation.feedback_detail==WHEEL_LEG_FEEDBACK_SKEW);
 }
}
int main(void) {
 for(unsigned s=0;s<2;++s)for(unsigned reverse=0;reverse<2;++reverse) {
  setup(s?5:1,s?6:2,0);timing(reverse,1,0);
 }
 puts("PASS: actual FOC RX frames between5/6ms observer ticks recover F6/R4 ages5/7 with unchanged6ms skew and10ms freshness; both legs/orientations, calibrated angle/speed/count, latest frame untouched");
 setup(5,6,0);timing(1,0,0);
 puts("PASS: no coherent real history remains a skew rejection; no fabricated samples or extended timestamps");
 setup(5,6,1);timing(1,1,1);
 stale_and_epoch();reanchor();transient_health();
 for(unsigned k=0;k<4;++k)bad_fast(k);
 puts("PASS: UINT32 wrap, true stale, epoch, >10ms reseed, changed anchor, transient sensor/diagnostic faults, malformed/sensor-invalid/angle-invalid/clipped FAST clear old history and fail closed");
 assert(MOTOR_FOC_CURRENT_HARD_LIMIT_A==3 && MOTOR_FOC_JOINT_CURRENT_HARD_LIMIT_A==3);
 puts("LIMIT: receive-dequeue timestamps are preserved; this fixes a missed valid received combination, not a claim to reconstruct the hardware sampling clock or this trial's exact RX jitter");
 return 0;
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="foc_feedback_pair_") as directory:
        work=Path(directory)
        (work/"runtime.h").write_text(runtime,encoding="utf-8")
        for name in ("main.h","FreeRTOS.h","task.h","bsp_fdcan.h"):
            (work/name).write_text('#include "runtime.h"\n',encoding="utf-8")
        (work/"test.c").write_text(prefix+fixture,encoding="utf-8")
        command=["gcc","-std=c99","-O2","-Wall","-Wextra","-Werror","-Wno-unused-function",
                 "-DMYLEGWHEEL_A1_CURRENT=1","-DROBOT_LEGS_ONLY_CALIBRATION=0","-I",str(work)]
        for folder in ("App/Tasks/Inc","App/Services/Inc","App/Protocol/Inc","Control/WheelLeg/Inc","BSP/Peripherals/Inc","tests/support/wheel_leg/Inc"):
            command += ["-I",str(ROOT/folder)]
        sources=["App/Tasks/Src/wheel_leg_observer.c","App/Services/Src/motor_foc.c","App/Services/Src/joint_calibration.c",
                 "App/Services/Src/motor_command.c","Control/WheelLeg/Src/lqr_k.c","Control/WheelLeg/Src/leg_pos.c",
                 "tests/support/wheel_leg/Src/leg_trajectory.c","BSP/Peripherals/Src/bsp_fdcan_deadline.c"]
        exe=work/"test.exe"
        subprocess.run(command+[str(work/"test.c"),*[str(ROOT/s) for s in sources],"-lm","-o",str(exe)],check=True)
        subprocess.run([str(exe)],check=True)


if __name__=="__main__":main()
