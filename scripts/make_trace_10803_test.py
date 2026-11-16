from pathlib import Path
root=Path(__file__).resolve().parents[1]/'build/lmpc_trace_10803'
s=(root/'tests/verify_actuation_alignment.py').read_text(encoding='utf-8')
begin=s.index("checks=r'''");end=s.index("'''",begin+11)
checks=r'''
static void speed(float value) {
 motors[WHEEL_LEFT_CAN_ID-1].speed_rad_s=-value/WHEEL_RADIUS_M;
 motors[WHEEL_RIGHT_CAN_ID-1].speed_rad_s=value/WHEEL_RADIUS_M;
}
int main(void) {
 for(unsigned i=0;i<8;++i)motors[i]=(MotorFeedback){.online=1,.joint_calibrated=1};
 axial_feedback(.07f,0);remote_ok=remote_fresh=1;
 remote_sample.frame.channel[4]=1000;step();remote_sample.frame.channel[4]=306;
 unsigned captured=0,learning=0,blocked=0,phase_mask=0;
 for(unsigned i=0;i<12000;++i) {
  speed(i<1000?.3f:i<6000?.04f:0);
  unsigned old=g_balance_trace.sequence;step();assert(g_robot_control.enable);
  if(g_balance_trace.sequence!=old) {
   ++captured;unsigned index=(g_balance_trace.next+WHEEL_BALANCE_TRACE_ROWS-1)%WHEEL_BALANCE_TRACE_ROWS;
   WheelBalanceTraceRow r=g_balance_trace.rows[index];
   assert(r.tick_ms==now && r.parking_lean_rad==g_balance.parking_lean_rad);
   assert(r.parking_filtered_speed_m_s==g_balance.parking_lean_filtered_speed_m_s);
   assert(((r.flags>>16)&1)==g_balance.parking_lean_learning);
   assert((r.flags>>24)==g_parking_learning_debug.gate_mask);
   learning+=(r.flags>>16)&1;blocked+=(r.flags>>24)!=0;phase_mask|=1U<<((r.flags>>8)&255);
  }
 }
 assert(captured==600 && learning>0 && blocked>0 && (phase_mask&(1U<<3)));
 assert(g_balance_trace.count==512);
 unsigned first=g_balance_trace.next;
 for(unsigned i=1;i<512;++i) {
  unsigned prev=(first+i-1)%512,index=(first+i)%512;
  assert(g_balance_trace.rows[index].tick_ms-g_balance_trace.rows[prev].tick_ms==100);
 }
 WheelBalanceTraceDebug saved=g_balance_trace;
 remote_sample.frame.channel[4]=1000;for(unsigned i=0;i<40;++i)step();
 assert(!g_robot_control.enable && !memcmp((const void*)&g_balance_trace,&saved,sizeof saved));
 remote_sample.frame.channel[4]=306;step();
 assert(g_balance_trace.count==1 && g_balance_trace.next==1 && g_balance_trace.rows[0].tick_ms==now);
 puts("PASS: 60 s task run; 100 ms samples, 51.2 s ring wrap, exact lean/gate fields, disarm freezes and rearm resets.");
}
'''
s=s[:begin]+"checks=r'''"+checks+s[end:]
(root/'tests/verify_long_trace.py').write_text(s,encoding='utf-8')
