"""Offline safety tests: fake target/Jim interpreter and real MCU service C.

The Jim tests invoke OpenOCD without interface/target/init and replace every
memory command with an in-process dictionary. No probe is opened.
"""
from pathlib import Path
import ast
import copy
import importlib.util
import json
import re
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('probe',ROOT/'diagnostics/leg1531_pulse_probe.py')
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


def fixture():
    layout={'objects':{},'scalars':{},'signatures':{0x08000000:[1,2,3,4,5,6,7,8]}}
    scalar_names=list(probe.CONTROL)
    for n in probe.NODES:scalar_names += [f'id{n}_{k}' for k in ('debug_running','pending','aligning','maintenance')]
    scalar_names += ['parameter0','debug_ma','debug_duration','maintenance_value','maintenance_aux']
    values={name:0 for name in scalar_names}
    values.update(remote_enable=1,remote_link=1,remote_status=2,ch5_raw=1000,center_raw=1000,
                  off_raw=1694,on_raw=306,tolerance_raw=50,tick=1000)
    for i,name in enumerate(scalar_names):layout['scalars'][name]=(0x20000000+4*i,'i' if name=='debug_ma' else 'I')
    objects={}
    definitions=[(f'node{n}',probe.NODE_FIELDS) for n in probe.NODES]+[(f'cal{n}',probe.CAL_FIELDS) for n in probe.NODES]
    definitions += [('dq',probe.DQ_FIELDS),('stored',probe.STORED_FIELDS)]
    for i,(name,fields) in enumerate(definitions):
        layout['objects'][name]=dict(base=0x20001000+i*0x400,bytes=len(fields)*4,
            fields={key:(4*j,fmt) for j,(key,fmt) in enumerate(fields.items())})
        objects[name]={key:0 for key in fields}
    for n in probe.NODES:
        x=objects[f'node{n}'];c=objects[f'cal{n}']
        x.update(status_ms=1000,diagnostic_ms=1000,status_count=1,diagnostic_count=1,
            fast_ms=1000,fast_count=1,flags=0x3e,fast_flags=0x3e,diagnostic_flags=64,
            session_ready=1,limit_verified=1,voltage_verified=1,slew_verified=1,fast_valid=1,
            remote_iq_limit_a=3.0,remote_voltage_limit_v=5.0 if n in probe.JOINTS else 7.2,
            remote_iq_slew_a_per_s=10.0)
        c.update(calibration_confirmed=1,dynamic_response_confirmed=1,angle_anchor_confirmed=1,
            joint_timing_confirmed=1,direction=1,kt_nm_per_a=.23,current_limit_a=3.0,minimum_iq_slew_a_per_s=10.0)
    before=dict(values=values,objects=objects,fresh={n:dict(fast_ms=1000,fast_count=1,fast_tick_ms=1000,
        status_ms=1000,diagnostic_ms=1000,status_count=1,diagnostic_count=1,health_tick_ms=1000) for n in probe.NODES},
        identity={'uid_words':probe.EXPECTED_UID},dhcsr=0,host_read_s=.01)
    return layout,before


def fake_jim(layout,before,node,action,ma=100,mutate=None):
    memory={base+4*i:(value&0xffffffff) for base,words in layout['signatures'].items()
            for i,value in enumerate(words)}
    for name,(address,fmt) in layout['scalars'].items():memory[address]=before['values'][name]&0xffffffff
    for name,definition in layout['objects'].items():
        for field,(offset,fmt) in definition['fields'].items():
            memory[definition['base']+offset]=struct.unpack('<I',struct.pack('<'+fmt,before['objects'][name][field]))[0]
    for i,word in enumerate(probe.EXPECTED_UID):memory[0x1ff1e800+4*i]=int(word,16)
    memory[0xe000edf0]=0
    if mutate:mutate(memory)
    request=layout['scalars']['request_mask'][0]
    maint=layout['scalars'][f'id{node}_maintenance'][0]
    stored=layout['objects']['stored'];stored_at=lambda key:stored['base']+stored['fields'][key][0]
    prefix='array set MOCK {'+' '.join(f'{a} {v}' for a,v in memory.items())+'}\n'
    prefix+='''
catch {rename read_memory native_read_memory}
proc read_memory {a bits count} {
 global MOCK
 if {$bits!=32} {error "unsupported fake read"}
 set result {}
 for {set i 0} {$i<$count} {incr i} {
  set at [expr {$a+4*$i}]
  if {![info exists MOCK($at)]} {error "unmapped fake memory $at"}
  lappend result [format "0x%08x" $MOCK($at)]
 }
 return $result
}
catch {rename mww native_mww}
'''
    prefix+=f'''proc mww {{a value}} {{
 global MOCK
 echo "MOCK_WRITE $a $value"
 set a [expr {{$a+0}}]
 set MOCK($a) $value
 if {{$a=={request}}} {{set MOCK($a) 0}}
 if {{$a=={maint}}} {{
  set MOCK($a) 0
  set MOCK({stored_at('rx_count')}) 1
  set MOCK({stored_at('reply_opcode')}) 21
  set MOCK({stored_at('reply_result')}) 0
  set MOCK({stored_at('ack_result')}) 0
  set MOCK({stored_at('completed')}) 1
 }}
}}
rename sleep native_sleep
proc sleep {{duration}} {{}}
'''
    plan=probe.tcl_plan(layout,before,node,ma,action)
    # init and poll off are removed; there are no adapter/interface/target
    # commands. Native shutdown terminates before OpenOCD's automatic init.
    script=prefix+'\n'.join(plan.splitlines()[2:])+'\n'
    with tempfile.TemporaryDirectory(prefix='leg1531_fake_jim_') as directory:
        path=Path(directory)/'mock.cfg';path.write_text(script,encoding='ascii')
        r=subprocess.run([probe.OCD,'-f',str(path)],capture_output=True,text=True,timeout=5)
    return r.stdout+r.stderr


class ProbeSafetyTests(unittest.TestCase):
    def test_bounds_and_explicit_physical_ack(self):
        for node,ma,execute,physical,read_cal in ((3,100,False,False,False),(5,0,False,False,False),
            (5,201,False,False,False),(5,-201,False,False,False),(5,100,True,False,False),
            (5,100,True,True,True)):
            with self.subTest(node=node,ma=ma),self.assertRaises(RuntimeError):
                probe.validate_request(node,ma,execute,physical,read_cal)
        for n in probe.JOINTS:
            for ma in (-200,200):probe.validate_request(n,ma,True,True,False)

    def test_strict_stop_parameters_and_health_guards(self):
        layout,s=fixture();probe.validate_snapshot(s,5,100)
        for name in ('robot','balance','dual','wheel_test','wheel_zero','storage','feedback_only','ch5',
                     'request_mask','stop_request','can_recovery','can_bus_off','can_error_passive','can_error_warning'):
            bad=copy.deepcopy(s);bad['values'][name]=1
            with self.subTest(guard=name),self.assertRaises(RuntimeError):probe.validate_snapshot(bad,5,100)
        for name,value in (('remote_link',0),('remote_status',1),('ch5_raw',306)):
            bad=copy.deepcopy(s);bad['values'][name]=value
            with self.subTest(guard=name),self.assertRaises(RuntimeError):probe.validate_snapshot(bad,5,100)
        for n in probe.NODES:
            for field,value in (('mode',3),('faults',1),('limit_verified',0),('fast_valid',0),
                                ('remote_iq_limit_a',2.0),('remote_iq_slew_a_per_s',1.0)):
                bad=copy.deepcopy(s);bad['objects'][f'node{n}'][field]=value
                with self.subTest(node=n,field=field),self.assertRaises(RuntimeError):probe.validate_snapshot(bad,5,100)
            bad=copy.deepcopy(s);bad['fresh'][n]['fast_ms']=989
            with self.assertRaises(RuntimeError):probe.validate_snapshot(bad,5,100)
            # A slow bulk SWD read does not age the separate live page checks.
            old_bulk=copy.deepcopy(s)
            old_bulk['objects'][f'node{n}'].update(status_ms=1,diagnostic_ms=1,fast_ms=1)
            probe.validate_snapshot(old_bulk,5,100)
            bad=copy.deepcopy(s);bad['fresh'][n]['diagnostic_ms']=959
            with self.assertRaises(RuntimeError):probe.validate_snapshot(bad,5,100)

    def test_generated_pulse_has_only_three_single_joint_writes(self):
        layout,s=fixture()
        log=fake_jim(layout,s,5,'pulse',-200)
        self.assertIn('PULSE_COMPLETED_STOPPED',log)
        writes=re.findall(r'MOCK_WRITE (\S+) (\S+)',log)
        self.assertEqual(len(writes),3,log)
        self.assertEqual([int(a,0) for a,b in writes],
            [layout['scalars'][key][0] for key in ('debug_ma','debug_duration','request_mask')])
        self.assertEqual([int(b,0) for a,b in writes],[(-200)&0xffffffff,100,16])
        rows=probe.parse_capture(log,layout,5)
        self.assertEqual(len(rows),20)
        self.assertTrue(all(row['node5']['mode']==0 for row in rows))
        self.assertTrue(all(row['host_read_duration_ms']>=0 for row in rows))

    def test_generated_cal_read_cannot_energize(self):
        layout,s=fixture();log=fake_jim(layout,s,5,'read_cal')
        self.assertIn('CAL_READ_COMPLETED',log)
        writes=re.findall(r'MOCK_WRITE (\S+) (\S+)',log)
        self.assertEqual([int(a,0) for a,b in writes],
            [layout['scalars'][key][0] for key in ('maintenance_value','maintenance_aux','id5_maintenance')])
        self.assertEqual([int(b,0) for a,b in writes],[0,0,0x15])

    def test_latency_plan_contains_no_memory_write(self):
        layout,s=fixture();plan=probe.tcl_plan(layout,s,5,100,'latency')
        self.assertNotIn('mww',plan)
        log=fake_jim(layout,s,5,'latency')
        self.assertIn('LATENCY_COMPLETED_NO_WRITES',log)
        self.assertNotIn('MOCK_WRITE',log)
        self.assertEqual(len(probe.parse_capture(log,layout,5)),20)

    def test_last_moment_owner_or_image_change_makes_no_request(self):
        layout,s=fixture()
        for address in (layout['scalars']['robot'][0],0x08000000,0x1ff1e800):
            log=fake_jim(layout,s,5,'pulse',mutate=lambda memory:memory.__setitem__(address,memory[address]^1))
            self.assertIn('PROBE_ERROR',log)
            self.assertNotIn('MOCK_WRITE',log)

    def test_default_main_never_writes_and_failure_never_retries_pulse(self):
        layout,s=fixture()
        with tempfile.TemporaryDirectory(prefix='leg1531_host_test_') as directory:
            for execute in (False,True):
                path=Path(directory)/('execute.json' if execute else 'prepare.json')
                identity={'uid_words':probe.EXPECTED_UID}
                with patch.object(probe,'local_image_info',return_value={'hex_sha256':probe.EXPECTED_HEX}), \
                     patch.object(probe,'build_layout',return_value=copy.deepcopy(layout)), \
                     patch.object(probe,'signature_words',return_value=layout['signatures']), \
                     patch.object(probe,'acquire_probe_lock',return_value=lambda:None), \
                     patch.object(probe,'require_no_openocd'), \
                     patch.object(probe,'read_blocks',return_value={}), \
                     patch.object(probe,'target_identity',return_value=identity), \
                     patch.object(probe,'check_firmware') as checked, \
                     patch.object(probe,'snapshot',return_value=s), \
                     patch.object(probe,'run_tcl',side_effect=[TimeoutError('lost transport'),'STOP_REQUEST_ONLY']) as run:
                    argv=['--id','5','--output',str(path)]
                    if execute:
                        argv+=['--execute','--physical-setup-confirmed']
                        with self.assertRaises(TimeoutError):probe.main(argv)
                        self.assertEqual(run.call_count,2)
                        emergency=run.call_args_list[1].args[0]
                        self.assertIn(f'mww {layout["scalars"]["stop_request"][0]:#x} 1',emergency)
                        self.assertNotIn('REQUEST_ONCE',emergency)
                    else:
                        probe.main(argv);run.assert_not_called()
                    checked.assert_called_once()
                report=json.loads(path.read_text())
                self.assertEqual(report['request_attempted'],execute)


class McuDeadlineTests(unittest.TestCase):
    def test_real_service_single_joint_100ms_expiry_and_explicit_stop(self):
        path=ROOT/'tests/verify_foc_current.py'
        source=path.read_text(encoding='utf-8');ns={'__file__':str(path)}
        exec(compile(source.split("harness=r'''")[0],str(path),'exec'),ns)
        tree=ast.parse(source)
        harness=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Name) and t.id=='harness' for t in n.targets))
        prefix=re.split(r'int main\(void\)\s*\{',harness,maxsplit=1)[0]
        fixture=r'''
int main(void) {
 hfdcan1.Instance=&hw;
 const unsigned ids[4]={1,2,5,6};
 for(unsigned index=0;index<4;++index)for(unsigned sign=0;sign<2;++sign) {
  unsigned slot=ids[index]-1;fixed_limit_fixture_reset();idle_ready();
  if(index==3)now=UINT32_MAX-55U; /* Deadline crosses unsigned wrap. */
  g_motor_foc.debug_current_ma[slot]=sign?200:-200;
  g_motor_foc.debug_duration_ms[slot]=100;g_motor_foc.debug_request_mask=1U<<slot;
  assert(cycle(NULL,0));uint32_t begin=now;
  assert(mode[slot]==3 && iq_ma[slot]==(sign?200:-200));
  for(unsigned i=0;i<8;++i)if(i!=slot)assert(mode[i]==0);
  for(unsigned k=0;k<19;++k){assert(cycle(NULL,0));assert(mode[slot]==3);}
  assert((uint32_t)(now-begin)==95U);assert(cycle(NULL,0));
  assert((uint32_t)(now-begin)==100U && mode[slot]==0 && !g_motor_foc.debug_request_mask);
  unsigned ended=iq_tx;for(unsigned k=0;k<60;++k)assert(cycle(NULL,0));assert(iq_tx==ended);
 }
 fixed_limit_fixture_reset();idle_ready();
 g_motor_foc.debug_current_ma[4]=100;g_motor_foc.debug_duration_ms[4]=100;g_motor_foc.debug_request_mask=16;
 assert(cycle(NULL,0) && mode[4]==3);g_motor_foc.stop_request=1;
 assert(cycle(NULL,0));assert(mode[4]==0 && !g_motor_foc.debug_request_mask);
 unsigned stopped=iq_tx;for(unsigned k=0;k<60;++k)assert(cycle(NULL,0));assert(iq_tx==stopped);
 puts("PASS: real MCU100ms expiry all four joints/signs, unsigned wrap, STOP and no replay");return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix='leg1531_mcu_') as directory:
            work=Path(directory);(work/'runtime.h').write_text(ns['runtime'])
            for name in ('main.h','bsp_fdcan.h','FreeRTOS.h','task.h'):(work/name).write_text('#include "runtime.h"\n')
            (work/'test.c').write_text(prefix+fixture)
            command=['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function',
                '-DMYLEGWHEEL_A1_CURRENT=1','-DROBOT_LEGS_ONLY_CALIBRATION=0']
            for inc in (work,ROOT/'App/Tasks/Inc',ROOT/'App/Services/Inc',ROOT/'BSP/Peripherals/Inc',
                        ROOT/'Control/WheelLeg/Inc',ROOT/'App/Protocol/Inc'):command+=['-I',str(inc)]
            command += [str(work/'test.c')]+[str(ROOT/p) for p in ('App/Services/Src/motor_foc.c',
                'App/Services/Src/motor_command.c','App/Services/Src/joint_calibration.c',
                'Control/WheelLeg/Src/leg_pos.c','BSP/Peripherals/Src/bsp_fdcan_deadline.c')]
            exe=work/'test.exe';subprocess.run(command+['-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__=='__main__':unittest.main()
