from pathlib import Path
import shutil, subprocess, xml.etree.ElementTree as ET
from f103_wheel_voltage_patch import configure_wheel_voltage

root = Path(__file__).resolve().parents[1]
src = root.parents[1] / 'stm32-foc/software'
dst = root / 'build/f103_left_wheel/source'
for folder in ('Src', 'Inc', 'USER', 'Drivers'):
    shutil.copytree(src / folder, dst / folder, dirs_exist_ok=True)
out = dst.parent
out.mkdir(parents=True, exist_ok=True)
limit_v, supply_v = configure_wheel_voltage(dst, root / 'App/Tasks/Inc/wheel_leg_config.h')
main = dst / 'Src/main.c'
s = main.read_text(encoding='utf-8-sig')
start = s.index('  // ===== CAN 控制闭环')
end = s.index('  /* USER CODE END WHILE */', start)
s = s[:start] + '''  /* Direct CAN voltage control: no forced open-loop startup. */
  while (1)
  {
    float command = targetVotage;
    if ((HAL_GetTick() - lastRecvTime) > 100U || command != command || command == 0.0f)
    {
      setTargetVotage(0.0f);
      setPhaseVoltage(0.0f, 0.0f, 0.0f);
      shaft_angle = shaftAngle();
      continue;
    }
    setTargetVotage(command);
    loopFOC();
  }
''' + s[end:]
s = s.replace('RCC_OSCILLATORTYPE_HSI;', 'RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;')
s = s.replace('RCC_OscInitStruct.HSIState = RCC_HSI_ON;', 'RCC_OscInitStruct.HSIState = RCC_HSI_ON;\n  RCC_OscInitStruct.HSEState = RCC_HSE_ON;')
s = s.replace('RCC_PLLSOURCE_HSI_DIV2', 'RCC_PLLSOURCE_HSE').replace('RCC_PLL_MUL16', 'RCC_PLL_MUL8')
s = s.replace('motorID = Flash_ReadMotorID();', 'motorID = 3; /* Left wheel CAN ID, explicitly selected for this build. */')
s = s.replace('/* USER CODE BEGIN Error_Handler_Debug */', '/* USER CODE BEGIN Error_Handler_Debug */\n  __disable_irq();\n  while (1) { /* Clock failure: do not initialize motor PWM. */ }')
assert 'OPENLOOP_VOLT' not in s
main.write_text(s, encoding='utf-8')
flash = dst / 'USER/FlashStorage.c'
s = flash.read_text(encoding='utf-8-sig').replace('uint32_t buf[5];', 'uint32_t buf[4];').replace('i<sizeof(buf)', 'i<sizeof(buf)/sizeof(buf[0])')
flash.write_text(s, encoding='utf-8')
tree = ET.parse(src / 'MDK-ARM/C6T6SimpleFoc.uvprojx')
tc = Path('D:/app/Keil_v5/ARM/ARMCC/bin')
includes = [dst / p for p in ('Inc','Drivers/STM32F1xx_HAL_Driver/Inc','Drivers/STM32F1xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Device/ST/STM32F1xx/Include','Drivers/CMSIS/Include','USER')]
objects = []
with (out/'build.log').open('w', encoding='utf-8') as log:
    def run(args):
        r = subprocess.run([str(x) for x in args], capture_output=True, text=True, errors='replace')
        log.write(subprocess.list2cmdline([str(x) for x in args])+'\n'+r.stdout+r.stderr); log.flush()
        if r.returncode: raise RuntimeError(r.stdout+r.stderr)
    for f in tree.findall('.//Groups/Group/Files/File'):
        p = (src/'MDK-ARM'/f.findtext('FilePath').replace('\\','/')).resolve()
        obj = out/(p.stem+'.o'); objects.append(obj)
        if p.suffix.lower() == '.s':
            run([tc/'armasm.exe','--cpu','Cortex-M3','--apcs=interwork','--pd','__MICROLIB SETA 1','-o',obj,p])
        else:
            p = dst/p.relative_to(src)
            run([tc/'armcc.exe','--cpu','Cortex-M3','--c99','-O2','-g','--split_sections','-D__MICROLIB','-DUSE_HAL_DRIVER','-DSTM32F103x6',*[v for inc in includes for v in ('-I',inc)],'-c',p,'-o',obj])
    run([tc/'armlink.exe',*objects,'--cpu','Cortex-M3','--library_type=microlib','--ro-base','0x08000000','--rw-base','0x20000000','--entry','Reset_Handler','--first','startup_stm32f103x6.o(RESET)','--map','--list',out/'left_wheel.map','-o',out/'left_wheel.axf'])
    run([tc/'fromelf.exe','--i32combined','--output',out/'left_wheel.hex',out/'left_wheel.axf'])
    run([tc/'fromelf.exe','--bin','--output',out/'left_wheel.bin',out/'left_wheel.axf'])
assert (out/'left_wheel.bin').stat().st_size <= 0x7800, 'Firmware overlaps calibration storage'
print(f'Built 8 MHz HSE / 64 MHz CPU, ID3, 7 pole pairs, +/-{limit_v:g} V CAN clamp, {supply_v:g} V supply model; boot alignment retained.')
print(out/'left_wheel.hex')
