"""Create distinct Keil projects so Download cannot select the read-only image."""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
directory = root/'MDK-ARM'
original = directory/'MyLegwheel.uvprojx'
options = directory/'MyLegwheel.uvoptx'
for side, index in (('right', 1), ('left', 0)):
    tree = ET.parse(original)
    target = tree.find('./Targets/Target')
    target_name = f'H723 single-leg PD {side}'
    target.find('TargetName').text = target_name
    common = target.find('./TargetOption/TargetCommonOption')
    common.find('OutputDirectory').text = f'..\\build\\leg-pd-{side}\\'
    common.find('ListingPath').text = f'..\\build\\leg-pd-{side}\\listings\\'
    common.find('OutputName').text = f'H723_leg_pd_{side}'
    define = target.find('./TargetOption/TargetArmAds/Cads/VariousControls/Define')
    values = [v for v in (define.text or '').split(',') if v and not v.startswith('ROBOT_LEG_PD_TEST_SIDE=')]
    values.append(f'ROBOT_LEG_PD_TEST_SIDE={index}')
    define.text = ','.join(values)
    destination = directory/f'MyLegwheel_leg_pd_{side}.uvprojx'
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    if options.exists():
        option_tree = ET.parse(options)
        option_target = option_tree.find('./Target')
        option_target.find('TargetName').text = target_name
        option_target.find('./TargetOption/OPTLEX/ListingPath').text = common.findtext('ListingPath')
        option_tree.write(destination.with_suffix('.uvoptx'), encoding='utf-8', xml_declaration=True)
    check = ET.parse(destination).find('./Targets/Target')
    assert check.findtext('./TargetOption/TargetCommonOption/OutputName') == f'H723_leg_pd_{side}'
    assert f'ROBOT_LEG_PD_TEST_SIDE={index}' in check.findtext('./TargetOption/TargetArmAds/Cads/VariousControls/Define')
    print(f'PASS: dedicated Keil project {destination}')
