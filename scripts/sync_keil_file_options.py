"""Match Keil file registration while preserving existing debug/options settings."""
from datetime import datetime
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

root=Path(__file__).resolve().parents[1]
project=ET.parse(root/'MDK-ARM/MyLegwheel.uvprojx')
groups=project.findall('./Targets/Target/Groups/Group')
target=root/'MDK-ARM/MyLegwheel.uvoptx'
text=target.read_text(encoding='utf-8')
blocks=re.findall(r'(?ms)^  <Group>.*?^  </Group>',text)
by_name={ET.fromstring(block).findtext('GroupName'):block for block in blocks}
assert [g.findtext('GroupName') for g in groups]==list(by_name)
number=0
for group_number,group in enumerate(groups,1):
    block=by_name[group.findtext('GroupName')]
    old_files=re.findall(r'(?ms)^    <File>.*?^    </File>',block)
    file_map={ET.fromstring(f).findtext('PathWithFileName'):f for f in old_files}
    updated=[]
    for definition in group.findall('./Files/File'):
        number+=1
        path=definition.findtext('FilePath')
        element=file_map.get(path)
        if element is None:
            element='''    <File>
      <GroupNumber>0</GroupNumber>
      <FileNumber>0</FileNumber>
      <FileType>1</FileType>
      <tvExp>0</tvExp>
      <tvExpOptDlg>0</tvExpOptDlg>
      <bDave2>0</bDave2>
      <PathWithFileName>PATH</PathWithFileName>
      <FilenameWithoutPath>NAME</FilenameWithoutPath>
      <RteFlg>0</RteFlg>
      <bShared>0</bShared>
    </File>'''.replace('PATH',path).replace('NAME',definition.findtext('FileName'))
        for tag,value in [('GroupNumber',group_number),('FileNumber',number),('FileType',definition.findtext('FileType'))]:
            element=re.sub(r'(<'+tag+r'>)[^<]*(</'+tag+r'>)',lambda match:match[1]+str(value)+match[2],element)
        updated.append(element)
    if old_files:
        prefix=block[:block.index(old_files[0])]
        suffix=block[block.index(old_files[-1])+len(old_files[-1]):]
    else:
        prefix=block[:block.rfind('  </Group>')]
        suffix='\n  </Group>'
    replacement=prefix+'\n'.join(updated)+suffix
    ET.fromstring(replacement)
    text=text.replace(block,replacement,1)
ET.fromstring(text)
backup=root/'backups'/('before_keil_options_sync_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
backup.mkdir();shutil.copy2(target,backup/target.name)
target.write_text(text,encoding='utf-8',newline='\r\n')
print(f'Synchronized {number} file options; original: {backup}')
