"""Package the bounded H723 bench images with a read-only rollback image."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

root = Path(__file__).resolve().parents[1]
output = root/'build/leg_pd_isolation_20261006'
output.mkdir(parents=True, exist_ok=True)
items = {}
for side in ('right', 'left'):
    for extension in ('hex', 'axf'):
        items[f'H723_leg_pd_{side}.{extension}'] = root/f'build/leg-pd-{side}/H723_leg_pd_{side}.{extension}'
for extension in ('hex', 'axf'):
    items[f'H723_READ_ONLY.{extension}'] = root/f'build/read-only/H723_READ_ONLY.{extension}'
items['README.txt'] = root/'docs/leg_pd_isolation.txt'
manifest = {'mode': 'single-leg angle PD bench test; NOT full balance',
            'duration_ms': 3000, 'joint_command_limit_mv': 200,
            'virtual_leg_pd_limit_nm': .005, 'files': {}}
for name, source in items.items():
    destination = output/name
    shutil.copy2(source, destination)
    data = destination.read_bytes()
    entry = {'size': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    if destination.suffix == '.hex':
        high = 0
        ranges = []
        eof = False
        for line in data.decode('ascii').splitlines():
            assert line.startswith(':')
            record = bytes.fromhex(line[1:])
            assert sum(record) % 256 == 0 and len(record) == record[0]+5
            count, offset, kind = record[0], int.from_bytes(record[1:3], 'big'), record[3]
            if kind == 4:
                high = int.from_bytes(record[4:6], 'big') << 16
            elif kind == 0:
                ranges.append((high+offset, high+offset+count))
            elif kind == 1:
                eof = True
        assert eof and ranges
        start, end = min(x[0] for x in ranges), max(x[1] for x in ranges)
        assert start == 0x08000000 and end < 0x08200000
        entry['address_start'] = f'0x{start:08X}'
        entry['address_end_exclusive'] = f'0x{end:08X}'
    manifest['files'][name] = entry
(output/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
archive = output.with_suffix('.zip')
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as handle:
    for name in (*items, 'manifest.json'):
        handle.write(output/name, name)
with zipfile.ZipFile(archive) as handle:
    assert handle.testzip() is None
print(f'PASS: firmware HEX checksums, H723 address range, file hashes and ZIP: {archive}')
