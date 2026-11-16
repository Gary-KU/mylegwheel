"""Stage LMPC changes inside this workspace; never alter the live project."""
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT.parent / 'H723_LMPC'
STAGE = ROOT / 'build/lmpc_parking_20261008'

if __name__ == '__main__':
    if STAGE.exists():
        raise SystemExit(f'Candidate already exists: {STAGE}')
    shutil.copytree(SOURCE, STAGE, ignore=shutil.ignore_patterns(
        'build', 'backups', '__pycache__', 'sim_cache', '*.uvguix.*'))
    (STAGE / 'build').mkdir()
    print(STAGE)
