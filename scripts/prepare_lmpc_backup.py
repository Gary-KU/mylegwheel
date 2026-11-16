from pathlib import Path
import shutil
ROOT=Path(__file__).resolve().parents[1]
stage=ROOT/'build/lmpc_backup_20261007'
assert not stage.exists()
shutil.copytree(ROOT.parent/'H723_LMPC',stage,ignore=shutil.ignore_patterns('build','backups','__pycache__','sim_cache'))
(stage/'build').mkdir()
shutil.copy2(ROOT/'diagnostics/current_board_lmpc_read_20261007.json',stage/'tests/data/parking_timeout_10707.json')
print(stage)
