"""Apply config/default.yaml SIM camera settings to local AirSim settings.json."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim.airsim_settings import (  # noqa: E402
    LOCAL_AIRSIM_SETTINGS_PATH,
    build_airsim_settings_with_overrides,
    write_local_airsim_settings,
)


def main():
    parser = argparse.ArgumentParser(description="Write local AirSim settings.json from config/default.yaml")
    parser.add_argument("--dry-run", action="store_true", help="只打印将写入的 settings，不修改文件")
    parser.add_argument("--no-backup", action="store_true", help="覆盖前不备份原 settings.json")
    args = parser.parse_args()

    settings = build_airsim_settings_with_overrides()
    if args.dry_run:
        print(f"[AirSimSettings] target: {LOCAL_AIRSIM_SETTINGS_PATH}")
        print(json.dumps(settings, ensure_ascii=False, indent=2))
        return

    path = write_local_airsim_settings(make_backup=not args.no_backup)
    print(f"[AirSimSettings] wrote: {path}")
    print("[AirSimSettings] restart AirSim/UE for camera resolution changes to take effect")


if __name__ == "__main__":
    main()
