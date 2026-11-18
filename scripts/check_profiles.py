"""Fail when a profile changes something switching cannot apply.

Switching a profile rebuilds the mission worker and reloads the scene; the
simulator process keeps running with its own display, ports, camera mounts and
appearance models. A profile that disagrees with the default one about any of
those is therefore either inert or quietly wrong: the planner would compute
depth with one camera while the engine renders another.

Run it on a deployment before a demo:

    python scripts/check_profiles.py
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from planner.config import (  # noqa: E402
    available_profiles,
    profile_catalog,
    profile_drift,
)


def main() -> int:
    profiles = profile_catalog()
    if not profiles:
        print(f"no profile under {ROOT / 'config' / 'core'}")
        return 1
    for info in profiles:
        print(f"{info.id}: {info.label}" + (f" — {info.note}" if info.note else ""))
    drift = profile_drift()
    if drift:
        print(
            "\nthese sections differ between profiles, but switching a profile "
            "does not restart the simulator:",
        )
        for line in drift:
            print(f"  {line}")
        print("put the difference in a switchable section, or keep the files equal.")
        return 1
    print(
        f"\n{len(available_profiles())} profiles differ only where a switch can apply."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
