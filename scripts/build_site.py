from __future__ import annotations

import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SITE_SRC = ROOT / "site"
OUT = ROOT / "_site"


def copy_tree_contents(source: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for item in source.iterdir():
        target = destination / item.name
        if item.is_dir():
            shutil.copytree(item, target, dirs_exist_ok=True)
        else:
            shutil.copy2(item, target)


def build() -> None:
    if OUT.exists():
        shutil.rmtree(OUT)

    copy_tree_contents(SITE_SRC, OUT)
    (OUT / ".nojekyll").write_text("", encoding="utf-8")


def main() -> int:
    build()
    print(f"Built site at {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
