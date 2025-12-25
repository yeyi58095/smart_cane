#!/usr/bin/env python3

###
import os
import sys

def main():
    python_path = "/home/daniel/smart_cane/rosenv/bin/python"
    target = (
        "/home/daniel/smart_cane/install/"
        "smart_cane_perception/lib/"
        "smart_cane_perception/yolo_sign_detector"
        "smart_cane_perception/yolo_lidar_landmark"
    )

    if not os.path.exists(target):
        print(f"[ERROR] target not found: {target}", file=sys.stderr)
        return 1

    with open(target, "r") as f:
        lines = f.readlines()

    if lines and lines[0].startswith("#!"):
        lines[0] = f"#!{python_path}\n"
    else:
        lines.insert(0, f"#!{python_path}\n")

    with open(target, "w") as f:
        f.writelines(lines)

    os.chmod(target, 0o755)
    print(f"[OK] shebang updated to {python_path}")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
###


#!/usr/bin/env python3
import os
import stat

PYTHON_PATH = "/home/daniel/smart_cane/rosenv/bin/python"

TARGETS = [
    "yolo_sign_detector",
    "yolo_lidar_landmark",
]

BASE_DIR = (
    "/home/daniel/smart_cane/install/"
    "smart_cane_perception/lib/"
    "smart_cane_perception"
)

def fix_one(path):
    if not os.path.exists(path):
        print(f"[SKIP] not found: {path}")
        return

    with open(path, "r") as f:
        lines = f.readlines()

    if lines and lines[0].startswith("#!"):
        lines[0] = f"#!{PYTHON_PATH}\n"
    else:
        lines.insert(0, f"#!{PYTHON_PATH}\n")

    with open(path, "w") as f:
        f.writelines(lines)

    os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)
    print(f"[OK] fixed shebang: {path}")

def main():
    for name in TARGETS:
        fix_one(os.path.join(BASE_DIR, name))

if __name__ == "__main__":
    main()

