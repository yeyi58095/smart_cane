#!/usr/bin/env python3
import os
import stat

# 你的 rosenv python
PYTHON_PATH = "/home/daniel/smart_cane/rosenv/bin/python"

# 你想集中修的 targets： (package_name, script_name)
TARGETS = [
    ("smart_cane_perception", "yolo_lidar_landmark"),
    ("smart_cane_perception", "yolo_sign_detector"),
    ("smart_cane_nav", "align_to_target"),
    # 之後你還有別支需要 rosenv，就一直往下加
    # ("smart_cane_xxx", "some_script"),
]

INSTALL_ROOT = "/home/daniel/smart_cane/install"

def fix_one(path: str):
    if not os.path.exists(path):
        print(f"[SKIP] not found: {path}")
        return False

    with open(path, "r") as f:
        lines = f.readlines()

    # replace/insert shebang
    if lines and lines[0].startswith("#!"):
        lines[0] = f"#!{PYTHON_PATH}\n"
    else:
        lines.insert(0, f"#!{PYTHON_PATH}\n")

    with open(path, "w") as f:
        f.writelines(lines)

    # ensure executable bit
    os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)
    print(f"[OK] fixed: {path}")
    return True

def main():
    ok = 0
    total = 0
    for pkg, script in TARGETS:
        total += 1
        path = os.path.join(INSTALL_ROOT, pkg, "lib", pkg, script)
        if fix_one(path):
            ok += 1
    print(f"[DONE] fixed {ok}/{total} scripts")

if __name__ == "__main__":
    main()
