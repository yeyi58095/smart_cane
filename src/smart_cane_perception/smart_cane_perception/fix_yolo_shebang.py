#!/usr/bin/env python3
import os
import sys

def main():
    python_path = "/home/daniel/smart_cane/rosenv/bin/python"
    target = (
        "/home/daniel/smart_cane/install/"
        "smart_cane_perception/lib/"
        "smart_cane_perception/yolo_sign_detector"
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
