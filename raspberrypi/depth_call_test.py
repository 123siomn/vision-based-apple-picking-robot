import os
import re
import subprocess


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEPTH_EXE = os.path.join(BASE_DIR, "depth_helper", "depth_xyz_reader")


def get_value(line, name):
    """
    从 OK 行里提取 x/y/z 等字段。
    例如：
    OK cx=320 cy=240 x=0.0 y=0.0 z=2169.0
    """
    pattern = rf"(?:^|\s){name}=([-0-9.]+)"
    match = re.search(pattern, line)
    if match is None:
        return None
    return float(match.group(1))


def get_depth_xyz(cx, cy, radius=5, frames=30):
    cmd = [
        DEPTH_EXE,
        str(cx),
        str(cy),
        str(radius),
        str(frames)
    ]

    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True
    )

    # SDK 的 warning 可能会混在 stdout 或 stderr 里，所以合并后统一解析
    output = (result.stdout + "\n" + result.stderr).strip()

    print("depth helper raw output:")
    print(output)

    # 不再要求输出必须以 OK 开头，而是从多行输出里找到 OK 那一行
    ok_line = None
    for line in output.splitlines():
        if line.startswith("OK "):
            ok_line = line
            break

    if ok_line is None:
        return None

    x = get_value(ok_line, "x")
    y = get_value(ok_line, "y")
    z = get_value(ok_line, "z")

    if x is None or y is None or z is None:
        return None

    return x, y, z


if __name__ == "__main__":
    # 先模拟草莓框中心点
    cx = 320
    cy = 240

    xyz = get_depth_xyz(cx, cy, radius=5, frames=30)

    if xyz is None:
        print("depth read failed")
    else:
        x, y, z = xyz
        print(f"camera xyz: X={x:.1f} mm, Y={y:.1f} mm, Z={z:.1f} mm")
