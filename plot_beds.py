import matplotlib.pyplot as plt

data_file = "bed_points.txt"

COLOR_MAP = {
    "bed_red":     "#FF0000",
    "bed_blue":    "#0066FF",
    "bed_green":   "#00CC66",
    "bed_yellow":  "#FFCC00",
    "bed_pink":    "#FF66CC",
    "bed_purple":  "#9933FF",
    "bed_orange":  "#FF8800",
    "bed_cyan":    "#00CCCC",
    "bed_gray":    "#888888",
}

beds = {}

with open(data_file, "r") as f:
    for line in f:
        if ":" not in line:
            continue

        color, pts_str = line.split(":")
        color = color.strip()

        if pts_str.strip() == "":
            continue

        pts = []
        parts = pts_str.split("),")
        for p in parts:
            p = p.replace("(", "").replace(")", "").strip()
            if not p:
                continue
            x_str, y_str = p.split(",")
            pts.append((float(x_str), float(y_str)))

        beds[color] = pts

plt.figure(figsize=(8, 8))

for color, pts in beds.items():
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]

    # 使用自定義顏色，如果沒有就 fallback 到 black
    plot_color = COLOR_MAP.get(color, "#000000")

    plt.scatter(xs, ys, label=color, color=plot_color)

plt.title("Detected Bed Positions")
plt.xlabel("X (map)")
plt.ylabel("Y (map)")
plt.grid(True)
plt.legend()
plt.axis("equal")
plt.show()
