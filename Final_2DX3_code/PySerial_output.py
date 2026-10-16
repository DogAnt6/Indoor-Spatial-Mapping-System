import serial
import math
import open3d as o3d

PORT = "COM9"
BAUD_RATE = 115200
OUTPUT_FILE = "tof_radar.xyz"

# 10 cm = 100 mm shift in x after each full scan
X_OFFSET_PER_SCAN_MM = 100.0
POINTS_PER_SCAN = 32

def polar_to_xyz(angle_deg: float, distance_mm: float, scan_index: int):
    theta = math.radians(angle_deg)

    # Polar scan in Y-Z plane
    y = distance_mm * math.sin(theta)
    z = distance_mm * math.cos(theta)

    # Each full scan shifts x by 100 mm
    x = scan_index * X_OFFSET_PER_SCAN_MM

    return x, y, z


def main():
    points = []
    scan_index = 0
    collecting = False

    with serial.Serial(PORT, BAUD_RATE, timeout=2) as ser:
        print(f"Listening on {PORT} at {BAUD_RATE} baud...")

        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            print("RX:", line)

            # Ignore CSV header
            if line == "Angle,Distance":
                continue

            # Start of a scan
            if line == "SCAN_START":
                collecting = True
                print(f"Started scan #{scan_index + 1}")
                continue

            # End of one full scan
            if line == "SCAN_END":
                collecting = False
                scan_index += 1
                print(f"Finished scan #{scan_index}")
                continue

            # Manual stop signal from MCU
            if line == "STOP_SCAN":
                print("Stop signal received.")
                break

            # Ignore non-data lines unless we're collecting
            if not collecting:
                continue

            try:
                angle_str, dist_str = line.split(",")
                angle = float(angle_str)
                distance = float(dist_str)

                x, y, z = polar_to_xyz(angle, distance, scan_index)
                points.append((x, y, z))

            except ValueError:
                print(f"Skipping invalid line: {line}")

    with open(OUTPUT_FILE, "w") as f:
        for x, y, z in points:
            f.write(f"{x:.2f} {y:.2f} {z:.2f}\n")

    print(f"\nSaved {len(points)} points to {OUTPUT_FILE}")

    if len(points) == 0:
        print("No points to display.")
        return

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    lines = []
    num_scans = len(points) // POINTS_PER_SCAN

    # Connect points within each scan
    for s in range(num_scans):
        start = s * POINTS_PER_SCAN

        for i in range(POINTS_PER_SCAN - 1):
            lines.append([start + i, start + i + 1])

        # Close each scan loop
        lines.append([start + POINTS_PER_SCAN - 1, start])

    # Connect matching points between adjacent scans
    for s in range(num_scans - 1):
        current_start = s * POINTS_PER_SCAN
        next_start = (s + 1) * POINTS_PER_SCAN

        for i in range(POINTS_PER_SCAN):
            lines.append([current_start + i, next_start + i])

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    point_cloud.paint_uniform_color([0, 1, 0])  # green points
    line_set.paint_uniform_color([1, 0, 0])     # red lines

    o3d.visualization.draw_geometries(
        [point_cloud, line_set],
        window_name="ToF 3D Scan"
    )

    # Save 3D model
    o3d.io.write_point_cloud("scan.ply", point_cloud)
    o3d.io.write_line_set("scan_lines.ply", line_set)


if __name__ == "__main__":
    main()