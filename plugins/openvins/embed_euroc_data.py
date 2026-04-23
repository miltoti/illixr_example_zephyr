#!/usr/bin/env python3
"""
embed_euroc_data.py

Reads EuRoC MAV dataset and generates C++ headers:
  - embedded_imu.hpp   : IMU samples as a struct array
  - embedded_cam.hpp   : PNG file bytes as uint8_t arrays + index table

Usage:
  python3 embed_euroc_data.py <euroc_mav0_dir> <output_dir> [num_cam_frames]

Example:
  python3 embed_euroc_data.py data/V1_02_medium/mav0 generated 50
"""

import csv
import os
import sys

def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <mav0_dir> <output_dir> [num_frames]")
        sys.exit(1)

    mav0_dir = sys.argv[1]
    out_dir = sys.argv[2]
    num_frames = int(sys.argv[3]) if len(sys.argv) > 3 else 50

    os.makedirs(out_dir, exist_ok=True)

    # ── Read camera CSV (cam0) to get timestamps + filenames ──────────
    cam0_csv = os.path.join(mav0_dir, "cam0", "data.csv")
    cam1_csv = os.path.join(mav0_dir, "cam1", "data.csv")

    cam0_entries = []
    with open(cam0_csv, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0].startswith("#"):
                continue
            ts_ns = int(row[0])
            filename = row[1].strip()
            cam0_entries.append((ts_ns, filename))

    cam1_entries = []
    with open(cam1_csv, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0].startswith("#"):
                continue
            ts_ns = int(row[0])
            filename = row[1].strip()
            cam1_entries.append((ts_ns, filename))

    # Use first num_frames
    cam0_entries = cam0_entries[:num_frames]
    cam1_entries = cam1_entries[:num_frames]

    # ── Read IMU CSV ──────────────────────────────────────────────────
    imu_csv = os.path.join(mav0_dir, "imu0", "data.csv")
    imu_entries = []
    with open(imu_csv, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0].startswith("#"):
                continue
            ts_ns = int(row[0])
            wx, wy, wz = float(row[1]), float(row[2]), float(row[3])
            ax, ay, az = float(row[4]), float(row[5]), float(row[6])
            imu_entries.append((ts_ns, wx, wy, wz, ax, ay, az))

    # Only keep IMU samples up to the last camera timestamp
    last_cam_ts = max(cam0_entries[-1][0], cam1_entries[-1][0])
    # Also include some IMU samples before the first camera frame
    first_cam_ts = min(cam0_entries[0][0], cam1_entries[0][0])
    imu_entries = [e for e in imu_entries if e[0] <= last_cam_ts + 50000000]

    print(f"Embedding {len(cam0_entries)} cam0 frames, {len(cam1_entries)} cam1 frames, {len(imu_entries)} IMU samples")

    # ── Generate embedded_imu.hpp ─────────────────────────────────────
    with open(os.path.join(out_dir, "embedded_imu.hpp"), "w") as f:
        f.write("#pragma once\n")
        f.write("#include <cstdint>\n")
        f.write("#include <cstddef>\n\n")
        f.write(f"static constexpr size_t kEmbeddedImuCount = {len(imu_entries)};\n\n")
        f.write("struct EmbeddedImuSample {\n")
        f.write("    int64_t ts_ns;\n")
        f.write("    double wx, wy, wz;  // rad/s\n")
        f.write("    double ax, ay, az;  // m/s^2\n")
        f.write("};\n\n")
        f.write("static const EmbeddedImuSample kEmbeddedImu[] = {\n")
        for ts, wx, wy, wz, ax, ay, az in imu_entries:
            f.write(f"    {{{ts}LL, {wx:.17e}, {wy:.17e}, {wz:.17e}, {ax:.17e}, {ay:.17e}, {az:.17e}}},\n")
        f.write("};\n")

    print(f"  Wrote embedded_imu.hpp ({len(imu_entries)} samples)")

    # ── Generate embedded_cam.hpp ─────────────────────────────────────
    # Each PNG becomes a uint8_t array. We generate an index table.
    with open(os.path.join(out_dir, "embedded_cam.hpp"), "w") as f:
        f.write("#pragma once\n")
        f.write("#include <cstdint>\n")
        f.write("#include <cstddef>\n\n")
        f.write(f"static constexpr size_t kEmbeddedCamCount = {len(cam0_entries)};\n\n")

        # Embed each PNG as a byte array
        total_bytes = 0
        for i, ((ts0, fn0), (ts1, fn1)) in enumerate(zip(cam0_entries, cam1_entries)):
            # cam0
            png0_path = os.path.join(mav0_dir, "cam0", "data", fn0)
            with open(png0_path, "rb") as pf:
                data0 = pf.read()
            f.write(f"// cam0 frame {i}: {fn0} ({len(data0)} bytes)\n")
            f.write(f"static const uint8_t kCam0Frame{i}[] = {{\n")
            write_byte_array(f, data0)
            f.write(f"}};\n\n")

            # cam1
            png1_path = os.path.join(mav0_dir, "cam1", "data", fn1)
            with open(png1_path, "rb") as pf:
                data1 = pf.read()
            f.write(f"// cam1 frame {i}: {fn1} ({len(data1)} bytes)\n")
            f.write(f"static const uint8_t kCam1Frame{i}[] = {{\n")
            write_byte_array(f, data1)
            f.write(f"}};\n\n")

            total_bytes += len(data0) + len(data1)

        # Index table
        f.write("struct EmbeddedCamFrame {\n")
        f.write("    int64_t ts_ns;\n")
        f.write("    const uint8_t* cam0_png;\n")
        f.write("    size_t cam0_size;\n")
        f.write("    const uint8_t* cam1_png;\n")
        f.write("    size_t cam1_size;\n")
        f.write("};\n\n")

        f.write("static const EmbeddedCamFrame kEmbeddedCam[] = {\n")
        for i, ((ts0, fn0), (ts1, fn1)) in enumerate(zip(cam0_entries, cam1_entries)):
            png0_path = os.path.join(mav0_dir, "cam0", "data", fn0)
            png1_path = os.path.join(mav0_dir, "cam1", "data", fn1)
            sz0 = os.path.getsize(png0_path)
            sz1 = os.path.getsize(png1_path)
            f.write(f"    {{{ts0}LL, kCam0Frame{i}, {sz0}, kCam1Frame{i}, {sz1}}},\n")
        f.write("};\n")

    print(f"  Wrote embedded_cam.hpp ({len(cam0_entries)} stereo pairs, {total_bytes / 1024 / 1024:.1f} MB)")


def write_byte_array(f, data, cols=16):
    """Write bytes as hex literals, 16 per line."""
    for i, b in enumerate(data):
        if i % cols == 0:
            f.write("    ")
        f.write(f"0x{b:02x},")
        if i % cols == cols - 1:
            f.write("\n")
    if len(data) % cols != 0:
        f.write("\n")


if __name__ == "__main__":
    main()