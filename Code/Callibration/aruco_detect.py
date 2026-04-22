"""
aruco_detect.py  —  ArUco + OpenCV AprilTag detection with pose estimation

Usage
-----
python aruco_detect.py                          # scan all dictionaries
python aruco_detect.py --dicts 4x4_50 6x6_250  # specific families
python aruco_detect.py --dicts cv_apriltag_36h11 --marker-size 150
python aruco_detect.py --list-dicts             # show all names and exit

Controls
--------
  q  Quit
  v  Verify distance (prompts for real measurement, saves to CSV)
  d  Toggle legend between all dicts / detected only
"""

import argparse
import csv
import os
import subprocess
import sys
import time

import cv2
import numpy as np

# ── Registry ─────────────────────────────────────────────────────────────────
REGISTRY = {				
    "4x4_1000":          cv2.aruco.DICT_4X4_1000,
    "5x5_1000":          cv2.aruco.DICT_5X5_1000,
    "6x6_1000":          cv2.aruco.DICT_6X6_1000,
    "7x7_1000":          cv2.aruco.DICT_7X7_1000,
    "aruco_original":    cv2.aruco.DICT_ARUCO_ORIGINAL,
    "cv_apriltag_16h5":  cv2.aruco.DICT_APRILTAG_16h5,
    "cv_apriltag_25h9":  cv2.aruco.DICT_APRILTAG_25h9,
    "cv_apriltag_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "cv_apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

_PALETTE = [
    (0, 255, 0), (0, 180, 255), (255, 80, 0), (0, 0, 255),
    (255, 0, 180), (0, 255, 220), (180, 0, 255), (255, 200, 0),
]


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dicts", "-d", nargs="+", metavar="DICT", default=None,
                   help="Dictionaries to enable (default: all). See --list-dicts.")
    p.add_argument("--list-dicts", "-l", action="store_true",
                   help="Print available dictionary names and exit.")
    p.add_argument("--marker-size", "-s", type=float, default=100.0, metavar="MM",
                   help="Physical marker side length in mm (default: 100).")
    p.add_argument("--calib", "-c", default="calibration.npz", metavar="FILE")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--csv", default="aruco_results.csv", metavar="FILE")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    return p.parse_args()


# ── Calibration ───────────────────────────────────────────────────────────────
def load_calibration(path):
    if not os.path.exists(path):
        print(f"Error: '{path}' not found. Run 2_calibrate.py first.")
        sys.exit(1)
    with np.load(path) as d:
        mtx, dist = d["K"], d["dist"]
        size = tuple(d["image_size"]) if "image_size" in d else None
    return mtx, dist, size


# ── Detectors ─────────────────────────────────────────────────────────────────
def build_detectors(names, use_new_api):
    out = []
    for i, name in enumerate(names):
        ad = cv2.aruco.getPredefinedDictionary(REGISTRY[name])
        pa = cv2.aruco.DetectorParameters()
        out.append({
            "name":    name,
            "ad":      ad,
            "pa":      pa,
            "det":     cv2.aruco.ArucoDetector(ad, pa) if use_new_api else None,
            "colour":  _PALETTE[i % len(_PALETTE)],
        })
    return out


def detect_all(gray, detectors, use_new_api):
    results = []
    for d in detectors:
        if use_new_api:
            corners, ids, _ = d["det"].detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, d["ad"], parameters=d["pa"])
        if ids is not None and len(ids):
            results.append({"corners": corners, "ids": ids,
                             "name": d["name"], "colour": d["colour"]})
    return results


# ── Pose ──────────────────────────────────────────────────────────────────────
def obj_points(size_mm):
    h = size_mm / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def estimate_pose(corners, mtx, dist, obj_pts):
    rvecs, tvecs = [], []
    for c in corners:
        _, rvec, tvec = cv2.solvePnP(obj_pts, c[0], mtx, dist,
                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(rvec); tvecs.append(tvec)
    return rvecs, tvecs


# ── CSV ───────────────────────────────────────────────────────────────────────
def save_csv(path, dict_name, marker_id, est_x, est_y, est_z, real_mm):
    est_mm = float(est_z)   # use Z only
    err    = est_mm - real_mm
    pct    = abs(err) / real_mm * 100 if real_mm else float("inf")
    exists = os.path.isfile(path)

    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=[
            "dict", "id",
            "est_x", "est_y", "est_z",
            "real_mm", "err_mm", "err_pct",
        ])
        if not exists:
            w.writeheader()
        w.writerow({
            "dict": dict_name,
            "id": marker_id,
            "est_x": round(est_x, 2),
            "est_y": round(est_y, 2),
            "est_z": round(est_z, 2),
            "real_mm": round(real_mm, 2),
            "err_mm": round(err, 2),
            "err_pct": round(pct, 2),
        })

    return err, pct, est_mm


# ── Drawing ───────────────────────────────────────────────────────────────────
def draw_brackets(frame, pts, colour, thickness=3):
    blen = max(12, int(np.linalg.norm(pts[1] - pts[0]) * 0.25))
    for j, pt in enumerate(pts):
        for nb in [pts[(j+1)%4], pts[(j-1)%4]]:
            d = (nb - pt).astype(float)
            n = d / (np.linalg.norm(d) + 1e-6)
            cv2.line(frame, tuple(pt), tuple((pt + n*blen).astype(int)), colour, thickness)


def draw_legend(frame, detectors, active):
    lh, pad, bw = 20, 8, 220
    bh = len(detectors) * lh + pad * 2
    x0, y0 = frame.shape[1] - bw - 10, frame.shape[0] - bh - 10
    ov = frame.copy()
    cv2.rectangle(ov, (x0, y0), (x0+bw, y0+bh), (20, 20, 20), -1)
    cv2.addWeighted(ov, 0.55, frame, 0.45, 0, frame)
    for i, d in enumerate(detectors):
        y = y0 + pad + i*lh + lh//2
        c = d["colour"] if d["name"] in active else tuple(v//4 for v in d["colour"])
        cv2.circle(frame, (x0+12, y), 5, c, -1)
        cv2.putText(frame, d["name"], (x0+22, y+4), cv2.FONT_HERSHEY_SIMPLEX, 0.36, c, 1)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    if args.list_dicts:
        print("\n".join(f"  {n}" for n in REGISTRY))
        sys.exit(0)

    names = list(REGISTRY) if args.dicts is None else [
        n.lower() for n in args.dicts if n.lower() in REGISTRY
    ]
    if not names:
        print("No valid dictionaries. Use --list-dicts.")
        sys.exit(1)

    mtx, dist, calib_size = load_calibration(args.calib)
    w = calib_size[0] if calib_size else args.width
    h = calib_size[1] if calib_size else args.height
    obj_pts = obj_points(args.marker_size)

    try:
        cv2.aruco.ArucoDetector; use_new_api = True
    except AttributeError:
        use_new_api = False

    detectors = build_detectors(names, use_new_api)
    print(f"Loaded {len(detectors)} detector(s): {', '.join(names)}")

    # Camera
    proc = cap = frame_size = None
    try:
        subprocess.check_call(["which", "rpicam-vid"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        proc = subprocess.Popen(
            ["rpicam-vid", "--width", str(w), "--height", str(h),
             "--framerate", str(args.fps), "--codec", "yuv420",
             "--timeout", "0", "--nopreview", "-o", "-"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        frame_size = w * h * 3 // 2
    except (FileNotFoundError, subprocess.CalledProcessError):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not cap.isOpened():
            print("Error: could not open camera."); sys.exit(1)
        ret, frame = cap.read()
        if ret:
            fh, fw = frame.shape[:2]
            if fw != w or fh != h:
                mtx[0,0] *= fw/w; mtx[1,1] *= fh/h
                mtx[0,2] *= fw/w; mtx[1,2] *= fh/h
                w, h = fw, fh

    print("q=quit  v=verify  d=toggle legend")
    budget = 1000.0 / args.fps
    show_all = True
    last = []

    while True:
        t0 = time.monotonic()

        if proc:
            raw = proc.stdout.read(frame_size)
            if len(raw) < frame_size: break
            yuv   = np.frombuffer(raw, dtype=np.uint8).reshape((h*3//2, w))
            frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        else:
            ret, frame = cap.read()
            if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detect_all(gray, detectors, use_new_api)
        if detections: last = detections

        active = {r["name"] for r in detections}
        total  = sum(len(r["ids"]) for r in detections)

        if detections:
            b = 6
            cv2.rectangle(frame, (b,b), (frame.shape[1]-b, frame.shape[0]-b), (0,255,0), b)
            for res in detections:
                rvecs, tvecs = estimate_pose(res["corners"], mtx, dist, obj_pts)
                for i, mid in enumerate(res["ids"]):
                    pts  = res["corners"][i][0].astype(int)
                    dist_mm = float(np.linalg.norm(tvecs[i]))
                    draw_brackets(frame, pts, res["colour"])
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], args.marker_size/2)
                    cv2.putText(frame, f"[{res['name']}] ID{mid[0]}  {dist_mm:.1f}mm",
                                (pts[0][0], pts[0][1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, res["colour"], 2)
            cv2.putText(frame, f"DETECTED: {total}  [{', '.join(sorted(active))}]",
                        (12,30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0), 2)
        else:
            cv2.putText(frame, "SCANNING...", (12,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,100,255), 2)

        draw_legend(frame, detectors if show_all else
                    [d for d in detectors if d["name"] in active], active)

        ms = (time.monotonic()-t0)*1000
        cv2.putText(frame, f"{ms:.1f}ms / {budget:.0f}ms  |  {len(detectors)} dicts",
                    (12, frame.shape[0]-12), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0,60,255) if ms > budget else (180,180,180), 1)

        cv2.imshow("ArUco Detector", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("d"):
            show_all = not show_all
        elif key == ord("v"):
            if not last:
                print("No markers detected yet.")
                continue
            for res in last:
                rvecs, tvecs = estimate_pose(res["corners"], mtx, dist, obj_pts)
                for i, mid in enumerate(res["ids"]):
                    tvec = tvecs[i].flatten()
                    ex, ey, ez = float(tvec[0]), float(tvec[1]), float(tvec[2])
                    z_est_mm = ez

                    print(f"\n  [{res['name']}] ID{mid[0]}")
                    print(f"  Estimated  x={ex:+.1f}  y={ey:+.1f}  z={ez:+.1f}  (mm)")

                    raw_in = input("  True Z distance / height (mm, blank=skip): ")
                    if not raw_in.strip():
                        print("  Skipped.")
                        continue

                    try:
                        true_z_mm = float(raw_in)
                        z_err = z_est_mm - true_z_mm
                        z_pct = (z_err / true_z_mm * 100.0) if true_z_mm != 0 else float("inf")

                        print(f"  Z error: {z_err:+.2f} mm ({z_pct:.2f}%)")
                    except ValueError:
                        print("  Invalid input.")

    if proc: proc.terminate()
    elif cap: cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
