"""
apriltag_detect.py  —  Native AprilTag detection with pose estimation
Requires: pip install pupil-apriltags

Usage
-----
python apriltag_detect.py                           # scan all families
python apriltag_detect.py --families tag36h11       # specific family
python apriltag_detect.py --families tag36h11 tag16h5 --marker-size 150
python apriltag_detect.py --list-families           # show all names and exit

Controls
--------
  q  Quit
  v  Verify distance (prompts for real measurement, saves to CSV)
"""

import argparse
import csv
import os
import subprocess
import sys
import time

import cv2
import numpy as np

try:
    from pupil_apriltags import Detector
except ImportError:
    print("Error: 'pupil_apriltags' package not found.")
    print("Install it with:  pip install pupil-apriltags")
    sys.exit(1)

# ── Available families ────────────────────────────────────────────────────────
FAMILIES = [
    "tag16h5",
    "tag25h9",
    "tag36h10",
    "tag36h11",
    "tagCircle21h7",
    "tagCircle49h12",
    "tagCustom48h12",
    "tagStandard41h12",
    "tagStandard52h13",
]

_PALETTE = [
    (0, 255, 100), (0, 180, 255), (255, 80, 0), (0, 0, 255),
    (255, 0, 180), (0, 255, 220), (180, 0, 255), (255, 200, 0),
]


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--families", "-f", nargs="+", metavar="FAMILY", default=None,
                   help="Families to enable (default: all). See --list-families.")
    p.add_argument("--list-families", "-l", action="store_true",
                   help="Print available family names and exit.")
    p.add_argument("--marker-size", "-s", type=float, default=100.0, metavar="MM",
                   help="Physical marker side length in mm (default: 100).")
    p.add_argument("--calib", "-c", default="calibration.npz", metavar="FILE")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--csv", default="apriltag_results.csv", metavar="FILE")
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
def build_detectors(families):
    """One detector per family so results stay individually labelled."""
    out = []
    for i, fam in enumerate(families):
        det = Detector(families=fam)
        out.append({"family": fam, "det": det, "colour": _PALETTE[i % len(_PALETTE)]})
    return out


def detect_all(gray, detectors, mtx, marker_size_mm):
    """
    Run every detector and normalise corners to OpenCV order (TL,TR,BR,BL).
    pupil_apriltags returns corners in order: TL, TR, BR, BL already.
    """
    fx, fy = mtx[0, 0], mtx[1, 1]
    cx, cy = mtx[0, 2], mtx[1, 2]
    tag_size_m = marker_size_mm / 1000.0

    results = []
    for d in detectors:
        hits = d["det"].detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy),
            tag_size=tag_size_m,
        )
        if not hits:
            continue
        corners, ids, poses = [], [], []
        for h in hits:
            c = h.corners.reshape(1, 4, 2).astype(np.float32)
            corners.append(c)
            ids.append(h.tag_id)
            poses.append((h.pose_R, h.pose_t))
        results.append({
            "corners": corners,
            "ids": ids,
            "poses": poses,
            "family": d["family"],
            "colour": d["colour"],
        })
    return results


# ── Pose ──────────────────────────────────────────────────────────────────────
def obj_points(size_mm):
    h = size_mm / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def pose_to_rvec_tvec(pose_R, pose_t, marker_size_mm):
    """Convert pupil_apriltags pose (metres) to rvec/tvec in mm for drawing."""
    tvec = pose_t.flatten() * 1000.0  # metres -> mm
    rvec, _ = cv2.Rodrigues(pose_R)
    return rvec, tvec.reshape(3, 1)


# ── CSV ───────────────────────────────────────────────────────────────────────
def save_csv(path, family, tag_id, est_mm, real_mm):
    err = est_mm - real_mm
    pct = abs(err) / real_mm * 100 if real_mm else float("inf")
    exists = os.path.isfile(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["family","id","est_mm","real_mm","err_mm","err_pct"])
        if not exists:
            w.writeheader()
        w.writerow({"family": family, "id": tag_id,
                    "est_mm": round(est_mm, 2), "real_mm": round(real_mm, 2),
                    "err_mm": round(err, 2), "err_pct": round(pct, 2)})
    return err, pct


# ── Drawing ───────────────────────────────────────────────────────────────────
def draw_brackets(frame, pts, colour, thickness=3):
    blen = max(12, int(np.linalg.norm(pts[1] - pts[0]) * 0.25))
    for j, pt in enumerate(pts):
        for nb in [pts[(j+1)%4], pts[(j-1)%4]]:
            d = (nb - pt).astype(float)
            n = d / (np.linalg.norm(d) + 1e-6)
            cv2.line(frame, tuple(pt), tuple((pt + n*blen).astype(int)), colour, thickness)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    if args.list_families:
        print("\n".join(f"  {f}" for f in FAMILIES))
        sys.exit(0)

    active_families = FAMILIES if args.families is None else [
        f for f in args.families if f in FAMILIES
    ]
    if not active_families:
        print("No valid families. Use --list-families.")
        sys.exit(1)

    mtx, dist, calib_size = load_calibration(args.calib)
    w = calib_size[0] if calib_size else args.width
    h = calib_size[1] if calib_size else args.height
    obj_pts = obj_points(args.marker_size)

    detectors = build_detectors(active_families)
    print(f"Loaded {len(detectors)} family detector(s): {', '.join(active_families)}")

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

    print("q=quit  v=verify")
    budget = 1000.0 / args.fps
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
        detections = detect_all(gray, detectors, mtx, args.marker_size)
        if detections: last = detections

        total = sum(len(r["ids"]) for r in detections)

        if detections:
            b = 6
            cv2.rectangle(frame, (b,b), (frame.shape[1]-b, frame.shape[0]-b), (0,255,100), b)
            for res in detections:
                for i, tag_id in enumerate(res["ids"]):
                    pts = res["corners"][i][0].astype(int)
                    pose_R, pose_t = res["poses"][i]
                    dist_mm = float(np.linalg.norm(pose_t)) * 1000.0
                    rvec, tvec = pose_to_rvec_tvec(pose_R, pose_t, args.marker_size)
                    draw_brackets(frame, pts, res["colour"])
                    cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, args.marker_size / 2)
                    cv2.putText(frame, f"[{res['family']}] ID{tag_id}  {dist_mm:.1f}mm",
                                (pts[0][0], pts[0][1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, res["colour"], 2)
            active_fams = ", ".join(sorted({r["family"] for r in detections}))
            cv2.putText(frame, f"DETECTED: {total}  [{active_fams}]",
                        (12,30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,100), 2)
        else:
            cv2.putText(frame, "SCANNING...", (12,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,100,255), 2)

        ms = (time.monotonic()-t0)*1000
        cv2.putText(frame, f"{ms:.1f}ms / {budget:.0f}ms  |  {len(detectors)} families",
                    (12, frame.shape[0]-12), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0,60,255) if ms > budget else (180,180,180), 1)

        cv2.imshow("AprilTag Detector", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("v"):
            if not last:
                print("No tags detected yet.")
                continue
            for res in last:
                for i, tag_id in enumerate(res["ids"]):
                    pose_R, pose_t = res["poses"][i]
                    est = float(np.linalg.norm(pose_t)) * 1000.0
                    print(f"  [{res['family']}] ID{tag_id}  est={est:.2f}mm")
                    raw_in = input("  True distance (mm, blank=skip): ")
                    if not raw_in.strip(): continue
                    try:
                        err, pct = save_csv(args.csv, res["family"], tag_id, est, float(raw_in))
                        print(f"  Error: {err:+.2f}mm ({pct:.2f}%)  -> {args.csv}")
                    except ValueError:
                        print("  Invalid input.")

    if proc: proc.terminate()
    elif cap: cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
