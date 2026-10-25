#!/usr/bin/env python3
"""
Render an offline video of one or several ORB-SLAM3 runs — no live viewer needed.

Left:  the camera frame the algorithm processed.
Right: top-down map: GNSS ground truth (gray) vs each ORB-SLAM3 trajectory
       (one color per run), drawn up to the current moment, with the current
       position error of every run and its overall ATE RMSE.

Each trajectory is aligned to the ground truth separately with Sim(3) (Umeyama),
the same alignment as `evo_ape -as`. RMSE is computed in the xy plane
(like `evo_ape ... --project_to_plane xy`).

Usage:
  python make_slam_video.py SEQ_DIR OUT.mp4 TRAJ [TRAJ ...] [--step 3] [--fps 15] [--fit gt|all]

Examples:
  python make_slam_video.py ~/datasets/ucu_seq1 ~/seq1.mp4 ~/datasets/ucu_seq1/KF_run1.txt
  python make_slam_video.py ~/datasets/ucu_seq1 ~/seq1_all.mp4 ~/datasets/ucu_seq1/KF_run*.txt

  --fit gt   (default) map is zoomed to the GNSS route; runs that drift far
             simply leave the map
  --fit all  map is zoomed to include every point of every run
"""
import argparse
import os

import cv2
import numpy as np

# BGR colors for runs
COLORS = [(200, 90, 30), (40, 40, 220), (40, 160, 40), (180, 60, 180), (0, 150, 230),
          (130, 100, 0), (90, 90, 200), (0, 120, 120)]


def read_tum(path):
    rows = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        rows.append([float(v) for v in line.split()[:4]])
    a = np.array(rows)
    return a[:, 0], a[:, 1:4]


def read_rgb_list(seq_dir):
    out = []
    for line in open(os.path.join(seq_dir, "rgb.txt")):
        if line.startswith("#") or not line.strip():
            continue
        t, name = line.split()[:2]
        out.append((float(t), os.path.join(seq_dir, name)))
    return out


def associate(t_a, t_b, max_diff):
    """For each time in t_a, the index of the closest time in t_b (or -1)."""
    idx = np.searchsorted(t_b, t_a)
    idx = np.clip(idx, 1, len(t_b) - 1)
    left, right = t_b[idx - 1], t_b[idx]
    idx = np.where(np.abs(t_a - left) <= np.abs(t_a - right), idx - 1, idx)
    ok = np.abs(t_b[idx] - t_a) <= max_diff
    return np.where(ok, idx, -1)


def umeyama_sim3(src, dst):
    """s, R, t minimizing ||dst - (s R src + t)||."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (xs ** 2).sum() / len(src)
    s = np.trace(np.diag(D) @ S) / var_s
    t = mu_d - s * R @ mu_s
    return s, R, t


class MapCanvas:
    def __init__(self, pts, size=(640, 480), margin=40, top=60):
        self.w, self.h = size
        lo, hi = pts[:, :2].min(0), pts[:, :2].max(0)
        span = np.maximum(hi - lo, 1e-6)
        self.scale = min((self.w - 2 * margin) / span[0], (self.h - 2 * margin - top) / span[1])
        self.center = (lo + hi) / 2
        self.top = top

    def px(self, xy):
        p = (np.atleast_2d(xy) - self.center) * self.scale
        x = self.w / 2 + p[:, 0]
        y = self.h / 2 + self.top / 2 - p[:, 1]  # north up
        return np.clip(np.stack([x, y], 1), -10000, 10000).astype(np.int32)

    def base(self, gt_xy):
        img = np.full((self.h, self.w, 3), 250, np.uint8)
        cv2.polylines(img, [self.px(gt_xy)], False, (170, 170, 170), 3, cv2.LINE_AA)
        # scale bar: 10 m, or 100 m if the map is large
        bar_m = 10 if 10 * self.scale >= 25 else 100
        L = int(bar_m * self.scale)
        cv2.line(img, (20, self.h - 20), (20 + L, self.h - 20), (60, 60, 60), 2)
        cv2.putText(img, f"{bar_m} m", (20, self.h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1, cv2.LINE_AA)
        return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seq_dir")
    ap.add_argument("out", help="output .mp4")
    ap.add_argument("trajs", nargs="+", help="one or more ORB-SLAM3 trajectories (TUM format)")
    ap.add_argument("--gt", help="ground truth file (default: SEQ_DIR/groundtruth.txt)")
    ap.add_argument("--step", type=int, default=3, help="use every N-th camera frame (default 3)")
    ap.add_argument("--fps", type=float, default=15, help="output video fps (default 15)")
    ap.add_argument("--fit", choices=["gt", "all"], default="gt", help="zoom map to GNSS route (gt) or to everything (all)")
    ap.add_argument("--t-max-diff", type=float, default=0.2)
    args = ap.parse_args()
    if not args.out.lower().endswith(".mp4"):
        ap.error("second argument must be the output .mp4 file")

    t_gt, p_gt = read_tum(args.gt or os.path.join(args.seq_dir, "groundtruth.txt"))
    frames = read_rgb_list(args.seq_dir)[:: args.step]

    runs = []
    for i, path in enumerate(args.trajs):
        t_est, p_est = read_tum(path)
        m = associate(t_est, t_gt, args.t_max_diff)
        ok = m >= 0
        if ok.sum() < 3:
            print(f"SKIP {path}: too few poses matched to ground truth")
            continue
        s, R, t = umeyama_sim3(p_est[ok], p_gt[m[ok]])
        p_al = (s * (R @ p_est.T)).T + t
        err = np.linalg.norm(p_al[ok, :2] - p_gt[m[ok], :2], axis=1)
        rmse = float(np.sqrt((err ** 2).mean()))
        name = os.path.splitext(os.path.basename(path))[0]
        print(f"{name}: {len(t_est)} keyframes, {ok.sum()} matched, ATE RMSE (xy) = {rmse:.2f} m, max = {err.max():.2f} m")
        runs.append(dict(name=name, t=t_est, p=p_al, rmse=rmse, color=COLORS[len(runs) % len(COLORS)]))
    if not runs:
        raise SystemExit("No usable trajectories.")

    if args.fit == "gt":
        span = p_gt[:, :2].max(0) - p_gt[:, :2].min(0)
        pad = 0.15 * span
        extent = np.vstack([p_gt[:, :2].min(0) - pad, p_gt[:, :2].max(0) + pad])
    else:
        extent = np.vstack([p_gt[:, :2]] + [r["p"][:, :2] for r in runs])
    canvas = MapCanvas(extent)
    base = canvas.base(p_gt[:, :2])
    gt_px = canvas.px(p_gt[:, :2])
    for r in runs:
        r["px"] = canvas.px(r["p"][:, :2])

    writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (1280, 480))
    t0 = frames[0][0]
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, (tf, path) in enumerate(frames):
        frame = cv2.imread(path)
        if frame is None:
            continue
        frame = cv2.resize(frame, (640, 480))
        mp = base.copy()

        g = min(np.searchsorted(t_gt, tf), len(t_gt) - 1)
        cv2.circle(mp, tuple(gt_px[g]), 7, (120, 120, 120), -1, cv2.LINE_AA)
        cv2.putText(mp, "GNSS (ground truth)", (15, 20), font, 0.45, (120, 120, 120), 1, cv2.LINE_AA)

        for j, r in enumerate(runs):
            k = np.searchsorted(r["t"], tf, side="right")  # keyframes up to now
            if k >= 2:
                cv2.polylines(mp, [r["px"][:k]], False, r["color"], 2, cv2.LINE_AA)
            if k >= 1:
                cv2.circle(mp, tuple(r["px"][k - 1]), 5, r["color"], -1, cv2.LINE_AA)
                if tf - r["t"][k - 1] > 2.0:
                    now = "no keyframe %.0fs" % (tf - r["t"][k - 1])
                else:
                    now = "now %.1f m" % np.linalg.norm(r["p"][k - 1, :2] - p_gt[g, :2])
            else:
                now = "not initialized"
            label = f"{r['name']}  RMSE {r['rmse']:.1f} m  |  {now}"
            cv2.putText(mp, label, (15, 40 + 18 * j), font, 0.45, r["color"], 1, cv2.LINE_AA)

        cv2.putText(frame, f"t = {tf - t0:6.1f} s", (10, 25), font, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        writer.write(np.hstack([frame, mp]))
        if i % 200 == 0:
            print(f"  {i}/{len(frames)} frames", flush=True)

    writer.release()
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()
