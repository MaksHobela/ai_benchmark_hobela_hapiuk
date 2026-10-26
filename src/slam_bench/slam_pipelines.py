"""Pipeline wrappers. Each p_*(ctx) returns a path to a TUM trajectory: t x y z qx qy qz qw."""
import re, json, shutil, os
from pathlib import Path
import numpy as np
from slam_common import *

CFG = dict(
    stride_orb=2, stride_dso=2, stride_dpvo=2, stride_droid=6, stride_gtsam=6, stride_ov=2,
    orb_pace=2.0,                       # ORB-SLAM3 replay speed (x real time); too fast -> mapping can't keep up
    yolo_weights='yolov8n-seg.pt',
    dyn_classes=[0, 1, 2, 3, 5, 7, 14, 15, 16, 17],   # COCO: person, bicycle, car, motorcycle, bus, truck, bird, cat, dog, horse
    # !!! replace with the Allan-variance values from calibration.yaml (these are generic, inflated defaults)
    imu_noise=dict(gyro_nd=2.0e-3, gyro_rw=2.0e-5, acc_nd=2.0e-2, acc_rw=3.0e-3, rate=125.0),
    droid_buffer=1000,
)


class Ctx:
    def __init__(self, n, seq, calib, tfs, frames):
        self.n, self.seq, self.calib, self.tfs, self.frames = n, Path(seq), calib, tfs, frames
        self.res = RESULTS / f'seq-{n}'
        self.out = self.res; self.timeout = 900; self.viz = False; self.mask_ok = {}

    def child(self, name, timeout, viz):
        c = Ctx(self.n, self.seq, self.calib, self.tfs, self.frames)
        c.mask_ok = self.mask_ok; c.out = self.res / name; c.out.mkdir(parents=True, exist_ok=True)
        c.timeout, c.viz = timeout, viz
        return c


def load_ctx(n, seq):
    seq = Path(seq)
    frames = [(float(l.split()[0]), l.split()[1]) for l in open(seq / 'rgb.txt') if not l.startswith('#')]
    return Ctx(n, seq, json.load(open(seq / 'calib.json')), json.load(open(seq / 'tf_static.json')), frames)


# ------------------------------------------------------------------ input preparation
def prep_set(ctx, variant, stride):
    """Undistorted, strided image set for a variant (raw / yolo / sam) + all per-tool list files."""
    import cv2
    d = ctx.seq / f'set_{variant}_s{stride}'
    frames = ctx.frames[::stride]
    ts = np.array([t for t, _ in frames])
    K = np.array(ctx.calib['K']).reshape(3, 3); D = np.array(ctx.calib['D'][:5], float)
    W, H = ctx.calib['w'], ctx.calib['h']
    info = dict(dir=d, ts=ts, w=W, h=H, fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2])
    if (d / 'DONE').exists():
        return info
    (d / 'images').mkdir(parents=True, exist_ok=True)
    srcdir = ctx.seq / {'raw': 'rgb', 'yolo': 'rgb_yolo', 'sam': 'rgb_sam'}[variant]
    for i, (t, name) in enumerate(frames):
        p = srcdir / Path(name).name
        img = cv2.imread(str(p if p.exists() else ctx.seq / name))
        cv2.imwrite(str(d / 'images' / f'{i:06d}.jpg'), cv2.undistort(img, K, D), [cv2.IMWRITE_JPEG_QUALITY, 95])
    with open(d / 'rgb.txt', 'w') as f:
        f.write('# color images\n# file: rgb.txt\n# timestamp filename\n')
        f.writelines(f'{t:.6f} images/{i:06d}.jpg\n' for i, t in enumerate(ts))
    with open(d / 'times.txt', 'w') as f:
        f.writelines(f'{i} {t:.6f} 0.0\n' for i, t in enumerate(ts))
    open(d / 'camera.txt', 'w').write(f'Pinhole {info["fx"]} {info["fy"]} {info["cx"]} {info["cy"]} 0\n{W} {H}\ncrop\n{W} {H}\n')
    open(d / 'calib.txt', 'w').write(f'{info["fx"]} {info["fy"]} {info["cx"]} {info["cy"]}\n')
    (d / 'DONE').touch()
    return info


def remap_times(traj, ts):
    """Trajectories indexed by frame number -> real timestamps."""
    idx = traj[:, 0].astype(int)
    ok = (idx >= 0) & (idx < len(ts)) & np.allclose(traj[:, 0], idx)
    if not ok.all():                      # tool already wrote real timestamps
        return traj
    traj = traj.copy(); traj[:, 0] = ts[idx]
    return traj


def need(path, what):
    if not Path(path).exists():
        raise NotAvailable(f'{what} not found at {path} (run 01_build_once.ipynb)')


# ------------------------------------------------------------------ masks (YOLOv8-Seg / MobileSAM)
def make_masks(ctx, kind):
    """Blank out potentially dynamic objects (people, vehicles, animals). Every 2nd frame only (all strides nest)."""
    import cv2
    from ultralytics import YOLO, SAM
    out = ctx.seq / ('rgb_yolo' if kind == 'yolo' else 'rgb_sam'); out.mkdir(exist_ok=True)
    frames = ctx.frames[::2]
    yolo = YOLO(CFG['yolo_weights'])
    sam = SAM('mobile_sam.pt') if kind == 'sam' else None
    n_det, B = 0, 32
    for s in range(0, len(frames), B):
        chunk = frames[s:s + B]
        paths = [str(ctx.seq / 'rgb' / Path(n).name) for _, n in chunk]
        res = yolo.predict(paths, imgsz=640, conf=0.3, classes=CFG['dyn_classes'], retina_masks=True,
                           verbose=False, device=0)
        for (t, n), p, r in zip(chunk, paths, res):
            dst = out / Path(n).name
            if dst.exists():
                continue
            if r.boxes is None or len(r.boxes) == 0 or r.masks is None:
                os.symlink(p, dst); continue
            img = cv2.imread(p); H, W = img.shape[:2]
            if kind == 'yolo':
                m = r.masks.data.any(0).cpu().numpy().astype(np.uint8)
            else:
                rr = sam.predict(img, bboxes=r.boxes.xyxy.cpu().numpy().tolist(), verbose=False)
                m = rr[0].masks.data.any(0).cpu().numpy().astype(np.uint8)
            if m.shape != (H, W):
                m = cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST)
            img[cv2.dilate(m, np.ones((15, 15), np.uint8)) > 0] = 127
            cv2.imwrite(str(dst), img, [cv2.IMWRITE_JPEG_QUALITY, 95]); n_det += 1
    json.dump(dict(frames=len(frames), frames_with_masks=n_det), open(ctx.res / f'masks_{kind}.json', 'w'))
    log(f'seq {ctx.n}: {kind} masks on {n_det}/{len(frames)} frames')


# ------------------------------------------------------------------ ORB-SLAM3 (mono)
def orb_yaml(S, path, fps):
    open(path, 'w').write(f"""%YAML:1.0
Camera.type: "PinHole"
Camera.fx: {S['fx']}
Camera.fy: {S['fy']}
Camera.cx: {S['cx']}
Camera.cy: {S['cy']}
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.width: {S['w']}
Camera.height: {S['h']}
Camera.fps: {fps}
Camera.RGB: 0
ORBextractor.nFeatures: 1500
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
""")


def p_orb(ctx, variant='raw'):
    exe = OPT / 'bin/orb_run'; need(exe, 'orb_run'); need(OPT / 'share/ORBvoc.txt', 'ORB vocabulary')
    st = CFG['stride_orb']; S = prep_set(ctx, variant, st)
    yaml = ctx.out / 'orb_settings.yaml'; orb_yaml(S, yaml, 30 // st)
    cmd = [str(exe), str(OPT / 'share/ORBvoc.txt'), str(yaml), str(S['dir']), str(ctx.out),
           str(CFG['orb_pace']), '1' if ctx.viz else '0']
    env = {'LD_LIBRARY_PATH': f"{OPT}/lib:" + os.environ.get('LD_LIBRARY_PATH', '')}
    if ctx.viz:
        with X11Rec(ctx.out / 'native_viewer.mp4') as rec:
            rc = run_cmd(cmd, ctx.out / 'run.log', ctx.timeout, env={**env, **rec.env})
    else:
        rc = run_cmd(cmd, ctx.out / 'run.log', ctx.timeout, env=env)
    txt = open(ctx.out / 'run.log').read()
    json.dump(dict(loops=len(re.findall(r'Loop detected', txt)), merges=len(re.findall(r'Merge detected', txt)),
                   rc=str(rc)), open(ctx.out / 'extra.json', 'w'))
    f = ctx.out / 'CameraTrajectory.txt'
    if rc != 0 and not f.exists():
        raise RuntimeError(f'orb_run rc={rc}, see run.log')
    return f


# ------------------------------------------------------------------ DSO
def p_dso(ctx, variant='raw'):
    exe = OPT / 'bin/dso_dataset'; need(exe, 'dso_dataset')
    S = prep_set(ctx, variant, CFG['stride_dso'])
    cmd = [str(exe), f"files={S['dir']}/images", f"calib={S['dir']}/camera.txt", 'mode=1', 'preset=0',
           'speed=0', 'quiet=1', 'nolog=1', f"nogui={0 if ctx.viz else 1}"]
    env = {'LD_LIBRARY_PATH': f"{OPT}/lib:" + os.environ.get('LD_LIBRARY_PATH', '')}
    if ctx.viz:
        with X11Rec(ctx.out / 'native_viewer.mp4') as rec:
            rc = run_cmd(cmd, ctx.out / 'run.log', ctx.timeout, cwd=ctx.out, env={**env, **rec.env})
    else:
        rc = run_cmd(cmd, ctx.out / 'run.log', ctx.timeout, cwd=ctx.out, env=env)
    f = ctx.out / 'result.txt'
    if not f.exists():
        raise RuntimeError(f'dso rc={rc}, no result.txt, see run.log')
    tr = np.loadtxt(f, ndmin=2)[:, :8]
    # DSO writes the timestamp column from times.txt, but fall back to index mapping if needed
    save_tum(ctx.out / 'traj_tum.txt', remap_times(tr, S['ts']))
    return ctx.out / 'traj_tum.txt'


# ------------------------------------------------------------------ OpenVINS (standalone driver, no ROS)
def p_openvins(ctx):
    exe = OPT / 'bin/ov_run'; need(exe, 'ov_run')
    tpl = OPT / 'share/ov_template'; need(tpl / 'estimator_config.yaml', 'OpenVINS template config')
    Tic = tf_pose(ctx.tfs, 'imu', 'camera_color')          # pose of camera in IMU frame (cam -> imu)
    if Tic is None:
        raise NotAvailable('no static tf between imu and camera_color')
    c, K, D = ctx.calib, ctx.calib['K'], ctx.calib['D']
    cd = ctx.out / 'ov_cfg'; cd.mkdir(exist_ok=True)
    est = open(tpl / 'estimator_config.yaml').read()
    for k, v in (('max_cameras', '1'), ('use_stereo', 'false'), ('verbosity', '"WARNING"')):
        est = re.sub(rf'^{k}:.*$', f'{k}: {v}', est, flags=re.M)
    open(cd / 'estimator_config.yaml', 'w').write(est)
    n = CFG['imu_noise']
    imu = open(tpl / 'kalibr_imu_chain.yaml').read()
    for k, v in (('accelerometer_noise_density', n['acc_nd']), ('accelerometer_random_walk', n['acc_rw']),
                 ('gyroscope_noise_density', n['gyro_nd']), ('gyroscope_random_walk', n['gyro_rw']),
                 ('update_rate', n['rate'])):
        imu = re.sub(rf'^(\s*){k}:.*$', rf'\g<1>{k}: {v}', imu, flags=re.M)
    open(cd / 'kalibr_imu_chain.yaml', 'w').write(imu)
    rows = '\n'.join('    - [' + ', '.join(f'{x:.9f}' for x in r) + ']' for r in Tic)
    open(cd / 'kalibr_imucam_chain.yaml', 'w').write(f"""%YAML:1.0
cam0:
  T_imu_cam:
{rows}
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [{', '.join(str(x) for x in D[:4])}]
  distortion_model: radtan
  intrinsics: [{K[0]}, {K[4]}, {K[2]}, {K[5]}]
  resolution: [{c['w']}, {c['h']}]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 0.0
""")
    out = ctx.out / 'traj_tum.txt'
    env = {'LD_LIBRARY_PATH': f"{OPT}/lib:" + os.environ.get('LD_LIBRARY_PATH', '')}
    rc = run_cmd([str(exe), str(cd / 'estimator_config.yaml'), str(ctx.seq), str(out), str(CFG['stride_ov'])],
                 ctx.out / 'run.log', ctx.timeout, env=env)
    if not out.exists() or out.stat().st_size == 0:
        raise RuntimeError(f'ov_run rc={rc}, no output (init failed?), see run.log')
    return out


def p_dso_ov_fusion(ctx, win=30.0):
    """OUR INTERPRETATION of 'DSO + OpenVINS': loosely-coupled fusion. DSO gives the shape (no drift in scale
    within window), OpenVINS gives metric scale: DSO increments are rescaled per window so that the window path
    length matches OpenVINS. Output has positions only (identity orientation)."""
    a, b = ctx.res / 'DSO' / 'traj_tum.txt', ctx.res / 'OpenVINS' / 'traj_tum.txt'
    if not (a.exists() and b.exists()):
        raise NotAvailable('needs successful DSO and OpenVINS runs')
    d, o = load_tum(a), load_tum(b)
    t0, t1 = max(d[0, 0], o[0, 0]), min(d[-1, 0], o[-1, 0])
    d = d[(d[:, 0] >= t0) & (d[:, 0] <= t1)]
    if len(d) < 20:
        raise RuntimeError('no time overlap between DSO and OpenVINS')
    O = np.stack([np.interp(d[:, 0], o[:, 0], o[:, i]) for i in (1, 2, 3)], 1)
    s, R, t = umeyama(d[:, 1:4], O, True)                      # DSO -> OpenVINS frame
    P = (s * (R @ d[:, 1:4].T)).T + t
    out = np.zeros((len(d), 3)); out[0] = O[0]
    edges = np.arange(d[0, 0], d[-1, 0] + win, win)
    for lo, hi in zip(edges[:-1], edges[1:]):
        idx = np.where((d[:, 0] >= lo) & (d[:, 0] < hi))[0]
        if len(idx) < 2:
            continue
        lp = np.linalg.norm(np.diff(P[idx], axis=0), axis=1).sum()
        lo_ = np.linalg.norm(np.diff(O[idx], axis=0), axis=1).sum()
        k = np.clip(lo_ / lp, 0.5, 2.0) if lp > 1e-6 else 1.0
        for j in idx:
            if j > 0:
                out[j] = out[j - 1] + k * (P[j] - P[j - 1])
    res = np.zeros((len(d), 8)); res[:, 0] = d[:, 0]; res[:, 1:4] = out; res[:, 7] = 1
    save_tum(ctx.out / 'traj_tum.txt', res)
    return ctx.out / 'traj_tum.txt'


def p_vidso(ctx):
    raise NotAvailable('VI-DSO (Stumberg et al.) has no official public code. Provide a fork/binary and wire it here.')


# ------------------------------------------------------------------ DPVO
def p_dpvo(ctx, variant='raw'):
    src = SRC / 'DPVO'; need(src / 'demo.py', 'DPVO'); need(src / 'dpvo.pth', 'DPVO weights')
    S = prep_set(ctx, variant, CFG['stride_dpvo'])
    name = f'seq{ctx.n}_{variant}'
    base = f"python demo.py --imagedir {S['dir']}/images --calib {S['dir']}/calib.txt --stride 1 --name {name} " \
           f"--network dpvo.pth --config config/default.yaml --save_trajectory"
    f = src / 'saved_trajectories' / f'{name}.txt'
    f.unlink(missing_ok=True)
    rc = run_cmd(base + ' --opts BUFFER_SIZE 4096', ctx.out / 'run.log', ctx.timeout, cwd=src)
    if not f.exists() and rc != 'TIMEOUT':           # key may not exist in this DPVO version -> default config
        rc = run_cmd(base, ctx.out / 'run.log2', ctx.timeout, cwd=src)
    if not f.exists():
        raise RuntimeError(f'DPVO rc={rc}, no trajectory, see run.log')
    save_tum(ctx.out / 'traj_tum.txt', remap_times(np.loadtxt(f, ndmin=2)[:, :8], S['ts']))
    return ctx.out / 'traj_tum.txt'


# ------------------------------------------------------------------ DROID-SLAM
def p_droid(ctx, variant='raw'):
    src = SRC / 'DROID-SLAM'; need(src / 'droid_slam/droid.py', 'DROID-SLAM'); need(src / 'droid.pth', 'DROID weights')
    S = prep_set(ctx, variant, CFG['stride_droid'])
    npy = ctx.out / 'traj.npy'
    cmd = [sys.executable, str(OPT / 'share/run_droid.py'), '--src', str(src), '--imagedir', f"{S['dir']}/images",
           '--calib', f"{S['dir']}/calib.txt", '--out', str(npy), '--buffer', str(CFG['droid_buffer'])]
    rc = run_cmd(cmd, ctx.out / 'run.log', ctx.timeout)
    if not npy.exists():
        raise RuntimeError(f'DROID rc={rc}, no trajectory, see run.log')
    tr = np.load(npy)                                 # N x 7: t xyz qxyzw, camera-to-world
    arr = np.hstack([np.arange(len(tr))[:, None], tr])
    save_tum(ctx.out / 'traj_tum.txt', remap_times(arr, S['ts']))
    return ctx.out / 'traj_tum.txt'


# ------------------------------------------------------------------ SuperPoint + LightGlue -> GTSAM
def p_sp_lg_gtsam(ctx):
    """Own pipeline (depth is unusable in v1): SuperPoint+LightGlue -> essential-matrix relative poses (metric scale
    from wheel odometry) -> GTSAM iSAM2 pose graph; loop closures found by LightGlue verification of
    pose-proximal keyframes. GT is never used by the estimator."""
    import torch, cv2, gtsam
    from lightglue import LightGlue, SuperPoint
    from lightglue.utils import rbd
    S = prep_set(ctx, 'raw', CFG['stride_gtsam']); ts = S['ts']; N = len(ts)
    Kc = np.array([[S['fx'], 0, S['cx']], [0, S['fy'], S['cy']], [0, 0, 1.]])
    od = np.loadtxt(ctx.seq / 'odom.txt', ndmin=2)
    ox = np.stack([np.interp(ts, od[:, 0], od[:, i]) for i in (1, 2, 3)], 1)
    dev = 'cuda'
    ext = SuperPoint(max_num_keypoints=1024).eval().to(dev)
    mat = LightGlue(features='superpoint').eval().to(dev)

    def feats(i):
        im = cv2.cvtColor(cv2.imread(str(S['dir'] / 'images' / f'{i:06d}.jpg')), cv2.COLOR_BGR2RGB)
        with torch.no_grad():
            return ext.extract(torch.from_numpy(im).permute(2, 0, 1).float().div(255).to(dev))

    def rel(f0, f1):
        with torch.no_grad():
            m = mat({'image0': f0, 'image1': f1})
        a, b, m = rbd(f0), rbd(f1), rbd(m)
        mm = m['matches'].cpu().numpy()
        if len(mm) < 25:
            return None
        p0 = a['keypoints'][mm[:, 0]].cpu().numpy(); p1 = b['keypoints'][mm[:, 1]].cpu().numpy()
        E, mask = cv2.findEssentialMat(p0, p1, Kc, cv2.RANSAC, 0.999, 1.0)
        if E is None or E.shape != (3, 3):
            return None
        n, R, t, _ = cv2.recoverPose(E, p0, p1, Kc, mask=mask)
        return R, t.ravel(), int(n)

    def pose(R, t):    # cam1-from-cam0 [R|t]  ->  pose of cam1 in cam0 frame
        return gtsam.Pose3(gtsam.Rot3(R.T), gtsam.Point3(*(-R.T @ t)))

    isam = gtsam.ISAM2(gtsam.ISAM2Params())
    F = [feats(0)]
    g, v = gtsam.NonlinearFactorGraph(), gtsam.Values()
    g.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(), gtsam.noiseModel.Diagonal.Sigmas(np.full(6, 1e-3))))
    v.insert(0, gtsam.Pose3()); isam.update(g, v)
    cur = [gtsam.Pose3()]; pos = [np.zeros(3)]; last_rel = gtsam.Pose3(); loops = []; cum = 0.0
    min_gap = max(40, int(20 * 30 / CFG['stride_gtsam']))
    for i in range(1, N):
        F.append(feats(i))
        d = float(np.linalg.norm(ox[i] - ox[i - 1])); cum += d
        r = rel(F[i - 1], F[i]) if d > 0.01 else None
        if r is not None and r[2] >= 30:
            rp = pose(r[0], r[1] / (np.linalg.norm(r[1]) + 1e-12) * d); sig = [0.03] * 3 + [0.05 * d + 0.01] * 3
            last_rel = rp
        else:                                      # stationary or visual failure -> hold last motion, wide noise
            rp = gtsam.Pose3(last_rel.rotation(), gtsam.Point3(*(last_rel.translation() / (np.linalg.norm(last_rel.translation()) + 1e-12) * d)))
            sig = [0.2] * 3 + [0.3 * d + 0.05] * 3
        g, v = gtsam.NonlinearFactorGraph(), gtsam.Values()
        g.add(gtsam.BetweenFactorPose3(i - 1, i, rp, gtsam.noiseModel.Diagonal.Sigmas(np.array(sig))))
        v.insert(i, cur[i - 1].compose(rp))
        # loop candidates: pose-proximal old keyframes, verified visually
        if i > min_gap:
            P = np.array(pos); dist = np.linalg.norm(P[:i - min_gap] - pos[i - 1], axis=1)
            radius = 8.0 + 0.03 * cum
            for j in np.argsort(dist)[:3]:
                if dist[j] > radius or any(abs(j - l[1]) < 15 and abs(i - l[0]) < 15 for l in loops):
                    continue
                r2 = rel(F[j], F[i])
                if r2 is None or r2[2] < 50:
                    continue
                dd = max(float(dist[j]), 1.0)
                lp = pose(r2[0], r2[1] / (np.linalg.norm(r2[1]) + 1e-12) * dd)
                huber = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(1.345),
                                                       gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1] * 3 + [0.5 * dd] * 3)))
                g.add(gtsam.BetweenFactorPose3(int(j), i, lp, huber)); loops.append((i, int(j), r2[2]))
        isam.update(g, v)
        if loops and loops[-1][0] == i:
            for _ in range(3):
                isam.update()
        est = isam.calculateEstimate()
        cur = [est.atPose3(k) for k in range(i + 1)] if (loops and loops[-1][0] == i) else cur + [est.atPose3(i)]
        pos = [p.translation() for p in cur]
        if i % 200 == 0:
            log(f'  gtsam kf {i}/{N} loops={len(loops)}')
    est = isam.calculateEstimate(); rows = []
    for k in range(N):
        p = est.atPose3(k); q = p.rotation().toQuaternion(); t = p.translation()
        rows.append([ts[k], t[0], t[1], t[2], q.x(), q.y(), q.z(), q.w()])
    save_tum(ctx.out / 'traj_tum.txt', np.array(rows))
    json.dump(dict(loops=len(loops), loop_edges=loops), open(ctx.out / 'extra.json', 'w'))
    return ctx.out / 'traj_tum.txt'
