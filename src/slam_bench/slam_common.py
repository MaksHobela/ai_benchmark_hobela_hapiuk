"""Shared helpers: data download/extraction, evaluation, process runner, recording, tables."""
import os, sys, json, time, shutil, subprocess, signal, itertools, threading
from pathlib import Path
import numpy as np

DRIVE   = Path('/content/drive/MyDrive/slam_bench')
BUILDS  = DRIVE / 'builds'
RESULTS = DRIVE / 'results'
WORK    = Path('/content/work')
OPT     = Path('/opt/slam')            # unpacked C++ builds
SRC     = Path('/content/src')         # unpacked DPVO / DROID sources
REPO_ID = 'ucu-autonomous-ugv/ucu-slam-dataset-v1'


def log(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


class NotAvailable(Exception):
    pass


# ------------------------------------------------------------------ data
def download_seq(n, token):
    os.environ.setdefault('HF_XET_HIGH_PERFORMANCE', '1')
    from huggingface_hub import hf_hub_download
    p = hf_hub_download(REPO_ID, f'sequence-{n}/sequence-{n}.mcap', repo_type='dataset',
                        token=token, local_dir=str(WORK / 'hf'))
    shutil.rmtree(Path.home() / '.cache/huggingface/xet', ignore_errors=True)
    return Path(p)


def drop_raw(n):
    shutil.rmtree(WORK / 'hf' / f'sequence-{n}', ignore_errors=True)
    shutil.rmtree(WORK / 'hf' / '.cache', ignore_errors=True)


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _to_bgr(m):
    import cv2
    a = np.frombuffer(bytes(m.data), np.uint8)
    if m.encoding in ('rgb8', 'bgr8'):
        a = a.reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        return np.ascontiguousarray(a[:, :, ::-1] if m.encoding == 'rgb8' else a)
    if m.encoding == 'mono8':
        a = a.reshape(m.height, m.step)[:, :m.width]
        return cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
    raise ValueError('unsupported image encoding ' + m.encoding)


def extract_seq(mcap_path, out):
    """MCAP -> rgb/*.jpg, rgb.txt, imu.txt, odom.txt, groundtruth.txt, calib.json, tf_static.json.
    Depth topics are NOT read (depth is broken in v1)."""
    import cv2
    from mcap_ros2.reader import read_ros2_messages
    out = Path(out)
    (out / 'rgb').mkdir(parents=True, exist_ok=True)
    rgb, imu, odom, gt, calib, tfs = [], [], [], [], {}, []
    topics = ['/cam_color/image_raw', '/cam_color/camera_info', '/imu', '/odom', '/pose', '/tf_static']
    for m in read_ros2_messages(str(mcap_path), topics=topics):
        tp, x = m.channel.topic, m.ros_msg
        if tp == '/cam_color/image_raw':
            t = _stamp(x.header)
            name = f'rgb/{t:017.6f}.jpg'
            cv2.imwrite(str(out / name), _to_bgr(x), [cv2.IMWRITE_JPEG_QUALITY, 95])
            rgb.append((t, name))
        elif tp == '/imu':
            a, w = x.linear_acceleration, x.angular_velocity
            imu.append([_stamp(x.header), w.x, w.y, w.z, a.x, a.y, a.z])
        elif tp in ('/odom', '/pose'):
            p, q = x.pose.pose.position, x.pose.pose.orientation
            (odom if tp == '/odom' else gt).append([_stamp(x.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        elif tp == '/cam_color/camera_info' and not calib:
            calib = dict(K=list(x.k), D=list(x.d), w=int(x.width), h=int(x.height), model=x.distortion_model)
        elif tp == '/tf_static':
            for tr in x.transforms:
                T, R = tr.transform.translation, tr.transform.rotation
                tfs.append(dict(parent=tr.header.frame_id, child=tr.child_frame_id,
                                t=[T.x, T.y, T.z], q=[R.x, R.y, R.z, R.w]))
    rgb.sort()
    with open(out / 'rgb.txt', 'w') as f:
        f.write('# color images\n# file: rgb.txt\n# timestamp filename\n')
        f.writelines(f'{t:.6f} {n}\n' for t, n in rgb)
    for name, arr in (('imu.txt', imu), ('odom.txt', odom), ('groundtruth.txt', gt)):
        arr = np.array(sorted(arr))
        np.savetxt(out / name, arr, fmt=['%.6f'] + ['%.9g'] * (arr.shape[1] - 1) if len(arr) else '%s')
    json.dump(calib, open(out / 'calib.json', 'w'), indent=1)
    json.dump(tfs, open(out / 'tf_static.json', 'w'), indent=1)
    log(f'extracted {out.name}: {len(rgb)} rgb, {len(imu)} imu, {len(odom)} odom, {len(gt)} gt')
    return len(rgb)


def tf_pose(tfs, target, source):
    """4x4 pose of frame `source` expressed in frame `target` (BFS over static tf tree)."""
    from scipy.spatial.transform import Rotation as Rot

    def mat(d):
        M = np.eye(4); M[:3, :3] = Rot.from_quat(d['q']).as_matrix(); M[:3, 3] = d['t']; return M
    edges = {}
    for d in tfs:
        M = mat(d)
        edges.setdefault(d['parent'], []).append((d['child'], M))
        edges.setdefault(d['child'], []).append((d['parent'], np.linalg.inv(M)))
    seen, queue = {target: np.eye(4)}, [target]
    while queue:
        a = queue.pop(0)
        for b, M in edges.get(a, []):
            if b not in seen:
                seen[b] = seen[a] @ M; queue.append(b)
    return seen.get(source)


# ------------------------------------------------------------------ evaluation
def load_tum(p):
    a = np.loadtxt(p, ndmin=2)
    return a[np.argsort(a[:, 0])]


def save_tum(p, arr):
    np.savetxt(p, arr, fmt=['%.6f'] + ['%.9g'] * 7)


def umeyama(X, Y, scale=True):
    """s,R,t minimising |Y - (s R X + t)|, X,Y: Nx3."""
    mx, my = X.mean(0), Y.mean(0)
    Xc, Yc = X - mx, Y - my
    U, D, Vt = np.linalg.svd(Yc.T @ Xc / len(X))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = (D * np.diag(S)).sum() / ((Xc ** 2).sum() / len(X)) if scale else 1.0
    return s, R, my - s * R @ mx


def aligned(est_file, gt_file, scale=True):
    est, gt = load_tum(est_file), load_tum(gt_file)
    m = (gt[:, 0] >= est[0, 0]) & (gt[:, 0] <= est[-1, 0])
    if m.sum() < 10:
        return None
    T, G = gt[m, 0], gt[m, 1:4]
    P = np.stack([np.interp(T, est[:, 0], est[:, i]) for i in (1, 2, 3)], 1)
    s, R, t = umeyama(P, G, scale)
    return T, (s * (R @ P.T)).T + t, G, s, est, gt


def evaluate(est_file, gt_file, radius=5.0, min_dt=30.0):
    """GT is position-only & meter-level, so: ATE on positions (Sim3 and SE3 alignment),
    horizontal ATE, and GT-revisit consistency as the loop-closure quality measure."""
    from scipy.spatial import cKDTree
    r = aligned(est_file, gt_file, True)
    if r is None:
        return dict(error='too few overlapping poses')
    T, A, G, s, est, gt = r
    out = dict(n_est=int(len(est)), n_gt=int(len(gt)), n_matched=int(len(T)),
               coverage=float((est[-1, 0] - est[0, 0]) / (gt[-1, 0] - gt[0, 0])), scale=float(s))
    for tag, sc in (('sim3', True), ('se3', False)):
        _, Aa, Gg, _, _, _ = aligned(est_file, gt_file, sc)
        d = Aa - Gg
        e, eh = np.linalg.norm(d, axis=1), np.linalg.norm(d[:, :2], axis=1)
        out.update({f'ate_{tag}_rmse': float(np.sqrt((e ** 2).mean())), f'ate_{tag}_mean': float(e.mean()),
                    f'ate_{tag}_median': float(np.median(e)), f'ate_{tag}_max': float(e.max()),
                    f'ate_{tag}_horiz_rmse': float(np.sqrt((eh ** 2).mean()))})
    # loop-closure quality: for GT revisits (close in space, far in time) how close are the *estimated* points
    pairs = cKDTree(G).query_pairs(radius, output_type='ndarray')
    if len(pairs):
        pairs = pairs[np.abs(T[pairs[:, 1]] - T[pairs[:, 0]]) > min_dt]
    out['loop_pairs'] = int(len(pairs))
    if len(pairs):
        err = np.linalg.norm(A[pairs[:, 0]] - A[pairs[:, 1]], axis=1)
        out.update(loop_revisit_mean=float(err.mean()), loop_revisit_median=float(np.median(err)),
                   loop_revisit_max=float(err.max()), loop_ok_ratio=float((err < 2 * radius).mean()))
    out['end_gap_est'] = float(np.linalg.norm(A[0] - A[-1]))
    out['end_gap_gt'] = float(np.linalg.norm(G[0] - G[-1]))
    return out


def plot_traj(est_file, gt_file, png, title=''):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    r = aligned(est_file, gt_file, True)
    if r is None:
        return
    T, A, G, *_ = r
    plt.figure(figsize=(6, 6))
    plt.plot(G[:, 0], G[:, 1], 'k-', lw=1, label='GNSS GT')
    plt.plot(A[:, 0], A[:, 1], 'r-', lw=1, label='estimate (Sim3-aligned)')
    plt.axis('equal'); plt.legend(); plt.title(title); plt.grid(alpha=.3)
    plt.savefig(png, dpi=110, bbox_inches='tight'); plt.close()


def traj_video(est_file, gt_file, mp4, label='', n_frames=450, size=720):
    """Cheap, always-works animation of estimated vs GT trajectory."""
    import cv2
    r = aligned(est_file, gt_file, True)
    if r is None:
        return
    T, A, G, *_ = r
    allp = np.vstack([A[:, :2], G[:, :2]])
    lo, hi = allp.min(0), allp.max(0)
    sc = (size - 60) / max((hi - lo).max(), 1e-6)
    px = lambda P: ((P[:, :2] - lo) * sc + 30).astype(np.int32)
    pg, pa = px(G), px(A)
    vw = cv2.VideoWriter(str(mp4), cv2.VideoWriter_fourcc(*'mp4v'), 30, (size, size))
    for k in np.linspace(2, len(T) - 1, n_frames).astype(int):
        im = np.full((size, size, 3), 255, np.uint8)
        cv2.polylines(im, [pg.reshape(-1, 1, 2)], False, (150, 150, 150), 2)
        cv2.polylines(im, [pa[:k].reshape(-1, 1, 2)], False, (0, 0, 220), 2)
        cv2.circle(im, tuple(int(v) for v in pa[k]), 6, (0, 120, 0), -1)
        cv2.putText(im, f'{label}  t={T[k]-T[0]:.0f}s', (15, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 2)
        vw.write(im)
    vw.release()


# ------------------------------------------------------------------ processes
def run_cmd(cmd, logfile, timeout, cwd=None, env=None):
    e = os.environ.copy(); e.update({k: str(v) for k, v in (env or {}).items()})
    sh = isinstance(cmd, str)
    with open(logfile, 'w') as f:
        p = subprocess.Popen(cmd, shell=sh, executable='/bin/bash' if sh else None, stdout=f,
                             stderr=subprocess.STDOUT, cwd=cwd, env=e, start_new_session=True)
        try:
            return p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL); p.wait()
            return 'TIMEOUT'


_disp = itertools.count(90)
_disp_lock = threading.Lock()


class X11Rec:
    """Virtual display + ffmpeg screen recording, for native viewers (Pangolin)."""
    def __init__(self, mp4, size='1280x720', fps=15):
        with _disp_lock:
            self.disp = f':{next(_disp)}'
        self.mp4, self.size, self.fps = str(mp4), size, fps
        self.env = {'DISPLAY': self.disp, 'LIBGL_ALWAYS_SOFTWARE': '1'}

    def __enter__(self):
        dn = subprocess.DEVNULL
        self.xvfb = subprocess.Popen(['Xvfb', self.disp, '-screen', '0', self.size + 'x24'], stdout=dn, stderr=dn)
        time.sleep(2)
        self.ff = subprocess.Popen(['ffmpeg', '-y', '-f', 'x11grab', '-framerate', str(self.fps), '-video_size',
                                    self.size, '-i', self.disp, '-c:v', 'libx264', '-preset', 'ultrafast',
                                    '-pix_fmt', 'yuv420p', self.mp4], stdin=subprocess.PIPE, stdout=dn, stderr=dn)
        return self

    def __exit__(self, *a):
        try:
            self.ff.stdin.write(b'q'); self.ff.stdin.flush(); self.ff.wait(30)
        except Exception:
            self.ff.kill()
        self.xvfb.terminate()


# ------------------------------------------------------------------ tables
def collect():
    import pandas as pd
    rows = []
    for st in RESULTS.glob('seq-*/*/status.json'):
        d = json.load(open(st))
        d['seq'] = int(st.parts[-3].split('-')[1]); d['pipeline'] = st.parts[-2]
        mf = st.parent / 'metrics.json'
        if mf.exists():
            d.update(json.load(open(mf)))
        rows.append(d)
    return pd.DataFrame(rows)


def show_tables(df=None, save=True):
    import pandas as pd
    from IPython.display import display
    df = collect() if df is None else df
    if df.empty:
        print('no results yet'); return df
    specs = [('ate_sim3_rmse', 'ATE RMSE, m (Sim3 align; scale-free)'), ('ate_sim3_mean', 'ATE mean, m (Sim3)'),
             ('ate_sim3_max', 'ATE MAX deviation, m (Sim3)'), ('ate_se3_rmse', 'ATE RMSE, m (SE3, metric scale kept)'),
             ('loop_revisit_mean', 'Loop closure: mean est. distance at GT revisits, m (lower=better)'),
             ('loop_ok_ratio', 'Loop closure: share of revisits closed (<10 m)'),
             ('end_gap_est', 'Start-end gap of estimate, m (GT gap in end_gap_gt)'),
             ('coverage', 'Trajectory coverage of GT time span'), ('runtime_s', 'Runtime, s'),
             ('status', 'Status')]
    out = RESULTS / '_tables'; out.mkdir(parents=True, exist_ok=True)
    pd.set_option('display.width', 250); pd.set_option('display.max_columns', 50)
    for col, title in specs:
        if col not in df:
            continue
        pv = df.pivot(index='seq', columns='pipeline', values=col)
        if col != 'status':
            pv = pv.astype(float); pv.loc['MEAN'] = pv.mean(); pv.loc['MAX'] = pv.drop('MEAN').max()
            pv = pv.round(3)
        print('\n###', title); display(pv)
        if save:
            pv.to_csv(out / f'{col}.csv')
    if save:
        df.to_csv(out / 'all_metrics.csv', index=False)
    return df
