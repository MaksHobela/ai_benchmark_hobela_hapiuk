"""Orchestrator: prefetch next sequence while the current one is processed; CPU and GPU pipelines run in parallel."""
import json, queue, shutil, threading, time, traceback
from concurrent.futures import ThreadPoolExecutor
from slam_common import *
from slam_pipelines import *


def safe(name):
    return re.sub(r'[^A-Za-z0-9_.+-]', '_', name)


def execute(ctx0, name, fn, timeout, viz=False, retry_failed=False):
    ctx = ctx0.child(safe(name), timeout, viz)
    st_file = ctx.out / 'status.json'
    if st_file.exists():
        old = json.load(open(st_file))['status']
        if old in ('OK', 'NOT_AVAILABLE') or (old == 'FAIL' and not retry_failed):
            return old
    t0, status, err = time.time(), 'OK', ''
    try:
        est = fn(ctx)
        if est is None or not Path(est).exists() or Path(est).stat().st_size == 0:
            raise RuntimeError('no trajectory produced')
        est = Path(est)
        if est.name != 'traj_tum.txt':
            shutil.copy(est, ctx.out / 'traj_tum.txt')
        gt = ctx.seq / 'groundtruth.txt'
        m = evaluate(ctx.out / 'traj_tum.txt', gt); m['runtime_s'] = round(time.time() - t0, 1)
        json.dump(m, open(ctx.out / 'metrics.json', 'w'), indent=1)
        if 'error' in m:
            status, err = 'FAIL', m['error']
        else:
            plot_traj(ctx.out / 'traj_tum.txt', gt, ctx.out / 'trajectory.png', f'seq {ctx.n} | {name}')
            traj_video(ctx.out / 'traj_tum.txt', gt, ctx.out / 'trajectory.mp4', f'seq {ctx.n} | {name}')
    except NotAvailable as e:
        status, err = 'NOT_AVAILABLE', str(e)
    except Exception:
        status, err = 'FAIL', traceback.format_exc()[-1500:]
    json.dump(dict(status=status, error=err, wall_s=round(time.time() - t0, 1)), open(st_file, 'w'), indent=1)
    log(f'seq {ctx.n:>2} | {name:<28} {status:<14} {time.time() - t0:6.0f}s {err.splitlines()[-1][:90] if err else ""}')
    return status


def _masked(ctx, kind, fn):
    def g(c):
        if not ctx.mask_ok.get(kind):
            raise RuntimeError(f'{kind} mask generation failed')
        return fn(c)
    return g


def run_sequence(n, seq_dir, budget, workers, viz, retry_failed):
    ctx = load_ctx(n, seq_dir); ctx.res.mkdir(parents=True, exist_ok=True)
    ev = {'yolo': threading.Event(), 'sam': threading.Event()}
    ex_ = lambda c, nm, f, to, v=False: execute(c, nm, f, to, v, retry_failed)

    cpu = [('ORB-SLAM3', lambda c: p_orb(c, 'raw'), None, viz),
           ('DSO', lambda c: p_dso(c, 'raw'), None, viz),
           ('OpenVINS', p_openvins, None, False),
           ('YOLOv8Seg+ORB-SLAM3', lambda c: p_orb(c, 'yolo'), 'yolo', False),
           ('YOLOv8Seg+DSO', lambda c: p_dso(c, 'yolo'), 'yolo', False),
           ('MobileSAM+ORB-SLAM3', lambda c: p_orb(c, 'sam'), 'sam', False)]
    gpu = [('SuperPoint+LightGlue+GTSAM', p_sp_lg_gtsam),
           ('DPVO', lambda c: p_dpvo(c, 'raw')),
           ('DROID-SLAM', lambda c: p_droid(c, 'raw')),
           ('YOLOv8Seg+DPVO', _masked(ctx, 'yolo', lambda c: p_dpvo(c, 'yolo'))),
           ('YOLOv8Seg+DROID-SLAM', _masked(ctx, 'yolo', lambda c: p_droid(c, 'yolo')))]
    t_cpu = max(150, 0.75 * budget * workers / len(cpu))
    t_gpu = max(150, 0.75 * budget / (len(gpu) + 1))

    def gpu_thread():
        for kind in ('yolo', 'sam'):
            try:
                make_masks(ctx, kind); ctx.mask_ok[kind] = True
            except Exception:
                log(f'seq {n}: mask {kind} FAILED\n' + traceback.format_exc()[-800:])
            ev[kind].set()
        for name, fn in gpu:
            ex_(ctx, name, fn, t_gpu)

    def cpu_task(name, fn, need_ev, v):
        if need_ev:
            ev[need_ev].wait()
            if not ctx.mask_ok.get(need_ev):
                fn = _masked(ctx, need_ev, fn)
        return ex_(ctx, name, fn, t_cpu, v)

    g = threading.Thread(target=gpu_thread); g.start()
    with ThreadPoolExecutor(workers) as pool:
        futs = [pool.submit(cpu_task, *t) for t in cpu]
        for f in futs:
            f.result()
    g.join()
    ex_(ctx, 'DSO+OpenVINS', p_dso_ov_fusion, 300)
    ex_(ctx, 'DSO+VI-DSO', p_vidso, 10)


def seq_done(n):
    names = ['ORB-SLAM3', 'DSO', 'OpenVINS', 'YOLOv8Seg+ORB-SLAM3', 'YOLOv8Seg+DSO', 'MobileSAM+ORB-SLAM3',
             'SuperPoint+LightGlue+GTSAM', 'DPVO', 'DROID-SLAM', 'YOLOv8Seg+DPVO', 'YOLOv8Seg+DROID-SLAM',
             'DSO+OpenVINS', 'DSO+VI-DSO']
    for nm in names:
        f = RESULTS / f'seq-{n}' / safe(nm) / 'status.json'
        if not f.exists():
            return False
    return True


def run_all(seqs, token, hours=6.0, workers=3, retry_failed=False, viz_seqs=(1,), keep_prefix=True):
    WORK.mkdir(parents=True, exist_ok=True); RESULTS.mkdir(parents=True, exist_ok=True)
    deadline = time.time() + hours * 3600
    todo = [n for n in seqs if retry_failed or not seq_done(n)]
    log(f'sequences to process: {todo}')
    q = queue.Queue(maxsize=1)

    def producer():
        for n in todo:
            try:
                d = WORK / f'seq{n}'
                if not (d / 'rgb.txt').exists():
                    t0 = time.time(); raw = download_seq(n, token)
                    log(f'seq {n}: downloaded {raw.stat().st_size / 1e9:.1f} GB in {time.time() - t0:.0f}s')
                    shutil.rmtree(d, ignore_errors=True); extract_seq(raw, d); drop_raw(n)
                q.put((n, d))
            except Exception:
                log(f'seq {n}: download/extract FAILED\n' + traceback.format_exc()[-800:])
                drop_raw(n); q.put((n, None))
        q.put(None)

    threading.Thread(target=producer, daemon=True).start()
    left = len(todo)
    while True:
        item = q.get()
        if item is None:
            break
        n, d = item
        if time.time() > deadline:
            log('DEADLINE reached, stopping'); break
        if d is not None:
            budget = max(600, (deadline - time.time()) / left)
            log(f'=== seq {n} start (budget {budget / 60:.0f} min, {left} seq left) ===')
            try:
                run_sequence(n, d, budget, workers, n in viz_seqs, retry_failed)
            except Exception:
                log(f'seq {n}: run FAILED\n' + traceback.format_exc()[-1000:])
            shutil.rmtree(d, ignore_errors=True)          # "delete" the sequence, results stay on Drive
        left -= 1
    log('finished'); return collect()
