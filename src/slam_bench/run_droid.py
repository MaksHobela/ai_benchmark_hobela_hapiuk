"""Headless DROID-SLAM driver (based on the official demo.py). Saves camera-to-world poses (N x 7: xyz qxyzw)."""
import sys, os, argparse
p = argparse.ArgumentParser()
p.add_argument('--src'); p.add_argument('--imagedir'); p.add_argument('--calib'); p.add_argument('--out')
p.add_argument('--buffer', type=int, default=1000)
a = p.parse_args()
sys.path.insert(0, os.path.join(a.src, 'droid_slam'))
import numpy as np, cv2, torch
from droid import Droid


def image_stream():
    fx, fy, cx, cy = np.loadtxt(a.calib)[:4]
    for t, f in enumerate(sorted(os.listdir(a.imagedir))):
        im = cv2.imread(os.path.join(a.imagedir, f))
        h0, w0 = im.shape[:2]
        h1, w1 = int(h0 * np.sqrt((384 * 512) / (h0 * w0))), int(w0 * np.sqrt((384 * 512) / (h0 * w0)))
        im = cv2.resize(im, (w1, h1))[:h1 - h1 % 8, :w1 - w1 % 8]
        K = torch.as_tensor([fx, fy, cx, cy]); K[0::2] *= w1 / w0; K[1::2] *= h1 / h0
        yield t, torch.as_tensor(im).permute(2, 0, 1)[None], K


class Args:
    weights = os.path.join(a.src, 'droid.pth'); buffer = a.buffer; disable_vis = True; stereo = False
    beta = 0.3; filter_thresh = 2.4; warmup = 8; keyframe_thresh = 4.0; frontend_thresh = 16.0
    frontend_window = 25; frontend_radius = 2; frontend_nms = 1; backend_thresh = 22.0
    backend_radius = 2; backend_nms = 3; upsample = False; t0 = 0; image_size = None; reconstruction_path = None


args, droid = Args(), None
torch.multiprocessing.set_start_method('spawn', force=True)
for t, image, K in image_stream():
    if droid is None:
        args.image_size = [image.shape[2], image.shape[3]]
        droid = Droid(args)
    droid.track(t, image, intrinsics=K)
traj = droid.terminate(image_stream())
np.save(a.out, np.asarray(traj))
