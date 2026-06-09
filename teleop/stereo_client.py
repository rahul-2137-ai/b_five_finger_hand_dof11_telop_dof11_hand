"""ZMQ stereo image client — views the high-res stereo pair from the sim.

The stereo server (a second SimCameraServer inside modinoid_sim.py) publishes a
msgpack dict on its own port (default 5577):

    {
        "images": {"stereo_left": "<b64 jpeg>", "stereo_right": "<b64 jpeg>"},
        "timestamp": <float>,
    }

This client shows the two eyes side by side (LEFT | RIGHT). Press 'a' to toggle
a red/cyan anaglyph view (use red/cyan glasses), 'q' to quit.

Run:
    python stereo_client.py                  # localhost:5577
    python stereo_client.py --port 5577
    python stereo_client.py --host 192.168.1.10 --port 5577
"""

import argparse
import base64
import threading
import time
from typing import Optional

import cv2
import msgpack
import numpy as np
import zmq

import logging_mp
logger_mp = logging_mp.get_logger(__name__)
logger_mp.setLevel(logging_mp.INFO)

LEFT = "stereo_left"
RIGHT = "stereo_right"


def decode(b64_str: str) -> np.ndarray:
    raw = base64.b64decode(b64_str)
    arr = np.frombuffer(raw, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)  # BGR


def anaglyph(left_bgr: np.ndarray, right_bgr: np.ndarray) -> np.ndarray:
    """Red (left) / cyan (right) anaglyph for red-cyan 3D glasses."""
    h = min(left_bgr.shape[0], right_bgr.shape[0])
    w = min(left_bgr.shape[1], right_bgr.shape[1])
    l = cv2.resize(left_bgr, (w, h))
    r = cv2.resize(right_bgr, (w, h))
    out = np.zeros((h, w, 3), dtype=np.uint8)
    out[:, :, 2] = l[:, :, 2]   # Red   from left eye
    out[:, :, 1] = r[:, :, 1]   # Green from right eye
    out[:, :, 0] = r[:, :, 0]   # Blue  from right eye
    return out


class StereoImageClient:
    """Background-threaded subscriber for the sim's stereo pair, shaped for VR.

    A receive thread keeps only the latest decoded left/right frames, so
    `get_stereo_frame()` is non-blocking and always returns the freshest image.
    Each eye is resized to `eye_resolution` and the two are hstacked into a
    single `(H, 2*W, 3)` BGR image — the binocular layout `televuer` expects for
    `render_to_xr()`. This client is *display only*; the ego_view dataset
    cameras keep flowing through the unchanged `teleop/image_client.py`.
    """

    def __init__(self, host="127.0.0.1", port=5577, eye_resolution=None,
                 request_bgr: bool = True, **kwargs):
        """
        Args:
            host:           IP/hostname of the stereo camera server.
            port:           ZMQ PUB port of the stereo server (default 5577).
            eye_resolution: (H, W) to resize each eye to. Default None = adopt the
                            server's NATIVE per-eye resolution (probed from the
                            first frame) so nothing is downscaled — this keeps the
                            full equirect sharpness and 1:1 aspect. The frame from
                            get_stereo_frame() is (H, 2*W, 3), and `image_shape`
                            reports [H, 2*W] for televuer.
            request_bgr:    Kept for API parity with ImageClient; always BGR.
        """
        self._host = host
        self._port = port
        # None until known: when eye_resolution is given we lock to it, otherwise
        # we adopt the first frame's native size (set in _wait_for_first_frame).
        self._eye_h = int(eye_resolution[0]) if eye_resolution else None
        self._eye_w = int(eye_resolution[1]) if eye_resolution else None

        self._lock = threading.Lock()
        self._frame = None          # latest hstacked (H, 2W, 3) BGR image
        self._fps_est = 0.0
        self._running = True

        self._ctx = zmq.Context.instance()
        self._socket = self._ctx.socket(zmq.SUB)
        self._socket.setsockopt(zmq.RCVHWM, 2)          # keep only the latest frames
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.connect(f"tcp://{self._host}:{self._port}")
        logger_mp.info(f"[StereoImageClient] subscribing to tcp://{self._host}:{self._port}")

        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        # Block briefly so image_shape is valid before televuer is configured.
        self._wait_for_first_frame(timeout=3.0)

    def _wait_for_first_frame(self, timeout: float):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._eye_h is not None and self._frame is not None:
                    logger_mp.info(f"[StereoImageClient] per-eye resolution: "
                                   f"{self._eye_w}x{self._eye_h} (HxW reported as image_shape).")
                    return
            time.sleep(0.02)
        if self._eye_h is None:
            self._eye_h, self._eye_w = 640, 640      # fallback if server not up yet
            logger_mp.warning(f"[StereoImageClient] no frame within {timeout}s; "
                              f"falling back to {self._eye_w}x{self._eye_h}. Is the stereo server running?")

    # ----- properties used to configure televuer's binocular display -----
    @property
    def binocular(self) -> bool:
        return True

    @property
    def image_shape(self):
        """[H, 2*W] of the hstacked stereo frame returned by get_stereo_frame()."""
        return [self._eye_h, self._eye_w * 2]

    @property
    def fps(self) -> float:
        return self._fps_est

    def _recv_loop(self):
        poller = zmq.Poller()
        poller.register(self._socket, zmq.POLLIN)
        last_t = None
        while self._running:
            try:
                events = dict(poller.poll(timeout=100))
                if self._socket not in events:
                    continue
                packed = self._socket.recv()
                data = msgpack.unpackb(packed, raw=False)
                images = data.get("images", {})

                left = decode(images[LEFT]) if isinstance(images.get(LEFT), str) else None
                right = decode(images[RIGHT]) if isinstance(images.get(RIGHT), str) else None
                if left is None or right is None:
                    continue

                # Lock the per-eye size to the first frame's NATIVE resolution
                # (unless a fixed eye_resolution was requested). No downscaling =
                # no extra blur; full server sharpness reaches the headset.
                if self._eye_h is None:
                    self._eye_h, self._eye_w = left.shape[0], left.shape[1]

                # Only resize if the incoming eye differs from the locked size
                # (keeps the hstacked frame at the fixed (H, 2W) televuer expects).
                if left.shape[:2] != (self._eye_h, self._eye_w):
                    left = cv2.resize(left, (self._eye_w, self._eye_h))
                if right.shape[:2] != (self._eye_h, self._eye_w):
                    right = cv2.resize(right, (self._eye_w, self._eye_h))
                stereo = np.hstack((left, right))

                now = time.perf_counter()
                if last_t is not None:
                    dt = now - last_t
                    if dt > 0:
                        inst = 1.0 / dt
                        self._fps_est = inst if self._fps_est == 0.0 else 0.9 * self._fps_est + 0.1 * inst
                last_t = now

                with self._lock:
                    self._frame = stereo
            except Exception as e:
                if self._running:
                    logger_mp.error(f"[StereoImageClient] recv loop error: {e}")

    def get_stereo_frame(self) -> Optional[np.ndarray]:
        """Latest hstacked LEFT|RIGHT BGR image (H, 2W, 3), or None if no frame yet."""
        with self._lock:
            return self._frame

    def close(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self._socket.close(linger=0)
        except Exception as e:
            logger_mp.warning(f"[StereoImageClient] error closing socket: {e}")
        logger_mp.info("[StereoImageClient] closed.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", default=5577, type=int)
    parser.add_argument("--height", default=480, type=int,
                        help="display height per eye (downscaled from the hi-res feed)")
    args = parser.parse_args()

    ctx = zmq.Context()
    socket = ctx.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVTIMEO, 3000)
    socket.setsockopt(zmq.RCVHWM, 2)                 # keep only the latest frame
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.connect(f"tcp://{args.host}:{args.port}")
    print(f"Connected to tcp://{args.host}:{args.port}  —  'a' anaglyph, 'q' quit")

    t0 = time.time()
    frames = 0
    use_anaglyph = False

    while True:
        try:
            packed = socket.recv()
        except zmq.Again:
            print("Waiting for stereo server...")
            continue

        data = msgpack.unpackb(packed, raw=False)
        images = data.get("images", {})
        left = decode(images[LEFT]) if isinstance(images.get(LEFT), str) else None
        right = decode(images[RIGHT]) if isinstance(images.get(RIGHT), str) else None
        if left is None or right is None:
            continue

        frames += 1
        fps = frames / (time.time() - t0)

        if use_anaglyph:
            view = anaglyph(left, right)
            scale = args.height / view.shape[0]
            view = cv2.resize(view, (int(view.shape[1] * scale), args.height))
            cv2.putText(view, "ANAGLYPH (red/cyan)", (8, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 80), 2)
        else:
            strips = []
            for name, img in ((LEFT, left), (RIGHT, right)):
                scale = args.height / img.shape[0]
                thumb = cv2.resize(img, (int(img.shape[1] * scale), args.height))
                cv2.putText(thumb, "LEFT EYE" if name == LEFT else "RIGHT EYE",
                            (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 80), 2)
                div = np.zeros((args.height, 3, 3), dtype=np.uint8)
                strips += [thumb, div]
            view = np.hstack(strips[:-1])

        cv2.putText(view, f"FPS: {fps:.1f}", (10, view.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Stereo view (port %d)" % args.port, view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("a"):
            use_anaglyph = not use_anaglyph

    socket.close()
    ctx.term()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
