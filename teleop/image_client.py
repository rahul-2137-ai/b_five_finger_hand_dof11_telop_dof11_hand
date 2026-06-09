"""
ZMQ image client for the MuJoCo sim camera server (modinoid_sim/sim_camera_server.py).

The sim server publishes ALL cameras in a SINGLE msgpack message on ONE port:

    {
        "images": {"<camera_name>": "<base64-encoded JPEG>", ...},
        "timestamp": <float seconds>,
    }

This module adapts that stream to the SAME interface the teleop code
(teleop_hand_and_arm.py) expects from `teleimager.ImageClient`, so it can be
used as a drop-in replacement:

    img_client   = ImageClient(host=..., request_bgr=True)
    cam_config   = img_client.get_cam_config()
    head_frame   = img_client.get_head_frame()          # head_frame.bgr -> np.ndarray
    left_frame   = img_client.get_left_wrist_frame()
    right_frame  = img_client.get_right_wrist_frame()
    img_client.close()

Standalone display (debug):
    python image_client.py --host 127.0.0.1 --port 5555
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

# Camera names as published by sim_camera_server.py (see modinoid_sim/config.yaml:
# CAMERA_HEAD_NAME / CAMERA_WRIST_NAMES).
HEAD_CAM_NAME = "ego_view"
LEFT_WRIST_CAM_NAME = "ego_view_left_mono"
RIGHT_WRIST_CAM_NAME = "ego_view_right_mono"


def decode(b64_str: str) -> Optional[np.ndarray]:
    """Decode a base64 JPEG string to a BGR image."""
    raw = base64.b64decode(b64_str)
    arr = np.frombuffer(raw, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)  # BGR


class SimImage:
    """Minimal stand-in for teleimager.TeleImage: exposes `.bgr` and `.fps`."""
    __slots__ = ["_bgr", "fps"]

    def __init__(self, bgr: Optional[np.ndarray], fps: float = 0.0):
        self._bgr = bgr
        self.fps = fps

    @property
    def bgr(self) -> Optional[np.ndarray]:
        return self._bgr

    def __bool__(self):
        return self._bgr is not None


class ImageClient:
    """Drop-in replacement for `teleimager.ImageClient`, backed by the sim's
    single-port msgpack camera stream.

    A background thread keeps only the latest decoded frame per camera, so the
    teleop main loop's get_*_frame() calls are non-blocking and always return
    the freshest image.
    """

    def __init__(self, host="127.0.0.1", port=5555, request_bgr: bool = True,
                 head_resolution=(480, 640), wrist_resolution=(480, 640),
                 fps=30, **kwargs):
        """
        Args:
            host:             IP/hostname of the sim camera server.
            port:             ZMQ PUB port of the sim camera server (config CAMERA_ZMQ_PORT).
            request_bgr:      Kept for API compatibility; this client always decodes to BGR.
            head_resolution:  (H, W) of the head camera, for the reported cam_config.
            wrist_resolution: (H, W) of each wrist camera, for the reported cam_config.
            fps:              Nominal stream fps, for the reported cam_config.
        """
        self._host = host
        self._port = port
        self._head_resolution = tuple(head_resolution)
        self._wrist_resolution = tuple(wrist_resolution)
        self._fps = fps

        # latest decoded frame per camera name (BGR np.ndarray)
        self._lock = threading.Lock()
        self._frames = {}
        self._fps_est = 0.0
        self._running = True

        # SUB socket
        self._ctx = zmq.Context.instance()
        self._socket = self._ctx.socket(zmq.SUB)
        self._socket.setsockopt(zmq.RCVHWM, 2)          # keep only the latest frames
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.connect(f"tcp://{self._host}:{self._port}")
        logger_mp.info(f"[ImageClient] subscribing to tcp://{self._host}:{self._port}")

        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        # Probe the first message (up to ~2s) to learn which cameras the
        # server actually publishes, so cam_config reflects reality.
        available = self._wait_for_cameras(timeout=2.0)
        self._cam_config = self._build_cam_config(available)
        logger_mp.info(f"[ImageClient] available cameras: {sorted(available)}")

    # --------------------------------------------------------
    # internal
    # --------------------------------------------------------
    def _build_cam_config(self, available: set) -> dict:
        """Build a cam_config dict shaped like teleimager's, so teleop's
        camera_config[...] accesses all work unchanged."""
        return {
            "head_camera": {
                "enable_zmq": HEAD_CAM_NAME in available,
                "enable_webrtc": False,
                "binocular": False,                      # ego_view is a single mono camera
                "image_shape": list(self._head_resolution),
                "fps": self._fps,
                "zmq_port": self._port,
                "webrtc_port": 8080,                     # unused (webrtc disabled)
            },
            "left_wrist_camera": {
                "enable_zmq": LEFT_WRIST_CAM_NAME in available,
                "enable_webrtc": False,
                "image_shape": list(self._wrist_resolution),
                "fps": self._fps,
                "zmq_port": self._port,
            },
            "right_wrist_camera": {
                "enable_zmq": RIGHT_WRIST_CAM_NAME in available,
                "enable_webrtc": False,
                "image_shape": list(self._wrist_resolution),
                "fps": self._fps,
                "zmq_port": self._port,
            },
        }

    def _wait_for_cameras(self, timeout: float) -> set:
        """Block until the first frame arrives (or timeout), then return the
        set of camera names seen. Falls back to head-only if nothing arrives."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._frames:
                    return set(self._frames.keys())
            time.sleep(0.02)
        logger_mp.warning(
            f"[ImageClient] no frames from tcp://{self._host}:{self._port} within "
            f"{timeout}s; assuming head camera only. Is the sim camera server running?")
        return {HEAD_CAM_NAME}

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
                images_raw = data.get("images", {})

                decoded = {}
                for cam_name, b64 in images_raw.items():
                    if isinstance(b64, str):
                        img = decode(b64)
                        if img is not None:
                            decoded[cam_name] = img

                # fps estimate from message arrival rate
                now = time.perf_counter()
                if last_t is not None:
                    dt = now - last_t
                    if dt > 0:
                        inst = 1.0 / dt
                        self._fps_est = inst if self._fps_est == 0.0 else 0.9 * self._fps_est + 0.1 * inst
                last_t = now

                with self._lock:
                    self._frames.update(decoded)
            except Exception as e:
                if self._running:
                    logger_mp.error(f"[ImageClient] recv loop error: {e}")

    def _get_frame(self, cam_name: str) -> SimImage:
        with self._lock:
            img = self._frames.get(cam_name)
            fps = self._fps_est
        return SimImage(bgr=img, fps=fps)

    # --------------------------------------------------------
    # public api (matches teleimager.ImageClient)
    # --------------------------------------------------------
    def get_cam_config(self):
        return self._cam_config

    def get_head_frame(self) -> SimImage:
        return self._get_frame(HEAD_CAM_NAME)

    def get_left_wrist_frame(self) -> SimImage:
        return self._get_frame(LEFT_WRIST_CAM_NAME)

    def get_right_wrist_frame(self) -> SimImage:
        return self._get_frame(RIGHT_WRIST_CAM_NAME)

    def close(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self._socket.close(linger=0)
        except Exception as e:
            logger_mp.warning(f"[ImageClient] error closing socket: {e}")
        logger_mp.info("[ImageClient] closed.")


def main():
    """Standalone debug viewer — stacks all received cameras side by side."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=5555, type=int)
    args = parser.parse_args()

    client = ImageClient(host=args.host, port=args.port, request_bgr=True)
    cam_config = client.get_cam_config()
    logger_mp.info(f"cam_config: {cam_config}")

    TARGET_H = 320
    try:
        while True:
            frames = {
                "head": client.get_head_frame(),
                "left_wrist": client.get_left_wrist_frame(),
                "right_wrist": client.get_right_wrist_frame(),
            }
            strips = []
            for label, frame in frames.items():
                if frame.bgr is None:
                    continue
                img = frame.bgr
                h, w = img.shape[:2]
                scale = TARGET_H / h
                thumb = cv2.resize(img, (int(w * scale), TARGET_H))
                cv2.putText(thumb, f"{label} {frame.fps:.1f}fps", (8, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 80), 2)
                div = np.zeros((TARGET_H, 3, 3), dtype=np.uint8)
                strips += [thumb, div]

            if strips:
                grid = np.hstack(strips[:-1])
                cv2.imshow("Sim Camera Feed", grid)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(0.005)
    finally:
        client.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
