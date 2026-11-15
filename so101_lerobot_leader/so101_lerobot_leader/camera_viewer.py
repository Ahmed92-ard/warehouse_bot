"""Leader-side dual camera viewer and follower microphone playback."""

from __future__ import annotations

import logging
import ctypes
import os
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

import cv2
import numpy as np
import rclpy
from PIL import Image, ImageOps, ImageTk
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from so101_lerobot_follower.msg import AudioChunk

CAMERA_TOPICS = (
    "/so101/follower/camera/image/compressed",
    "/so101/follower/camera2/image/compressed",
    "/mars/camera/image/compressed",
)
CAMERA_NAMES = ("Arm camera 1", "Arm camera 2", "Rover front camera")
AUDIO_TOPIC = "/so101/follower/audio/pcm"


def _set_parent_death_signal() -> None:
    """Exit this ROS child if its ros2 launch supervisor disappears."""
    parent_pid = os.getppid()
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        return
    if os.getppid() != parent_pid:
        os.kill(os.getpid(), signal.SIGTERM)


class LatestFrameBuffer:
    """A thread-safe one-frame slot that drops superseded compressed messages."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._latest: tuple[CompressedImage, float] | None = None
        self._closed = False
        self._scan_active = False

    def put(self, message: CompressedImage) -> bool:
        with self._condition:
            if self._closed:
                return False
            self._latest = (message, time.monotonic())
            self._condition.notify_all()
            return True

    def take_latest(self) -> tuple[CompressedImage, float] | None:
        with self._condition:
            message = self._latest
            self._latest = None
            return message

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._latest = None
            self._condition.notify_all()

    def set_scan_active(self, active: bool) -> None:
        with self._condition:
            self._scan_active = active

    @property
    def scan_active(self) -> bool:
        with self._condition:
            return self._scan_active

    @property
    def closed(self) -> bool:
        with self._condition:
            return self._closed


class AudioWaveformBuffer:
    """Thread-safe latest PCM sample window for the leader's small signal graph."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: tuple[np.ndarray, float] | None = None

    def put(self, data: bytes) -> None:
        samples = np.frombuffer(data, dtype="<i2").astype(np.float32) / 32768.0
        with self._lock:
            self._latest = (samples, time.monotonic())

    def snapshot(self) -> tuple[np.ndarray, float] | None:
        with self._lock:
            return self._latest


class CameraViewerNode(Node):
    def __init__(self, frames: list[LatestFrameBuffer]) -> None:
        super().__init__("leader_camera_viewer", namespace="so101")
        topics = (
            str(self.declare_parameter("camera_topic", CAMERA_TOPICS[0]).value),
            str(self.declare_parameter("camera2_topic", CAMERA_TOPICS[1]).value),
            str(self.declare_parameter("rover_camera_topic", CAMERA_TOPICS[2]).value),
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._media_subscriptions = [self.create_subscription(
            CompressedImage, topic, frames[index].put, qos) for index, topic in enumerate(topics)]
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._qr_scan_subscription = self.create_subscription(
            Bool, "/mars/warehouse/qr_scan_active",
            lambda message: frames[1].set_scan_active(bool(message.data)), scan_qos)
        self.get_logger().info(
            f"camera viewer subscribed to {', '.join(topics)} qos=best_effort depth=1")


class AudioPlayerNode(Node):
    """Play PCM on a worker thread so audio output never blocks ROS callbacks."""

    def __init__(self, waveform: AudioWaveformBuffer | None = None) -> None:
        super().__init__("leader_audio_player", namespace="so101")
        self.waveform = waveform
        self.topic = str(self.declare_parameter("audio_topic", AUDIO_TOPIC).value)
        self.device = str(self.declare_parameter("playback_device", "default").value)
        self.transmit_active = False
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pending: queue.Queue[tuple[bytes, int, int]] = queue.Queue(maxsize=3)
        self.subscription = self.create_subscription(AudioChunk, self.topic, self._on_audio, qos)
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mic_control_subscription = self.create_subscription(
            Bool, "/mars/audio/laptop_mic_enabled", self._on_mic_control,
            control_qos)
        self.running = True
        self.process: subprocess.Popen | None = None
        self.thread = threading.Thread(target=self._play_loop, name="audio-playback", daemon=True)
        self.thread.start()
        self.get_logger().info(f"follower audio playback topic={self.topic} output={self.device}")

    def _on_audio(self, msg: AudioChunk) -> None:
        if msg.encoding != "S16_LE" or msg.sample_rate != 16000 or msg.channels != 1:
            return
        data = bytes(msg.data)
        if self.waveform is not None:
            self.waveform.put(data)
        if self.transmit_active:
            return
        try:
            self.pending.put_nowait((data, msg.sample_rate, msg.channels))
        except queue.Full:
            try:
                self.pending.get_nowait()
            except queue.Empty:
                pass
            try:
                self.pending.put_nowait((data, msg.sample_rate, msg.channels))
            except queue.Full:
                pass

    def _on_mic_control(self, msg: Bool) -> None:
        self.transmit_active = bool(msg.data)
        if self.transmit_active:
            while True:
                try:
                    self.pending.get_nowait()
                except queue.Empty:
                    break

    def _play_loop(self) -> None:
        while self.running:
            try:
                data, rate, channels = self.pending.get(timeout=0.25)
            except queue.Empty:
                continue
            if self.transmit_active:
                continue
            if self.device.lower() in ("null", "none", "disabled"):
                continue
            if self.process is None or self.process.poll() is not None:
                try:
                    command = (
                        ["pw-cat", "--playback", "--format=s16", "--rate", str(rate),
                         "--channels", str(channels), "--latency=20ms", "-"]
                        if self.device in ("", "default") else
                        ["aplay", "-q", "-D", self.device, "-t", "raw", "-f", "S16_LE",
                         "-r", str(rate), "-c", str(channels)]
                    )
                    self.process = subprocess.Popen(
                        command,
                        stdin=subprocess.PIPE,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        bufsize=0,
                    )
                    self.get_logger().info("Leader audio output opened")
                except FileNotFoundError:
                    self.get_logger().error("Audio playback executable is missing on the leader")
                    return
            try:
                if self.process.stdin is None:
                    raise BrokenPipeError("aplay stdin is unavailable")
                self.process.stdin.write(data)
            except (BrokenPipeError, OSError) as error:
                self.get_logger().warning(f"Audio output stopped: {error}")
                self._stop_player()

    def _stop_player(self) -> None:
        process, self.process = self.process, None
        if process is not None:
            if process.stdin is not None:
                process.stdin.close()
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self._stop_player()
        super().destroy_node()


class CameraViewer:
    def __init__(self, root: tk.Tk, frames: list[LatestFrameBuffer]) -> None:
        self.root = root
        self.log = logging.getLogger("so101.leader.camera_viewer")
        self.closed = False
        self.photos: list[ImageTk.PhotoImage | None] = [None, None, None]
        self.last_render = [0.0, 0.0, 0.0]
        self.fullscreen: tuple[tk.Toplevel, int, tk.Label] | None = None
        self._font_resize_after: str | None = None

        root.title("MARS | Cameras")
        root.geometry("1440x900")
        root.minsize(900, 600)
        root.configure(bg="#101820")
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure("Root.TFrame", background="#101820")
        style.configure("Card.TFrame", background="#182631")
        style.configure("Title.TLabel", background="#101820", foreground="#e5edf2",
                        font=("Sans", 18, "bold"))
        style.configure("Sub.TLabel", background="#101820", foreground="#9eafb9",
                        font=("Sans", 10))
        style.configure("Name.TLabel", background="#182631", foreground="#70c5d9",
                        font=("Sans", 11, "bold"))
        style.configure("Status.TLabel", background="#182631", foreground="#9eafb9",
                        font=("Sans", 9))
        outer = ttk.Frame(root, style="Root.TFrame", padding=18)
        outer.pack(fill="both", expand=True)
        heading = ttk.Frame(outer, style="Root.TFrame")
        heading.pack(fill="x", pady=(0, 14))
        ttk.Label(heading, text="MARS  /  LIVE CAMERAS", style="Title.TLabel").pack(anchor="w")
        ttk.Label(heading, text="Independent video window · double-click any feed for fullscreen · Esc to return",
                  style="Sub.TLabel").pack(anchor="w", pady=(4, 0))

        grid = ttk.Frame(outer, style="Root.TFrame")
        grid.pack(fill="both", expand=True)
        grid.grid_columnconfigure(0, weight=1, uniform="feed")
        grid.grid_columnconfigure(1, weight=1, uniform="feed")
        grid.grid_rowconfigure(0, weight=1, uniform="feedrow")
        grid.grid_rowconfigure(1, weight=1, uniform="feedrow")
        self.canvases: list[tk.Label] = []
        self.statuses: list[ttk.Label] = []
        self.frames = frames
        for index, name in enumerate(CAMERA_NAMES):
            row, col = (0, index) if index < 2 else (1, 0)
            span = 1 if index < 2 else 2
            card = ttk.Frame(grid, style="Card.TFrame", padding=10)
            card.grid(row=row, column=col, columnspan=span, sticky="nsew", padx=6, pady=6)
            ttk.Label(card, text=name, style="Name.TLabel").pack(anchor="w", pady=(0, 7))
            canvas = tk.Label(card, background="#182631", cursor="hand2", height=250,
                              borderwidth=0, anchor="center")
            canvas.pack(fill="both", expand=True)
            status = ttk.Label(card, text="Waiting for feed…", style="Status.TLabel")
            status.pack(fill="x", pady=(7, 0))
            canvas.bind("<Double-Button-1>", lambda _e, i=index: self._open_fullscreen(i))
            self.canvases.append(canvas)
            self.statuses.append(status)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Configure>", self._schedule_responsive_fonts, add="+")
        self._apply_responsive_fonts()
        root.after(50, self._render_latest)

    def _schedule_responsive_fonts(self, event: tk.Event) -> None:
        if event.widget is not self.root:
            return
        if self._font_resize_after is not None:
            try:
                self.root.after_cancel(self._font_resize_after)
            except tk.TclError:
                pass
        self._font_resize_after = self.root.after(100, self._apply_responsive_fonts)

    def _apply_responsive_fonts(self) -> None:
        self._font_resize_after = None
        width = max(1, self.root.winfo_width())
        height = max(1, self.root.winfo_height())
        scale = max(1.4904, min(2.139, min(width / 1440, height / 900) * 1.725))
        style = ttk.Style(self.root)
        default_font = tkfont.nametofont("TkDefaultFont", root=self.root)

        def apply(widget: tk.Misc) -> None:
            try:
                spec = widget.cget("font")
            except tk.TclError:
                spec = ""
            if not spec:
                try:
                    style_name = widget.cget("style") or widget.winfo_class()
                    spec = style.lookup(style_name, "font") or default_font
                except (tk.TclError, TypeError):
                    spec = default_font
            if not hasattr(widget, "_warehouse_bot_font"):
                try:
                    actual = tkfont.Font(root=self.root, font=spec).actual()
                    widget._warehouse_bot_font = (
                        actual["family"], max(8, abs(int(actual["size"]))),
                        actual["weight"], actual["slant"], actual["underline"],
                        actual["overstrike"],
                    )
                    wrap = widget.cget("wraplength")
                    widget._warehouse_bot_wraplength = int(wrap) if str(wrap).isdigit() and int(wrap) > 0 else None
                except (tk.TclError, TypeError, ValueError):
                    widget._warehouse_bot_font = None
                    widget._warehouse_bot_wraplength = None
            base = getattr(widget, "_warehouse_bot_font", None)
            if base:
                family, size, weight, slant, underline, overstrike = base
                try:
                    # Enlarge the actual displayed widget text by 20%, in addition
                    # to adapting the font to the current window dimensions.
                    widget.configure(font=(family, max(10, round(size * scale * 1.2)), weight,
                                           slant, underline, overstrike))
                    wrap = getattr(widget, "_warehouse_bot_wraplength", None)
                    if wrap:
                        widget.configure(wraplength=max(120, round(wrap * scale)))
                except tk.TclError:
                    pass
            for child in widget.winfo_children():
                apply(child)

        apply(self.root)

    def _render_one(self, slot: int) -> None:
        target = self.fullscreen[2] if self.fullscreen and self.fullscreen[1] == slot else self.canvases[slot]
        status = self.statuses[slot]
        try:
            if not target.winfo_exists():
                self.fullscreen = None
                target = self.canvases[slot]
        except tk.TclError:
            self.fullscreen = None
            target = self.canvases[slot]
        item = self.frames[slot].take_latest()
        if item is None:
            return
        message, received_at = item
        try:
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if bgr is None:
                return
            if slot == 1 and self.frames[1].scan_active:
                self._draw_qr_aim_box(bgr)
            width = max(160, target.winfo_width())
            height = max(120, target.winfo_height())
            interpolation = cv2.INTER_AREA if width < bgr.shape[1] or height < bgr.shape[0] else cv2.INTER_LINEAR
            bgr = cv2.resize(bgr, (width, height), interpolation=interpolation)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            image = ImageOps.fit(image, (width, height), method=Image.Resampling.BILINEAR,
                                 centering=(0.5, 0.5))
            photo = ImageTk.PhotoImage(image, master=target)
            target.configure(image=photo, text="")
            target.image = photo
            self.photos[slot] = photo
            self.last_render[slot] = time.monotonic()
            height, width = rgb.shape[:2]
            status.configure(text=f"{width} × {height}  ·  {max(0.0, time.monotonic() - received_at) * 1000:.0f} ms")
        except Exception as error:
            self.log.warning("camera %d frame decode/render failed: %s", slot + 1, error)

    @staticmethod
    def _draw_qr_aim_box(frame: np.ndarray) -> None:
        height, width = frame.shape[:2]
        box_w, box_h = int(width * 0.62), int(height * 0.68)
        x1, y1 = (width - box_w) // 2, (height - box_h) // 2
        x2, y2 = x1 + box_w, y1 + box_h
        color = (60, 245, 245)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        corner = max(16, min(width, height) // 12)
        for x, y, sx, sy in ((x1, y1, 1, 1), (x2, y1, -1, 1),
                             (x1, y2, 1, -1), (x2, y2, -1, -1)):
            cv2.line(frame, (x, y), (x + sx * corner, y), color, 5, cv2.LINE_AA)
            cv2.line(frame, (x, y), (x, y + sy * corner), color, 5, cv2.LINE_AA)
        label = "QR SCAN  ·  align tag inside frame"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.456, min(0.744, width / 900 * 1.2))
        text_size, baseline = cv2.getTextSize(label, font, scale, 1)
        top = max(0, y1 - text_size[1] - baseline - 10)
        cv2.rectangle(frame, (x1, top), (x1 + text_size[0] + 14, y1), (18, 38, 48), -1)
        cv2.putText(frame, label, (x1 + 7, y1 - baseline - 5), font, scale,
                    color, 1, cv2.LINE_AA)

    def _render_latest(self) -> None:
        if self.closed:
            return
        now = time.monotonic()
        for slot in range(3):
            if now - self.last_render[slot] >= 1 / 15:
                self._render_one(slot)
        self.root.after(30, self._render_latest)

    def _open_fullscreen(self, slot: int) -> None:
        if self.fullscreen:
            self.fullscreen[0].destroy()
        window = tk.Toplevel(self.root)
        window.title(f"MARS | {CAMERA_NAMES[slot]}")
        window.configure(bg="#0a1218")
        window.attributes("-fullscreen", True)
        image = tk.Label(window, bg="#101820", borderwidth=0)
        image.pack(fill="both", expand=True)
        window.bind("<Escape>", lambda _e: self._close_fullscreen())
        window.protocol("WM_DELETE_WINDOW", self._close_fullscreen)
        window.bind("<F11>", lambda _e: window.attributes(
            "-fullscreen", not bool(window.attributes("-fullscreen"))))
        image.bind("<Double-Button-1>", lambda _e: self._close_fullscreen())
        self.fullscreen = (window, slot, image)

    def _close_fullscreen(self) -> None:
        fullscreen, self.fullscreen = self.fullscreen, None
        if fullscreen:
            try:
                fullscreen[0].destroy()
            except tk.TclError:
                pass

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        for frame in self.frames:
            frame.close()
        if self.fullscreen:
            self.fullscreen[0].destroy()
        self.root.destroy()


def main(args: list[str] | None = None) -> None:
    _set_parent_death_signal()
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    rclpy.init(args=args)
    frames = [LatestFrameBuffer() for _ in CAMERA_TOPICS]
    cameras = CameraViewerNode(frames)
    executor = SingleThreadedExecutor()
    executor.add_node(cameras)
    spin_thread = threading.Thread(target=executor.spin, name="leader-media-ros", daemon=True)
    try:
        root = tk.Tk()
        CameraViewer(root, frames)
        spin_thread.start()
        root.mainloop()
    finally:
        for frame in frames:
            frame.close()
        executor.shutdown(timeout_sec=1.0)
        executor.remove_node(cameras)
        cameras.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread.is_alive():
            spin_thread.join(timeout=2.0)


def audio_main(args: list[str] | None = None) -> None:
    """Run follower audio playback without opening a second camera window."""
    _set_parent_death_signal()
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    rclpy.init(args=args)
    node = AudioPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
