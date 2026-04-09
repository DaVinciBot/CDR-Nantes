from __future__ import annotations
 
import asyncio
import base64
import importlib
import inspect
import json
import logging
import math
import struct
import threading
import time
from pathlib import Path
from typing import Any, Optional
 
logger = logging.getLogger("foxglove_bridge")
 
# ─────────────────────────────────────────────
# Import optionnel : si non installé, le bridge
# se désactive silencieusement
# ─────────────────────────────────────────────
try:
    from foxglove_websocket.server import FoxgloveServer
    _FOXGLOVE_AVAILABLE = True
except ImportError:
    _FOXGLOVE_AVAILABLE = False
    logger.warning("foxglove_websocket non installé — visualisation désactivée")
 
 
# ─────────────────────────────────────────────
# Paramètres visuels (modifiables à l'import)
# ─────────────────────────────────────────────
ROBOT_DIAMETER_M = 0.35
ROBOT_HEIGHT_M   = 0.10
ROBOT_COLOR      = {"r": 0.2, "g": 0.6, "b": 1.0, "a": 0.85}
DIRECTION_COLOR  = {"r": 1.0, "g": 0.3, "b": 0.1, "a": 0.9}
FOXGLOVE_PORT    = 8765

# PNG/JPG de map a projeter dans Foxglove 3D (render mode: Plane projection)
MAP_IMAGE_PATH      = Path(__file__).parent / "map_assets" / "eurobot2026" / "textures" / "playmat_2026.jpg"
MAP_IMAGE_WIDTH_PX  = 3000
MAP_IMAGE_HEIGHT_PX = 2000
MAP_FRAME_ID        = "world"
MAP_IMAGE_FRAME_ID  = "map_camera"
MAP_LENGTH_M        = 3.0
MAP_WIDTH_M         = 2.0
MAP_CAMERA_HEIGHT_M = 2.0
 
 
# ─────────────────────────────────────────────
# Utilitaires internes
# ─────────────────────────────────────────────
 
def _theta_to_quat(theta: float) -> dict:
    return {"x": 0.0, "y": 0.0,
            "z": math.sin(theta / 2.0),
            "w": math.cos(theta / 2.0)}
 
def _now_ns() -> int:
    return time.time_ns()
 
async def _send(server: Any, ch: int, payload: bytes) -> None:
    ns = _now_ns()
    n  = len(inspect.signature(server.send_message).parameters)
    r  = server.send_message(ch, ns, ns, payload) if n >= 4 else \
         server.send_message(ch, ns, payload)      if n == 3 else \
         server.send_message(ch, payload)
    if inspect.isawaitable(r):
        await r


def _image_format_from_path(path: Path) -> Optional[str]:
    suffix = path.suffix.lower()
    if suffix in {".jpg", ".jpeg"}:
        return "jpeg"
    if suffix == ".png":
        return "png"
    if suffix == ".webp":
        return "webp"
    if suffix == ".avif":
        return "avif"
    return None


def _image_format_from_bytes(data: bytes) -> Optional[str]:
    if data.startswith(b"\xFF\xD8\xFF"):
        return "jpeg"
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        return "png"
    if data.startswith(b"RIFF") and len(data) >= 12 and data[8:12] == b"WEBP":
        return "webp"
    if len(data) >= 12 and data[4:12] in {b"ftypavif", b"ftypavis"}:
        return "avif"
    return None


def _image_size_from_bytes(data: bytes, image_format: Optional[str]) -> Optional[tuple[int, int]]:
    fmt = image_format or _image_format_from_bytes(data)

    if fmt == "png":
        if len(data) >= 24 and data.startswith(b"\x89PNG\r\n\x1a\n"):
            width = int.from_bytes(data[16:20], "big")
            height = int.from_bytes(data[20:24], "big")
            if width > 0 and height > 0:
                return width, height
        return None

    if fmt == "jpeg":
        if not data.startswith(b"\xFF\xD8"):
            return None

        sof_markers = {
            0xC0, 0xC1, 0xC2, 0xC3,
            0xC5, 0xC6, 0xC7,
            0xC9, 0xCA, 0xCB,
            0xCD, 0xCE, 0xCF,
        }
        i = 2
        data_len = len(data)

        while i + 1 < data_len:
            if data[i] != 0xFF:
                i += 1
                continue

            while i < data_len and data[i] == 0xFF:
                i += 1
            if i >= data_len:
                break

            marker = data[i]
            i += 1

            if marker in {0xD8, 0xD9, 0x01} or 0xD0 <= marker <= 0xD7:
                continue

            if i + 1 >= data_len:
                break

            segment_len = int.from_bytes(data[i:i + 2], "big")
            i += 2
            if segment_len < 2 or i + segment_len - 2 > data_len:
                break

            if marker in sof_markers and segment_len >= 7:
                height = int.from_bytes(data[i + 1:i + 3], "big")
                width = int.from_bytes(data[i + 3:i + 5], "big")
                if width > 0 and height > 0:
                    return width, height
                return None

            i += segment_len - 2

    return None
 
# ─────────────────────────────────────────────
# Builders de messages
# ─────────────────────────────────────────────
 
def _pose_msg(x: float, y: float, theta: float) -> bytes:
    return json.dumps({"x": x, "y": y, "theta": theta}).encode()
 
def _tf_msg(x_m: float, y_m: float, theta: float) -> bytes:
    ns = _now_ns()
    return json.dumps({
        "timestamp": {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000},
        "parent_frame_id": "world",
        "child_frame_id":  "robot",
        "translation": {"x": x_m, "y": y_m, "z": 0.0},
        "rotation": _theta_to_quat(theta),
    }).encode()
 
def _scene_msg(x_m: float, y_m: float, theta: float) -> bytes:
    ns  = _now_ns()
    ts  = {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}
    q   = _theta_to_quat(theta)
    tip_x = x_m + ROBOT_DIAMETER_M * 0.7 * math.cos(theta)
    tip_y = y_m + ROBOT_DIAMETER_M * 0.7 * math.sin(theta)
    entity = {
        "timestamp": ts,
        "frame_id":  "world",
        "id":        "robot_body",
        "lifetime":  {"sec": 0, "nsec": 200_000_000},
        "frame_locked": False,
        "metadata":  [],
        "cylinders": [{
            "pose": {
                "position":    {"x": x_m, "y": y_m, "z": ROBOT_HEIGHT_M / 2},
                "orientation": q,
            },
            "size":  {"x": ROBOT_DIAMETER_M, "y": ROBOT_DIAMETER_M, "z": ROBOT_HEIGHT_M},
            "color": ROBOT_COLOR,
        }],
        "arrows": [{
            "pose": {
                "position":    {"x": x_m, "y": y_m, "z": ROBOT_HEIGHT_M},
                "orientation": q,
            },
            "shaft_length": ROBOT_DIAMETER_M * 0.7,
            "shaft_diameter": 0.02,
            "head_diameter":  0.06,
            "head_length":    0.06,
            "color": DIRECTION_COLOR,
        }],
        "points": [], "texts": [], "lines": [],
        "triangles": [], "spheres": [], "models": [],
    }
    return json.dumps({"deletions": [], "entities": [entity]}).encode()


def _map_image_msg(image_b64: str, image_format: str) -> bytes:
    ns = _now_ns()
    return json.dumps({
        "timestamp": {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000},
        "frame_id": MAP_IMAGE_FRAME_ID,
        "format": image_format,
        "data": image_b64,
    }).encode()


def _map_camera_info_msg(width_px: int, height_px: int) -> bytes:
    ns = _now_ns()
    fx = (width_px * MAP_CAMERA_HEIGHT_M) / MAP_LENGTH_M
    fy = (height_px * MAP_CAMERA_HEIGHT_M) / MAP_WIDTH_M
    cx = width_px / 2.0
    cy = height_px / 2.0

    msg = {
        "timestamp": {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000},
        "frame_id": MAP_IMAGE_FRAME_ID,
        "width": width_px,
        "height": height_px,
        "distortion_model": "plumb_bob",
        "D": [],
        "K": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "P": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }
    return json.dumps(msg).encode()


def _map_camera_tf_msg() -> bytes:
    ns = _now_ns()
    return json.dumps({
        "timestamp": {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000},
        "parent_frame_id": MAP_FRAME_ID,
        "child_frame_id": MAP_IMAGE_FRAME_ID,
        "translation": {"x": 0.0, "y": 0.0, "z": MAP_CAMERA_HEIGHT_M},
        # Rotation de 180 deg autour de X pour une frame optique regardant vers le plan z=0.
        "rotation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
    }).encode()


def _fallback_map_scene_msg() -> bytes:
    """Carte minimale (table) si la map détaillée n'est pas disponible."""
    ns = _now_ns()
    ts = {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}
    table = {
        "timestamp": ts,
        "frame_id": "world",
        "id": "cdr_table",
        "frame_locked": False,
        "metadata": [],
        "cubes": [
            {
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.001},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "size": {"x": 3.0, "y": 2.0, "z": 0.002},
                "color": {"r": 0.16, "g": 0.42, "b": 0.18, "a": 0.22},
            }
        ],
        "cylinders": [],
        "arrows": [],
        "points": [],
        "texts": [],
        "lines": [],
        "triangles": [],
        "spheres": [],
        "models": [],
    }
    return json.dumps({"deletions": [], "entities": [table]}).encode()
 
 
# ─────────────────────────────────────────────
# Classe principale
# ─────────────────────────────────────────────
 
class FoxgloveBridge:
    """
    Brique Foxglove 3D — s'attache à un objet com existant.
 
    Paramètres
    ----------
    com      : instance Com déjà ouverte (Com ou WebotsComBridge)
    port     : port websocket Foxglove (défaut 8765)
    enabled  : False pour désactiver complètement sans changer le code
 
    Exemple
    -------
    bridge = FoxgloveBridge(com)
    bridge.start()
    # ... code principal ...
    bridge.stop()
    """
 
    def __init__(
        self,
        com: Any,
        port: int = FOXGLOVE_PORT,
        enabled: bool = True,
        auto_subscribe: bool = True,
    ):
        self.com      = com
        self.port     = port
        self.enabled  = enabled and _FOXGLOVE_AVAILABLE
        self.auto_subscribe = auto_subscribe
        self._thread: Optional[threading.Thread] = None
        self._loop:   Optional[asyncio.AbstractEventLoop] = None
        self._stop_event: Optional[asyncio.Event] = None
        self._ready   = threading.Event()   # signalé quand le serveur est prêt

    def with_auto_subscribe(self, enabled: bool) -> "FoxgloveBridge":
        """Change le mode d'abonnement automatique au callback odométrie."""
        self.auto_subscribe = enabled
        return self
 
    # ── API publique ──────────────────────────────────────────────
 
    def start(self) -> "FoxgloveBridge":
        """Lance le bridge dans un thread d'arrière-plan. Non bloquant."""
        if not self.enabled:
            logger.info("Foxglove désactivé — visualisation ignorée")
            return self
        self._thread = threading.Thread(
            target=self._run_thread,
            name="FoxgloveBridgeThread",
            daemon=True,          # s'arrête si le programme principal quitte
        )
        self._thread.start()
        # Attendre max 3s que le serveur soit prêt
        if self._ready.wait(timeout=3.0):
            logger.info("Foxglove prêt sur ws://localhost:%d", self.port)
        else:
            logger.warning("Foxglove n'a pas démarré dans les 3s — on continue sans")
        return self
 
    def stop(self) -> None:
        """Arrête le bridge proprement."""
        if self._loop and self._stop_event:
            self._loop.call_soon_threadsafe(self._stop_event.set)
        if self._thread:
            self._thread.join(timeout=2.0)
 
    def publish(self, x: float, y: float, theta: float) -> None:
        """
        Publie manuellement une pose (x, y en mm, theta en rad).
        Utile si vous ne voulez pas passer par le callback automatique.
        """
        if not self.enabled or not self._loop:
            return
        self._schedule_send(x, y, theta)
 
    # ── Interne ──────────────────────────────────────────────────
 
    def _run_thread(self) -> None:
        """Thread dédié avec sa propre event loop asyncio."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
        except Exception as exc:
            logger.error("Foxglove bridge erreur : %s", exc)
        finally:
            self._loop.close()
 
    async def _serve(self) -> None:
        self._stop_event = asyncio.Event()
 
        async with FoxgloveServer("0.0.0.0", self.port, "robot-3d-bridge") as server:
 
            ch_pose  = await self._add_channel(server, "robot/pose",  "robot.Pose2D",            self._pose_schema())
            ch_tf    = await self._add_channel(server, "robot/tf",    "foxglove.FrameTransform",  self._tf_schema())
            ch_scene = await self._add_channel(server, "robot/scene", "foxglove.SceneUpdate",     self._scene_schema())
            ch_map   = await self._add_channel(server, "robot/map_scene", "foxglove.SceneUpdate", self._scene_schema())
            ch_map_img = await self._add_channel(server, "robot/map_image/compressed", "foxglove.CompressedImage", self._compressed_image_schema())
            ch_map_cam = await self._add_channel(server, "robot/map_image/camera_info", "foxglove.CameraCalibration", self._camera_calibration_schema())
 
            self._server   = server
            self._ch_pose  = ch_pose
            self._ch_tf    = ch_tf
            self._ch_scene = ch_scene
            self._ch_map   = ch_map
            self._ch_map_img = ch_map_img
            self._ch_map_cam = ch_map_cam

            map_image_b64: Optional[str] = None
            map_image_format: Optional[str] = None
            map_image_width_px = MAP_IMAGE_WIDTH_PX
            map_image_height_px = MAP_IMAGE_HEIGHT_PX
            if MAP_IMAGE_PATH.exists():
                image_data = MAP_IMAGE_PATH.read_bytes()
                map_image_format = _image_format_from_bytes(image_data) or _image_format_from_path(MAP_IMAGE_PATH)
                if map_image_format is None:
                    logger.warning("Format image map non supporte: %s", MAP_IMAGE_PATH)
                else:
                    image_size = _image_size_from_bytes(image_data, map_image_format)
                    if image_size:
                        map_image_width_px, map_image_height_px = image_size
                    else:
                        logger.warning(
                            "Dimensions image non detectees, fallback calibration: %dx%d",
                            map_image_width_px,
                            map_image_height_px,
                        )

                    map_image_b64 = base64.b64encode(image_data).decode("ascii")
                    logger.info(
                        "Map image chargee: %s (%dx%d, %s)",
                        MAP_IMAGE_PATH,
                        map_image_width_px,
                        map_image_height_px,
                        map_image_format,
                    )
            else:
                logger.warning("Image map introuvable: %s", MAP_IMAGE_PATH)

            map_payload = None
            last_map_error: Optional[Exception] = None
            map_scene_modules = (
                "foxglove.foxglove_3d_bridge",
                "robot1.rasp.foxglove.foxglove_3d_bridge",
                "foxglove_3d_bridge",
            )

            for module_name in map_scene_modules:
                try:
                    module = importlib.import_module(module_name)
                    build_map_scene_msg = getattr(module, "build_map_scene_msg")
                    map_payload = build_map_scene_msg()
                    logger.info("Map détaillée chargée via %s", module_name)
                    break
                except Exception as exc:
                    last_map_error = exc

            if map_payload is None:
                logger.warning("Map détaillée indisponible (%s), fallback simple", last_map_error)
                map_payload = _fallback_map_scene_msg()

            async def _map_publisher() -> None:
                tick = 0
                while not self._stop_event.is_set():
                    await _send(self._server, self._ch_map, map_payload)
                    await _send(self._server, self._ch_tf, _map_camera_tf_msg())

                    # Republier periodiquement l'image + calibration pour les reconnexions client.
                    if map_image_b64 and map_image_format and tick % 10 == 0:
                        await _send(self._server, self._ch_map_img, _map_image_msg(map_image_b64, map_image_format))
                        await _send(
                            self._server,
                            self._ch_map_cam,
                            _map_camera_info_msg(map_image_width_px, map_image_height_px),
                        )

                    tick += 1
                    await asyncio.sleep(1.0)

            map_task = asyncio.create_task(_map_publisher())
 
            # Enregistrer le callback sur com
            if self.auto_subscribe:
                try:
                    from loader import loader as _loader

                    _Messages = _loader.load_class("usb_com", "Messages")
                    self.com.add_callback(self._on_odometry, _Messages.UPDATE_ROLLING_BASIS.value)
                except Exception as exc:
                    logger.warning("Impossible d'enregistrer callback odométrie : %s", exc)
 
            self._ready.set()   # signaler que le serveur est opérationnel
            try:
                await self._stop_event.wait()
            finally:
                map_task.cancel()
                await asyncio.gather(map_task, return_exceptions=True)
 
    async def _add_channel(self, server, topic, schema_name, schema):
        ch = server.add_channel({
            "topic":          topic,
            "encoding":       "json",
            "schemaName":     schema_name,
            "schemaEncoding": "jsonschema",
            "schema":         schema,
        })
        if inspect.isawaitable(ch):
            ch = await ch
        return ch
 
    def _on_odometry(self, data: bytes) -> None:
        """Callback appelé depuis le thread série — schedule l'envoi asyncio."""
        if len(data) < 24:
            return
        x, y, theta = struct.unpack("<ddd", data[:24])
        self._schedule_send(x, y, theta)
 
    def _schedule_send(self, x: float, y: float, theta: float) -> None:
        if not self._loop:
            return
        asyncio.run_coroutine_threadsafe(
            self._async_send(x, y, theta),
            self._loop,
        )
 
    async def _async_send(self, x: float, y: float, theta: float) -> None:
        x_m = x / 1000.0
        y_m = y / 1000.0
        try:
            await _send(self._server, self._ch_pose,  _pose_msg(x, y, theta))
            await _send(self._server, self._ch_tf,    _tf_msg(x_m, y_m, theta))
            await _send(self._server, self._ch_scene, _scene_msg(x_m, y_m, theta))
        except Exception as exc:
            logger.debug("Envoi Foxglove échoué (client déconnecté?) : %s", exc)
 
    # ── Schemas ──────────────────────────────────────────────────
 
    @staticmethod
    def _pose_schema() -> str:
        return json.dumps({
            "type": "object",
            "properties": {
                "x":     {"type": "number", "description": "Position X (mm)"},
                "y":     {"type": "number", "description": "Position Y (mm)"},
                "theta": {"type": "number", "description": "Orientation (rad)"},
            },
        })
 
    @staticmethod
    def _tf_schema() -> str:
        return json.dumps({
            "type": "object",
            "title": "foxglove.FrameTransform",
            "properties": {
                "timestamp":       {"type": "object"},
                "parent_frame_id": {"type": "string"},
                "child_frame_id":  {"type": "string"},
                "translation":     {"type": "object"},
                "rotation":        {"type": "object"},
            },
        })
 
    @staticmethod
    def _scene_schema() -> str:
        return json.dumps({
            "type": "object",
            "title": "foxglove.SceneUpdate",
            "properties": {
                "deletions": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "additionalProperties": True,
                    },
                },
                "entities": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "additionalProperties": True,
                    },
                },
            },
            "additionalProperties": True,
        })

    @staticmethod
    def _compressed_image_schema() -> str:
        return json.dumps({
            "type": "object",
            "title": "foxglove.CompressedImage",
            "properties": {
                "timestamp": {"type": "object"},
                "frame_id": {"type": "string"},
                "format": {"type": "string"},
                # In JSON encoding, Foxglove bytes fields must be base64 strings.
                "data": {"type": "string", "contentEncoding": "base64"},
            },
            "required": ["timestamp", "frame_id", "format", "data"],
            "additionalProperties": True,
        })

    @staticmethod
    def _camera_calibration_schema() -> str:
        return json.dumps({
            "type": "object",
            "title": "foxglove.CameraCalibration",
            "properties": {
                "timestamp": {"type": "object"},
                "frame_id": {"type": "string"},
                "width": {"type": "integer"},
                "height": {"type": "integer"},
                "distortion_model": {"type": "string"},
                "D": {"type": "array", "items": {"type": "number"}},
                "K": {"type": "array", "items": {"type": "number"}},
                "R": {"type": "array", "items": {"type": "number"}},
                "P": {"type": "array", "items": {"type": "number"}},
            },
            "required": ["timestamp", "frame_id", "width", "height", "K", "R", "P"],
            "additionalProperties": True,
        })