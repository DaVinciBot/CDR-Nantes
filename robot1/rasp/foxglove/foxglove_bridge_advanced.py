#!/usr/bin/env python3
"""
Bridge Foxglove Advanced — Synchronisation de 4 sources de données.

Architecture générale:
  - UPDATE_ROLLING_BASIS (Teensy) → position odométrie (x_odom, y_odom)
  - LIDAR_POSITION         (Lidar) → position calculée (x_lidar, y_lidar)
  - IMU_DATA              (IMU)    → angle orienté θ_imu (+ accel/gyro optionnels)
  - SET_TARGET_POSITION   (Path)   → position cible (x_target, y_target)

Fusion:
  - L'angle θ vient TOUJOURS de l'IMU (plus fiable que l'odométrie)
  - Position fusionnée = fusion(x_odom + x_lidar, y_odom + y_lidar)
  - Visualisation: 1 robot (fused) + 3 marqueurs (target, odom, lidar)

Types de données générales:
  - SensorData: {x, y, theta, timestamp, confidence}
  - Position2D: {x, y} en mm
  - Orientation1D: {theta} en rad (de l'IMU)
"""

from __future__ import annotations

import asyncio
import inspect
import json
import logging
import math
import signal
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger("foxglove_bridge_advanced")

try:
    from foxglove_websocket.server import FoxgloveServer
    _FOXGLOVE_AVAILABLE = True
except ImportError:
    _FOXGLOVE_AVAILABLE = False
    logger.warning("foxglove_websocket non installé")

# ─────────────────────────────────────────────
# Types de données génériques
# ─────────────────────────────────────────────

@dataclass
class Position2D:
    """Position en 2D (mm)."""
    x: float = 0.0
    y: float = 0.0
    
    def to_mm(self) -> tuple[float, float]:
        return (self.x, self.y)
    
    def to_m(self) -> tuple[float, float]:
        return (self.x / 1000.0, self.y / 1000.0)

@dataclass
class Orientation1D:
    """Orientation : angle yaw en radians."""
    theta: float = 0.0  # rad

@dataclass
class SensorData:
    """Données génériques d'un capteur avec métadonnées."""
    position: Position2D = field(default_factory=Position2D)
    theta: float = 0.0  # rad (optionnel, peut être ignoré si source=IMU)
    timestamp: float = 0.0  # ns
    confidence: float = 1.0  # 0.0-1.0
    source_name: str = "unknown"
    
    def __repr__(self) -> str:
        return (f"{self.source_name} @ ({self.position.x:.0f},{self.position.y:.0f}) "
                f"θ={self.theta:.3f} conf={self.confidence:.2f}")

# ─────────────────────────────────────────────
# Paramètres visuels
# ─────────────────────────────────────────────

ROBOT_DIAMETER_M = 0.35
ROBOT_HEIGHT_M = 0.10
ROBOT_COLOR = {"r": 0.2, "g": 0.6, "b": 1.0, "a": 0.85}
DIRECTION_COLOR = {"r": 1.0, "g": 0.3, "b": 0.1, "a": 0.9}

# Couleurs des marqueurs
MARKER_COLORS = {
    "target": {"r": 0, "g": 1, "b": 0, "a": 0.7},  # Vert
    "odom":   {"r": 0, "g": 0, "b": 1, "a": 0.7},  # Bleu
    "lidar":  {"r": 1, "g": 1, "b": 0, "a": 0.7},  # Jaune
}

# ─────────────────────────────────────────────
# Utilitaires
# ─────────────────────────────────────────────

def theta_to_quaternion(theta: float) -> dict:
    return {
        "x": 0.0, "y": 0.0,
        "z": math.sin(theta / 2.0),
        "w": math.cos(theta / 2.0)
    }

def now_ns() -> int:
    return time.time_ns()

async def maybe_await(value: Any) -> Any:
    if inspect.isawaitable(value):
        return await value
    return value

async def send_msg(server: Any, channel_id: int, payload: bytes) -> None:
    """Envoi compatible avec toutes versions de foxglove-websocket."""
    ns = now_ns()
    n = len(inspect.signature(server.send_message).parameters)
    if n >= 4:
        result = server.send_message(channel_id, ns, ns, payload)
    elif n == 3:
        result = server.send_message(channel_id, ns, payload)
    else:
        result = server.send_message(channel_id, payload)
    await maybe_await(result)

# ─────────────────────────────────────────────
# Schemas JSON
# ─────────────────────────────────────────────

POSE_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "x_target": {"type": "number", "description": "Target X (mm)"},
        "y_target": {"type": "number", "description": "Target Y (mm)"},
        "x_odom":   {"type": "number", "description": "Odometry X (mm)"},
        "y_odom":   {"type": "number", "description": "Odometry Y (mm)"},
        "x_lidar":  {"type": "number", "description": "Lidar X (mm)"},
        "y_lidar":  {"type": "number", "description": "Lidar Y (mm)"},
        "x_fused":  {"type": "number", "description": "Fused X (mm)"},
        "y_fused":  {"type": "number", "description": "Fused Y (mm)"},
        "theta_imu": {"type": "number", "description": "IMU angle (rad)"},
    },
})

TF_SCHEMA = json.dumps({
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

SCENE_SCHEMA = json.dumps({
    "type": "object",
    "title": "foxglove.SceneUpdate",
    "properties": {
        "deletions": {"type": "array", "items": {"type": "object"}},
        "entities": {"type": "array", "items": {"type": "object"}},
    },
    "additionalProperties": True,
})

# ─────────────────────────────────────────────
# Builders (modèle générique)
# ─────────────────────────────────────────────

def build_pose_msg(
    x_target: float, y_target: float,
    x_odom: float, y_odom: float,
    x_lidar: float, y_lidar: float,
    x_fused: float, y_fused: float,
    theta_imu: float
) -> bytes:
    """Message pose avec 4 positions + angle IMU."""
    return json.dumps({
        "x_target": x_target, "y_target": y_target,
        "x_odom":   x_odom,   "y_odom":   y_odom,
        "x_lidar":  x_lidar,  "y_lidar":  y_lidar,
        "x_fused":  x_fused,  "y_fused":  y_fused,
        "theta_imu": theta_imu
    }).encode()

def build_tf_msg(x_m: float, y_m: float, theta: float) -> bytes:
    """Transform world → robot (en mètres)."""
    ns = now_ns()
    q = theta_to_quaternion(theta)
    msg = {
        "timestamp": {
            "sec":   ns // 1_000_000_000,
            "nsec":  ns %  1_000_000_000,
        },
        "parent_frame_id": "world",
        "child_frame_id":  "robot",
        "translation": {"x": x_m, "y": y_m, "z": 0.0},
        "rotation": q,
    }
    return json.dumps(msg).encode()

def build_scene_msg(
    x_fused_m: float, y_fused_m: float, theta: float,
    x_target_m: float, y_target_m: float,
    x_odom_m: float, y_odom_m: float,
    x_lidar_m: float, y_lidar_m: float
) -> bytes:
    """Scène 3D : robot (fused) + 3 marqueurs simples."""
    ns = now_ns()
    ts = {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}
    q = theta_to_quaternion(theta)
    
    entities = []
    
    # 1. Robot (fused) avec flèche
    arrow_len = ROBOT_DIAMETER_M * 0.7
    robot_entity = {
        "timestamp": ts,
        "frame_id": "world",
        "id": "robot_body",
        "lifetime": {"sec": 0, "nsec": 200_000_000},
        "frame_locked": False,
        "metadata": [],
        "cylinders": [{
            "pose": {
                "position": {"x": x_fused_m, "y": y_fused_m, "z": ROBOT_HEIGHT_M / 2},
                "orientation": q,
            },
            "size": {"x": ROBOT_DIAMETER_M, "y": ROBOT_DIAMETER_M, "z": ROBOT_HEIGHT_M},
            "color": ROBOT_COLOR,
        }],
        "arrows": [{
            "pose": {
                "position": {"x": x_fused_m, "y": y_fused_m, "z": ROBOT_HEIGHT_M},
                "orientation": q,
            },
            "shaft_length": arrow_len,
            "shaft_diameter": 0.02,
            "head_diameter": 0.06,
            "head_length": 0.06,
            "color": DIRECTION_COLOR,
        }],
        "points": [], "texts": [], "lines": [],
        "triangles": [], "spheres": [], "models": [],
    }
    entities.append(robot_entity)
    
    # 2. Marqueurs simples (target, odom, lidar)
    markers = [
        ("target", x_target_m, y_target_m, MARKER_COLORS["target"]),
        ("odom",   x_odom_m,   y_odom_m,   MARKER_COLORS["odom"]),
        ("lidar",  x_lidar_m,  y_lidar_m,  MARKER_COLORS["lidar"]),
    ]
    
    for marker_id, x, y, color in markers:
        marker_entity = {
            "timestamp": ts,
            "frame_id": "world",
            "id": marker_id,
            "lifetime": {"sec": 0, "nsec": 200_000_000},
            "frame_locked": False,
            "metadata": [],
            "cylinders": [{
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.05},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "size": {"x": 0.08, "y": 0.08, "z": 0.08},
                "color": color
            }],
            "arrows": [], "points": [], "texts": [], "lines": [],
            "triangles": [], "spheres": [], "models": [],
        }
        entities.append(marker_entity)
    
    return json.dumps({"deletions": [], "entities": entities}).encode()

# ─────────────────────────────────────────────
# Gestionnaire d'état thread-safe
# ─────────────────────────────────────────────

class SensorFusionManager:
    """Gère les données de 4 sources avec thread-safety."""
    
    def __init__(self):
        self._lock = threading.RLock()
        self.target = SensorData(source_name="target")
        self.odom = SensorData(source_name="odom")
        self.lidar = SensorData(source_name="lidar")
        self.imu_theta = Orientation1D()  # Source unique pour l'angle
        
    def update_odom(self, x: float, y: float, timestamp: float = 0.0):
        """Reçoit odométrie du Teensy (ignore son angle)."""
        with self._lock:
            self.odom = SensorData(
                position=Position2D(x=x, y=y),
                theta=0.0,  # Ignoré (on utilise IMU)
                timestamp=timestamp or now_ns(),
                source_name="odom"
            )
    
    def update_lidar(self, x: float, y: float, theta: float = 0.0, confidence: float = 1.0):
        """Reçoit position Lidar (calcule par balises)."""
        with self._lock:
            self.lidar = SensorData(
                position=Position2D(x=x, y=y),
                theta=theta,  # Peut être ignoré
                timestamp=now_ns(),
                confidence=confidence,
                source_name="lidar"
            )
    
    def update_imu(self, theta: float):
        """Reçoit l'angle IMU — SEULE source fiable pour θ."""
        with self._lock:
            self.imu_theta = Orientation1D(theta=theta)
    
    def update_target(self, x: float, y: float):
        """Reçoit la position cible du Path Finding."""
        with self._lock:
            self.target = SensorData(
                position=Position2D(x=x, y=y),
                timestamp=now_ns(),
                source_name="target"
            )
    
    def get_snapshot(self) -> Dict[str, SensorData]:
        """Snapshot thread-safe de tous les capteurs."""
        with self._lock:
            return {
                "target": SensorData(
                    position=Position2D(x=self.target.position.x, y=self.target.position.y),
                    theta=0.0,
                    timestamp=self.target.timestamp,
                    source_name=self.target.source_name
                ),
                "odom": SensorData(
                    position=Position2D(x=self.odom.position.x, y=self.odom.position.y),
                    theta=self.imu_theta.theta,  # ← IMU!
                    timestamp=self.odom.timestamp,
                    source_name=self.odom.source_name
                ),
                "lidar": SensorData(
                    position=Position2D(x=self.lidar.position.x, y=self.lidar.position.y),
                    theta=self.imu_theta.theta,  # ← IMU!
                    timestamp=self.lidar.timestamp,
                    confidence=self.lidar.confidence,
                    source_name=self.lidar.source_name
                ),
                "imu": SensorData(
                    theta=self.imu_theta.theta,
                    source_name="imu"
                ),
            }
    
    def fuse_position(self) -> tuple[float, float, float]:
        """
        Fusionne odométrie + Lidar.
        Stratégie simple: moyenne pondérée si Lidar valide (conf > 0.2),
        sinon utilise odométrie brute.
        """
        snap = self.get_snapshot()
        
        odom_data = snap["odom"]
        lidar_data = snap["lidar"]
        imu_data = snap["imu"]
        
        # Si Lidar valide (confiance suffisante et données récentes)
        if lidar_data.confidence > 0.2:
            # Moyenne pondérée: 60% Lidar, 40% Odom
            x_fused = 0.6 * lidar_data.position.x + 0.4 * odom_data.position.x
            y_fused = 0.6 * lidar_data.position.y + 0.4 * odom_data.position.y
        else:
            # Lidar absent → odométrie seule
            x_fused = odom_data.position.x
            y_fused = odom_data.position.y
        
        return (x_fused, y_fused, imu_data.theta)

# ─────────────────────────────────────────────
# Bridge Foxglove
# ─────────────────────────────────────────────

class FoxgloveBridgeAdvanced:
    """Bridge synchronisant 4 sources capteurs → Foxglove."""
    
    def __init__(self, port: int = 8765):
        self.port = port
        self.enabled = _FOXGLOVE_AVAILABLE
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._fusion = SensorFusionManager()
        self._stop_event: Optional[asyncio.Event] = None
        self._ready = threading.Event()
    
    def start(self) -> FoxgloveBridgeAdvanced:
        """Lance le bridge dans un thread arrière-plan."""
        if not self.enabled:
            logger.warning("Foxglove désactivé")
            return self
        self._thread = threading.Thread(
            target=self._run_thread,
            name="FoxgloveBridgeThread",
            daemon=True
        )
        self._thread.start()
        if self._ready.wait(timeout=3.0):
            logger.info(f"Foxglove prêt sur ws://localhost:{self.port}")
        else:
            logger.warning("Foxglove n'a pas démarré à temps")
        return self
    
    def stop(self) -> None:
        """Arrête le bridge proprement."""
        if self._loop and self._stop_event:
            self._loop.call_soon_threadsafe(self._stop_event.set)
        if self._thread:
            self._thread.join(timeout=2.0)
    
    def register_callbacks(self, com: Any) -> None:
        """Enregistre les callbacks sur l'objet com (USB)."""
        try:
            from loader import loader
            Messages = loader.load_class("usb_com", "Messages")
            
            # Callback odométrie Teensy (x, y, theta brut — on ignore theta)
            def on_rolling_basis(data: bytes):
                if len(data) >= 24:
                    x, y, theta = struct.unpack("<ddd", data[:24])
                    self._fusion.update_odom(x, y)
            
            com.add_callback(on_rolling_basis, Messages.UPDATE_ROLLING_BASIS.value)
            logger.info("✓ Callback UPDATE_ROLLING_BASIS enregistré")
        except Exception as exc:
            logger.warning(f"Impossible d'enregistrer callbacks: {exc}")
    
    def publish_data(
        self,
        target_x: float, target_y: float,
        odom_x: float, odom_y: float,
        lidar_x: float = 0.0, lidar_y: float = 0.0,
        lidar_confidence: float = 0.0,
        imu_theta: float = 0.0
    ) -> None:
        """
        Interface publique pour envoyer manuellement les données.
        Utile pour tester ou intégrer des sources personnalisées.
        """
        if not self.enabled or not self._loop:
            return
        
        self._fusion.update_target(target_x, target_y)
        self._fusion.update_odom(odom_x, odom_y)
        self._fusion.update_lidar(lidar_x, lidar_y, confidence=lidar_confidence)
        self._fusion.update_imu(imu_theta)
        
        self._schedule_send()
    
    def _run_thread(self) -> None:
        """Thread dédié avec sa propre event loop."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
        except Exception as exc:
            logger.error(f"Bridge error: {exc}")
        finally:
            self._loop.close()
    
    async def _serve(self) -> None:
        self._stop_event = asyncio.Event()
        
        async with FoxgloveServer("0.0.0.0", self.port, "robot-multimodal-bridge") as server:
            
            ch_pose = await maybe_await(server.add_channel({
                "topic": "robot/pose",
                "encoding": "json",
                "schemaName": "robot.Pose2D",
                "schemaEncoding": "jsonschema",
                "schema": POSE_SCHEMA,
            }))
            
            ch_tf = await maybe_await(server.add_channel({
                "topic": "robot/tf",
                "encoding": "json",
                "schemaName": "foxglove.FrameTransform",
                "schemaEncoding": "jsonschema",
                "schema": TF_SCHEMA,
            }))
            
            ch_scene = await maybe_await(server.add_channel({
                "topic": "robot/scene",
                "encoding": "json",
                "schemaName": "foxglove.SceneUpdate",
                "schemaEncoding": "jsonschema",
                "schema": SCENE_SCHEMA,
            }))
            
            logger.info("Channels créés: robot/pose | robot/tf | robot/scene")
            
            self._ready.set()
            
            async def send_loop() -> None:
                while not self._stop_event.is_set():
                    snap = self._fusion.get_snapshot()
                    x_fused, y_fused, theta_imu = self._fusion.fuse_position()
                    
                    try:
                        # Envoyer pose (mm)
                        await send_msg(server, ch_pose, build_pose_msg(
                            snap["target"].position.x, snap["target"].position.y,
                            snap["odom"].position.x, snap["odom"].position.y,
                            snap["lidar"].position.x, snap["lidar"].position.y,
                            x_fused, y_fused,
                            theta_imu
                        ))
                        
                        # Envoyer transform (m)
                        await send_msg(server, ch_tf, build_tf_msg(
                            x_fused / 1000.0, y_fused / 1000.0, theta_imu
                        ))
                        
                        # Envoyer scène 3D (m)
                        await send_msg(server, ch_scene, build_scene_msg(
                            x_fused / 1000.0, y_fused / 1000.0, theta_imu,
                            snap["target"].position.x / 1000.0,
                            snap["target"].position.y / 1000.0,
                            snap["odom"].position.x / 1000.0,
                            snap["odom"].position.y / 1000.0,
                            snap["lidar"].position.x / 1000.0,
                            snap["lidar"].position.y / 1000.0,
                        ))
                    except Exception as exc:
                        logger.debug(f"Envoi échoué: {exc}")
                    
                    await asyncio.sleep(0.05)  # 20 Hz
            
            send_task = asyncio.create_task(send_loop())
            
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    self._loop.add_signal_handler(sig, self._stop_event.set)
                except NotImplementedError:
                    pass
            
            try:
                await self._stop_event.wait()
            finally:
                send_task.cancel()
                await asyncio.gather(send_task, return_exceptions=True)
                logger.info("Bridge arrêté")
    
    def _schedule_send(self) -> None:
        """Demande un envoi de données."""
        # Les données sont envoyées en boucle dans send_loop()
        pass

if __name__ == "__main__":
    logger.setLevel(logging.DEBUG)
    handler = logging.StreamHandler()
    handler.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    logger.info("Module foxglove_bridge_advanced chargé")
