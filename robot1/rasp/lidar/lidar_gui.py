import time
import math
import os
import sys
import itertools
from collections import deque
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

try:
    from .lidar_logic import *
except ImportError:
    from lidar_logic import *

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

try:
    from lidar_processor import LidarPoint
    from fusion_layer import FusionLayer
    FUSION_AVAILABLE = True
    FUSION_IMPORT_ERROR = ""
except Exception as exc:
    LidarPoint = None
    FusionLayer = None
    FUSION_AVAILABLE = False
    FUSION_IMPORT_ERROR = str(exc)

# ── COULEURS ──────────────────────────────────────────────────────────────────
BG       = '#0a0e1a'
BG2      = '#0f1628'
ACCENT   = '#00e5ff'
ACCENT2  = '#ff3d71'
GRID_COL = '#1a2540'
TEXT_COL = '#8899bb'
BEACON_CONFIRMED_COL = '#00ff88'
BEACON_CANDIDATE_COL = '#ffaa00'


# ── APPLICATION ───────────────────────────────────────────────────────────────
class LidarApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RPLIDAR A2M12 — Radar + Plateau + Pose")
        self.root.configure(bg=BG)
        self.root.geometry("1320x820")
        self.root.minsize(900, 620)

        self.next_track_id = 1
        self.tracks = {}
        self.track_artist_pool = {}
        self.dynamic_map_artists = []
        self.thread_status = "Connexion..."

        # Etat localisation
        # Position initiale connue pour les tests lidar (environ 57 cm, 30 cm)
        self.robot_x = 570.0
        self.robot_y = 300.0
        self.robot_theta = 0.0
        self.pose_confidence = 0.0
        self.pose_beacon_ids = []
        self.pose_localized = False
        self.pose_miss_streak = 0
        self.robot_traj = deque(maxlen=120)
        self.last_pose_update_ts = time.time()
        # Verrou d'initialisation : on refuse toute pose tant que 3 balises
        # ne sont pas confirmees avec un RMS suffisamment bas.
        self.pose_init_locked = True

        # Etat robot adverse
        self.opponent_xy = None
        self.opponent_missed = 0
        self.opponent_history = deque(maxlen=120)

        # Artistes balises candidates (affichage temps reel)
        self.beacon_candidate_artists = []

        self.fusion = None
        self.auto_localization_enabled = AUTO_BEACON_LOCALIZATION
        self.localization_enabled = False
        self.localization_error = ""

        if FUSION_AVAILABLE:
            try:
                self.fusion = FusionLayer(team_color="blue")
                self.fusion.initialize_pose(self.robot_x, self.robot_y, self.robot_theta)

                proc = self.fusion.lidar_processor
                proc.set_sim_mode(False)
                proc._own_beacons = dict(BEACONS_TEST)
                proc._adverse_beacons = {}
                proc._all_beacons = dict(BEACONS_TEST)
                proc.allow_distance_only_fallback = True

                proc._thresholds = {"beacon": 8, "obstacle": 2}
                proc.DISTANCE_MIN_MM = POSE_MIN_DIST_MM
                proc.DISTANCE_MAX_MM = POSE_MAX_DIST_MM
                proc.OBSTACLE_MAX_DIST_MM = POSE_MAX_DIST_MM
                proc.CLUSTER_GAP_MM = int(CLUSTER_GAP_MM)
                proc.CLUSTER_MIN_POINTS = 2
                proc.BEACON_MAX_RADIUS_MM = int(BEACON_HALF_DEPTH_MM + 40.0)
                proc.BEACON_MAX_RESIDUAL_RAD = 0.24
                proc.BEACON_MAX_RESIDUAL_NO_INTENSITY_RAD = 0.36

                self.fusion.MIN_CONFIDENCE_FOR_CORRECTION = 0.12
                self.fusion.ALPHA_CONFIDENCE_1_0 = 0.95
                self.fusion.ALPHA_CONFIDENCE_0_5 = 0.75
                self.localization_enabled = True
            except Exception as exc:
                self.localization_error = str(exc)
        else:
            self.localization_error = FUSION_IMPORT_ERROR

        self._build_ui()
        self._start_lidar()

        if self.localization_error and not self.auto_localization_enabled:
            self._log(f"[WARN] Localisation desactivee: {self.localization_error}\n")
        elif self.localization_error and self.auto_localization_enabled:
            self._log(f"[INFO] Fusion indisponible, mode auto-balises actif: {self.localization_error}\n")

        self.ani = animation.FuncAnimation(
            self.fig, self._update_plot,
            interval=80, blit=False, cache_frame_data=False
        )
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        header = tk.Frame(self.root, bg=BG, pady=10)
        header.pack(fill='x', padx=20)
        tk.Label(header, text="RPLIDAR", bg=BG,
                 fg=ACCENT, font=('Courier New', 22, 'bold')).pack(side='left')
        tk.Label(header, text="  A2M12 — RADAR VIEW", bg=BG,
                 fg=TEXT_COL, font=('Courier New', 14)).pack(side='left', pady=6)
        self.status_var = tk.StringVar(value="Connexion...")
        tk.Label(header, textvariable=self.status_var, bg=BG,
                 fg=ACCENT2, font=('Courier New', 11)).pack(side='right')
        tk.Frame(self.root, bg=GRID_COL, height=1).pack(fill='x', padx=20)

        main = tk.Frame(self.root, bg=BG)
        main.pack(fill='both', expand=True, padx=10, pady=10)

        self.fig = plt.Figure(figsize=(8.4, 7.4), facecolor=BG)
        gs = self.fig.add_gridspec(2, 1, height_ratios=[1.05, 1.0], hspace=0.24)

        self.ax = self.fig.add_subplot(gs[0, 0], projection='polar')
        self._style_polar()
        self.scatter = self.ax.scatter([], [], s=2, c=[], cmap='cool',
                                       vmin=0, vmax=15, alpha=0.85, linewidths=0)
        # Scatter dedie aux candidats-balises sur le radar polaire
        self.scatter_beacons_polar = self.ax.scatter(
            [], [], s=18, c=BEACON_CANDIDATE_COL,
            marker='D', alpha=0.95, zorder=8, linewidths=0
        )

        self.ax_map = self.fig.add_subplot(gs[1, 0])
        self._style_map()
        self.map_points_scatter = self.ax_map.scatter(
            [], [], s=8, c=ACCENT, alpha=0.35, linewidths=0, zorder=2
        )
        self.map_clusters_scatter = self.ax_map.scatter(
            [], [], s=70, c=[], alpha=0.95, edgecolors='white', linewidths=0.6, zorder=5
        )
        # Scatter dedie aux candidats-balises sur la carte
        self.map_beacons_scatter = self.ax_map.scatter(
            [], [], s=90, c=BEACON_CANDIDATE_COL,
            marker='D', edgecolors='white', linewidths=0.7,
            alpha=0.9, zorder=9
        )

        self.canvas = FigureCanvasTkAgg(self.fig, master=main)
        self.canvas.get_tk_widget().configure(bg=BG, highlightthickness=0)
        self.canvas.get_tk_widget().pack(side='left', fill='both', expand=True)

        # Colonne droite
        right = tk.Frame(main, bg=BG, width=240)
        right.pack(side='right', fill='y', padx=(10, 0))
        right.pack_propagate(False)

        panel = tk.Frame(right, bg=BG2, padx=15, pady=15)
        panel.pack(fill='x')

        self._stat_label(panel, "Points / scan")
        self.lbl_points = self._stat_value(panel, "—")
        self._stat_label(panel, "Distance min")
        self.lbl_dmin = self._stat_value(panel, "—")
        self._stat_label(panel, "Distance max")
        self.lbl_dmax = self._stat_value(panel, "—")
        self._stat_label(panel, "Distance moy.")
        self.lbl_dmoy = self._stat_value(panel, "—")

        self._stat_label(panel, "Objets groupés")
        self.lbl_clusters = self._stat_value(panel, "0")
        self._stat_label(panel, "Suivis actifs")
        self.lbl_tracks = self._stat_value(panel, "0")

        # ── SECTION BALISES ──────────────────────────────────────────────────
        tk.Frame(panel, bg=GRID_COL, height=1).pack(fill='x', pady=12)
        self._stat_label(panel, "Balises détectées")
        self.lbl_beacons_raw = self._stat_value(panel, "0")
        self.lbl_beacons_raw.config(fg=BEACON_CANDIDATE_COL)
        self._stat_label(panel, "Balises confirmées")
        self.lbl_beacons_confirmed = self._stat_value(panel, "aucune")
        self.lbl_beacons_confirmed.config(fg=BEACON_CONFIRMED_COL)

        tk.Frame(panel, bg=GRID_COL, height=1).pack(fill='x', pady=12)

        self._stat_label(panel, "Pose X")
        self.lbl_pose_x = self._stat_value(panel, "—")
        self._stat_label(panel, "Pose Y")
        self.lbl_pose_y = self._stat_value(panel, "—")
        self._stat_label(panel, "Cap")
        self.lbl_pose_theta = self._stat_value(panel, "—")
        self._stat_label(panel, "Confiance")
        self.lbl_pose_conf = self._stat_value(panel, "0.00")
        self._stat_label(panel, "Balises utilisées")
        self.lbl_pose_beacons = self._stat_value(panel, "aucune")

        tk.Frame(panel, bg=GRID_COL, height=1).pack(fill='x', pady=12)

        self._stat_label(panel, "Robot adverse X")
        self.lbl_opp_x = self._stat_value(panel, "—")
        self._stat_label(panel, "Robot adverse Y")
        self.lbl_opp_y = self._stat_value(panel, "—")
        self._stat_label(panel, "Robot adverse")
        self.lbl_opp_state = self._stat_value(panel, "aucun")
        self.lbl_opp_x.config(fg='#ffd166')
        self.lbl_opp_y.config(fg='#ffd166')
        self.lbl_opp_state.config(fg='#ffd166')

        tk.Frame(panel, bg=GRID_COL, height=1).pack(fill='x', pady=12)

        self._stat_label(panel, "Portée affichée")
        self.range_var = tk.IntVar(value=MAX_DIST)
        ttk.Scale(panel, from_=500, to=MAX_DIST, orient='horizontal',
                  variable=self.range_var).pack(fill='x', pady=(4, 0))
        self.lbl_range = tk.Label(panel, text=f"{MAX_DIST} mm",
                                  bg=BG2, fg=ACCENT, font=('Courier New', 10))
        self.lbl_range.pack(pady=(2, 10))

        self._stat_label(panel, "Qualité min")
        self.qual_var = tk.IntVar(value=MIN_QUAL)
        ttk.Scale(panel, from_=0, to=15, orient='horizontal',
                  variable=self.qual_var).pack(fill='x', pady=(4, 0))
        self.lbl_qual = tk.Label(panel, text=f"≥ {MIN_QUAL}",
                                 bg=BG2, fg=ACCENT, font=('Courier New', 10))
        self.lbl_qual.pack(pady=(2, 6))

        tk.Frame(panel, bg=GRID_COL, height=1).pack(fill='x', pady=8)

        tk.Label(panel, text="Qualité signal", bg=BG2,
                 fg=TEXT_COL, font=('Courier New', 9)).pack(pady=(4, 4))
        cb_fig = plt.Figure(figsize=(1.8, 0.3), facecolor=BG2)
        cb_ax  = cb_fig.add_axes([0.05, 0.4, 0.9, 0.5])
        cb_ax.imshow(np.linspace(0, 15, 256).reshape(1, -1),
                     aspect='auto', cmap='cool', vmin=0, vmax=15)
        cb_ax.set_xticks([0, 128, 255])
        cb_ax.set_xticklabels(['0', '7', '15'], color=TEXT_COL, fontsize=7)
        cb_ax.set_yticks([])
        for sp in cb_ax.spines.values():
            sp.set_color(GRID_COL)
        cb_fig.patch.set_facecolor(BG2)
        cb_cvs = FigureCanvasTkAgg(cb_fig, master=panel)
        cb_cvs.get_tk_widget().configure(bg=BG2, highlightthickness=0)
        cb_cvs.get_tk_widget().pack(fill='x')
        cb_cvs.draw()

        console_frame = tk.Frame(right, bg=BG2, pady=8, padx=8)
        console_frame.pack(fill='both', expand=True, pady=(10, 0))
        tk.Label(console_frame, text="LOG CONSOLE", bg=BG2,
                 fg=TEXT_COL, font=('Courier New', 9, 'bold')).pack(anchor='w')
        self.console = tk.Text(
            console_frame, bg='#060a14', fg='#44ff88',
            font=('Courier New', 8), wrap='none',
            state='disabled', relief='flat', bd=0
        )
        scrollbar = ttk.Scrollbar(console_frame, command=self.console.yview)
        self.console.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        self.console.pack(fill='both', expand=True, pady=(4, 0))

    def _style_polar(self):
        ax = self.ax
        ax.set_facecolor(BG)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_rlim(0, MAX_DIST)
        ax.set_rticks([2000, 4000, 6000, 8000, 10000, 12000])
        ax.set_yticklabels(['2m','4m','6m','8m','10m','12m'],
                           color=TEXT_COL, fontsize=7)
        ax.set_thetagrids(range(0, 360, 30),
                          [f'{a}°' for a in range(0, 360, 30)],
                          color=TEXT_COL, fontsize=7)
        ax.grid(color=GRID_COL, linewidth=0.6, alpha=0.8)
        ax.spines['polar'].set_color(GRID_COL)
        self.ax.plot(0, 0, 'o', color=ACCENT2, markersize=6, zorder=10)

    def _style_map(self):
        ax = self.ax_map
        ax.set_facecolor(BG)
        ax.set_title(
            f"Plateau {MAP_W_MM / 1000.0:.2f} x {MAP_H_MM / 1000.0:.2f} m (suivi + localisation)",
            color=TEXT_COL,
            fontsize=10,
        )
        ax.set_xlim(-MAP_VIEW_MARGIN_MM, MAP_W_MM + MAP_VIEW_MARGIN_MM)
        ax.set_ylim(-MAP_VIEW_MARGIN_MM, MAP_H_MM + MAP_VIEW_MARGIN_MM)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('X (mm)', color=TEXT_COL, fontsize=8)
        ax.set_ylabel('Y (mm)', color=TEXT_COL, fontsize=8)
        ax.tick_params(colors=TEXT_COL, labelsize=8)
        ax.grid(color=GRID_COL, linewidth=0.6, alpha=0.8)
        for sp in ax.spines.values():
            sp.set_color(GRID_COL)

        plateau_border = plt.Rectangle(
            (0.0, 0.0), float(MAP_W_MM), float(MAP_H_MM),
            fill=False, edgecolor='#2f4f7f', linewidth=1.2, zorder=2
        )
        ax.add_patch(plateau_border)

        beacon_w = max(24.0, float(BEACON_HALF_DEPTH_MM) * 2.0)
        beacon_h = beacon_w
        for bid, (bx, by) in BEACONS_TEST.items():
            rect = plt.Rectangle(
                (bx - beacon_w / 2.0, by - beacon_h / 2.0),
                beacon_w, beacon_h,
                facecolor='#00ff88', edgecolor='white',
                linewidth=0.8, alpha=0.30, zorder=7
            )
            ax.add_patch(rect)
            ax.text(bx + beacon_w * 0.18, by + beacon_h * 0.15, bid, color='#00ff88', fontsize=8,
                    fontweight='bold', zorder=7)

        self.robot_marker = ax.scatter([self.robot_x], [self.robot_y], marker='o', s=70,
                                       color=ACCENT2, edgecolors='white', linewidths=0.7, zorder=8)
        self.robot_heading_line, = ax.plot([], [], color=ACCENT2,
                                           linewidth=1.8, alpha=0.9, zorder=8)
        self.robot_traj_line, = ax.plot([], [], color=ACCENT2,
                                        linewidth=1.1, alpha=0.35, zorder=4)
        self.robot_label = ax.text(self.robot_x + 12, self.robot_y + 10, "Robot",
                                   color=ACCENT2, fontsize=8, zorder=8)

        self.opponent_marker = ax.scatter([], [], marker='X', s=110,
                          color='#ffd166', edgecolors='black',
                          linewidths=0.7, zorder=8)
        self.opponent_traj_line, = ax.plot([], [], color='#ffd166',
                           linewidth=1.2, alpha=0.6, zorder=4)
        self.opponent_label = ax.text(0, 0, "Adverse",
                          color='#ffd166', fontsize=8,
                          zorder=8, visible=False)

    def _stat_label(self, parent, text):
        tk.Label(parent, text=text, bg=BG2, fg=TEXT_COL,
                 font=('Courier New', 9)).pack(anchor='w', pady=(8, 0))

    def _stat_value(self, parent, text):
        lbl = tk.Label(parent, text=text, bg=BG2, fg=ACCENT,
                       font=('Courier New', 13, 'bold'))
        lbl.pack(anchor='w')
        return lbl

    # ── CONSOLE ───────────────────────────────────────────────────────────────
    def _log(self, msg):
        def _append():
            self.console.configure(state='normal')
            self.console.insert('end', msg)
            self.console.see('end')
            self.console.configure(state='disabled')
        self.root.after(0, _append)

    def _set_status_from_thread(self, msg):
        def _apply():
            self.thread_status = str(msg)
        self.root.after(0, _apply)

    @staticmethod
    def _normalize_angle(angle):
        return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi

    def _clamp_pose(self, x, y, theta):
        x = max(0.0, min(float(MAP_W_MM), float(x)))
        y = max(0.0, min(float(MAP_H_MM), float(y)))
        return x, y, self._normalize_angle(float(theta))

    def _blend_pose(self, x, y, theta):
        if self.pose_localized:
            self.robot_x = (1.0 - POSE_BLEND_ALPHA_XY) * self.robot_x + POSE_BLEND_ALPHA_XY * x
            self.robot_y = (1.0 - POSE_BLEND_ALPHA_XY) * self.robot_y + POSE_BLEND_ALPHA_XY * y
            dtheta = self._normalize_angle(theta - self.robot_theta)
            self.robot_theta = self._normalize_angle(self.robot_theta + POSE_BLEND_ALPHA_THETA * dtheta)
        else:
            self.robot_x = float(x)
            self.robot_y = float(y)
            self.robot_theta = self._normalize_angle(float(theta))

    def _degrade_pose(self, decay=0.90):
        self.pose_confidence = max(0.0, self.pose_confidence * float(decay))
        self.pose_miss_streak += 1
        if self.pose_confidence <= 0.05 or self.pose_miss_streak >= POSE_MAX_MISSES_BEFORE_UNLOCK:
            self.pose_localized = False

    def _accept_pose(self, x, y, theta, confidence, beacon_ids=None):
        x = float(x)
        y = float(y)
        theta = self._normalize_angle(float(theta))
        conf = float(max(0.0, min(1.0, confidence)))

        if self.pose_localized:
            dist_jump = math.hypot(x - self.robot_x, y - self.robot_y)
            theta_jump = abs(self._normalize_angle(theta - self.robot_theta))
            if dist_jump < POSE_DEADBAND_MM and theta_jump < math.radians(POSE_DEADBAND_DEG):
                self.pose_confidence = 0.90 * self.pose_confidence + 0.10 * conf
                self.pose_localized = self.pose_confidence > 0.05
                self.pose_miss_streak = 0
                if beacon_ids is not None:
                    self.pose_beacon_ids = list(beacon_ids)
                return

        self._blend_pose(x, y, theta)
        if self.pose_localized:
            self.pose_confidence = max(conf, 0.60 * self.pose_confidence)
        else:
            self.pose_confidence = conf
        self.pose_localized = self.pose_confidence > 0.05
        self.pose_miss_streak = 0
        if beacon_ids is not None:
            self.pose_beacon_ids = list(beacon_ids)
        if self.pose_confidence > 0.08:
            self.robot_traj.append((self.robot_x, self.robot_y))

    # ── DETECTION BALISES ─────────────────────────────────────────────────────

    @staticmethod
    def _clean_beacon_cluster(angles, dists, quals, min_intensity=None, drop_edges=None):
        if min_intensity is None:
            min_intensity = BEACON_QUAL_MIN
        if drop_edges is None:
            drop_edges = BEACON_EDGE_DROP_POINTS

        angles = np.asarray(angles, dtype=float)
        dists  = np.asarray(dists, dtype=float)
        quals  = np.asarray(quals, dtype=float)

        mask = quals >= float(min_intensity)
        a_filt = angles[mask]
        d_filt = dists[mask]
        q_filt = quals[mask]

        if len(a_filt) < 3:
            return None, None, None

        order = np.argsort(a_filt)
        a_sorted = a_filt[order]
        d_sorted = d_filt[order]
        q_sorted = q_filt[order]

        drop = max(0, int(drop_edges))
        if drop == 0:
            return a_sorted, d_sorted, q_sorted

        if len(a_sorted) > (2 * drop + 1):
            return (
                a_sorted[drop:-drop],
                d_sorted[drop:-drop],
                q_sorted[drop:-drop],
            )

        mid = len(a_sorted) // 2
        return (
            a_sorted[mid:mid + 1],
            d_sorted[mid:mid + 1],
            q_sorted[mid:mid + 1],
        )

    @staticmethod
    def _fit_square_beacon(xs, ys, lidar_x=0.0, lidar_y=0.0):
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        if len(xs) < 2:
            return None

        mean_x = float(np.mean(xs))
        mean_y = float(np.mean(ys))
        dx = xs - mean_x
        dy = ys - mean_y

        cov = np.array([
            [float(np.mean(dx * dx)), float(np.mean(dx * dy))],
            [float(np.mean(dx * dy)), float(np.mean(dy * dy))],
        ], dtype=float)

        try:
            eigvals, eigvecs = np.linalg.eigh(cov)
        except np.linalg.LinAlgError:
            return None

        principal_idx = int(np.argmax(eigvals))
        dir_vec = eigvecs[:, principal_idx]
        dir_x = float(dir_vec[0])
        dir_y = float(dir_vec[1])
        norm_dir = math.hypot(dir_x, dir_y)
        if norm_dir <= 1e-9:
            return None
        dir_x /= norm_dir
        dir_y /= norm_dir

        projections = (dx * dir_x) + (dy * dir_y)
        if projections.size == 0:
            return None
        seg_len = float(np.max(projections) - np.min(projections))

        norm_x = -dir_y
        norm_y = dir_x
        ray_x = mean_x - float(lidar_x)
        ray_y = mean_y - float(lidar_y)
        if (norm_x * ray_x + norm_y * ray_y) < 0.0:
            norm_x = -norm_x
            norm_y = -norm_y

        center_x = mean_x + (norm_x * BEACON_HALF_DEPTH_MM)
        center_y = mean_y + (norm_y * BEACON_HALF_DEPTH_MM)

        if not np.isfinite([center_x, center_y, seg_len]).all():
            return None

        return float(center_x), float(center_y), float(seg_len)

    @staticmethod
    def _fit_l_beacon(xs, ys, lidar_x=0.0, lidar_y=0.0):
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        n = len(xs)
        if n < 4:
            return None

        dists_to_lidar = np.hypot(xs - float(lidar_x), ys - float(lidar_y))

        if n >= 5:
            kernel = np.ones(3) / 3.0
            smoothed = np.convolve(dists_to_lidar, kernel, mode='same')
            smoothed[0] = dists_to_lidar[0]
            smoothed[-1] = dists_to_lidar[-1]
            corner_idx = int(np.argmin(smoothed))
        else:
            corner_idx = int(np.argmin(dists_to_lidar))

        if corner_idx < 1 or corner_idx > n - 2:
            return None

        arm1_xs = xs[:corner_idx + 1]
        arm1_ys = ys[:corner_idx + 1]
        arm2_xs = xs[corner_idx:]
        arm2_ys = ys[corner_idx:]

        if len(arm1_xs) < 2 or len(arm2_xs) < 2:
            return None

        def _pca_face(ax, ay):
            mx = float(np.mean(ax))
            my = float(np.mean(ay))
            dx = ax - mx
            dy = ay - my
            cov = np.array([
                [float(np.mean(dx * dx)), float(np.mean(dx * dy))],
                [float(np.mean(dx * dy)), float(np.mean(dy * dy))],
            ], dtype=float)
            try:
                eigvals, eigvecs = np.linalg.eigh(cov)
            except np.linalg.LinAlgError:
                return None

            minor_idx = int(np.argmin(eigvals))
            nv = eigvecs[:, minor_idx]
            nx, ny = float(nv[0]), float(nv[1])
            nrm = math.hypot(nx, ny)
            if nrm < 1e-9:
                return None
            nx /= nrm
            ny /= nrm

            ray_x = mx - float(lidar_x)
            ray_y = my - float(lidar_y)
            if nx * ray_x + ny * ray_y < 0.0:
                nx, ny = -nx, -ny

            major_idx = 1 - minor_idx
            mv = eigvecs[:, major_idx]
            proj = dx * float(mv[0]) + dy * float(mv[1])
            seg_len = float(np.max(proj) - np.min(proj))
            return nx, ny, seg_len

        f1 = _pca_face(arm1_xs, arm1_ys)
        f2 = _pca_face(arm2_xs, arm2_ys)
        if f1 is None or f2 is None:
            return None

        n1x, n1y, len1 = f1
        n2x, n2y, len2 = f2

        dot = abs(n1x * n2x + n1y * n2y)
        if dot > 0.45:
            return None

        corner_x = float(xs[corner_idx])
        corner_y = float(ys[corner_idx])
        cx = corner_x + BEACON_HALF_DEPTH_MM * (n1x + n2x)
        cy = corner_y + BEACON_HALF_DEPTH_MM * (n1y + n2y)

        if not np.isfinite([cx, cy, len1, len2]).all():
            return None

        return float(cx), float(cy), float(max(len1, len2))

    # BEACON EXTRACTION MOVED: All beacon candidate extraction is now in PoseEngine (lidar_logic.py)
    # The GUI retrieves pre-filtered candidates via get_latest_beacon_candidates()
    
    def _extract_beacon_candidates(self, angles, dists, quals):
        """DEPRECATED: Use get_latest_beacon_candidates() from lidar_logic instead.
        Kept as stub for compatibility."""
        # All beacon extraction logic has moved to PoseEngine in lidar_logic.py
        return get_latest_beacon_candidates()

    # ── POSE CALCULATION MOVED TO PoseEngine ────────────────────────────────
    # All pose calculation (trilateration, SVD fitting, 2/3-beacon logic,
    # and opponent detection) has been moved to PoseEngine in lidar_logic.py.
    # The GUI now retrieves calculated poses via get_corrected_pose() (SVD-corrected)
    # and get_latest_opponent() instead of computing them directly.

    # ── POSE UPDATE ───────────────────────────────────────────────────────────

    def _update_pose_from_scan(self, angles, dists, quals):
        """
        Retrieve corrected pose from lidar_logic.py via SVD Umeyama.
        The mathematical work is done in lidar_logic._compute_corrected_pose.
        This GUI method just displays the calculated pose.
        """
        latest_pose = get_corrected_pose()
        
        if latest_pose and latest_pose.is_localized and latest_pose.confidence >= AUTO_POSE_MIN_CONFIDENCE:
            # Accept the corrected pose from SVD
            self._accept_pose(
                latest_pose.x,
                latest_pose.y,
                latest_pose.theta,
                latest_pose.confidence,
                latest_pose.beacon_ids or []
            )
            self.pose_beacon_ids = latest_pose.beacon_ids or []
        else:
            # Degrade pose if no good localization
            self._degrade_pose(decay=0.92)
            self.pose_beacon_ids = []

    # ── AFFICHAGE POSE ────────────────────────────────────────────────────────

    def _update_pose_artists(self):
        self.robot_marker.set_offsets(np.array([[self.robot_x, self.robot_y]]))

        head_len = 110.0
        hx = self.robot_x + head_len * math.sin(self.robot_theta)
        hy = self.robot_y + head_len * math.cos(self.robot_theta)
        self.robot_heading_line.set_data([self.robot_x, hx], [self.robot_y, hy])
        self.robot_label.set_position((self.robot_x + 12, self.robot_y + 10))

        if self.robot_traj:
            tx = [p[0] for p in self.robot_traj]
            ty = [p[1] for p in self.robot_traj]
            self.robot_traj_line.set_data(tx, ty)
        else:
            self.robot_traj_line.set_data([], [])

        # Robot adverse
        if self.opponent_xy is not None:
            ox, oy = self.opponent_xy
            self.opponent_marker.set_offsets(np.array([[ox, oy]]))
            self.opponent_label.set_position((ox + 12, oy + 10))
            self.opponent_label.set_visible(True)
            if len(self.opponent_history) >= 2:
                hx_opp = [p[0] for p in self.opponent_history]
                hy_opp = [p[1] for p in self.opponent_history]
                self.opponent_traj_line.set_data(hx_opp, hy_opp)
            else:
                self.opponent_traj_line.set_data([], [])
        else:
            self.opponent_marker.set_offsets(np.empty((0, 2)))
            self.opponent_label.set_visible(False)
            self.opponent_traj_line.set_data([], [])

    def _refresh_pose_stats(self):
        cap_deg = (math.degrees(self.robot_theta) + 360.0) % 360.0
        self.lbl_pose_x.config(text=f"{self.robot_x:.0f} mm")
        self.lbl_pose_y.config(text=f"{self.robot_y:.0f} mm")
        self.lbl_pose_theta.config(text=f"{cap_deg:.1f} deg")
        self.lbl_pose_conf.config(text=f"{self.pose_confidence:.2f}")
        conf_color = (
            '#00ff88' if self.pose_confidence >= 0.75
            else '#ffaa00' if self.pose_confidence >= 0.35
            else ACCENT2
        )
        self.lbl_pose_conf.config(fg=conf_color)
        ids = " ".join(self.pose_beacon_ids) if self.pose_beacon_ids else "aucune"
        self.lbl_pose_beacons.config(text=ids)

        # Adversaire
        if self.opponent_xy is not None:
            ox, oy = self.opponent_xy
            self.lbl_opp_x.config(text=f"{ox:.0f} mm")
            self.lbl_opp_y.config(text=f"{oy:.0f} mm")
            self.lbl_opp_state.config(text="détecté")
        else:
            self.lbl_opp_x.config(text="—")
            self.lbl_opp_y.config(text="—")
            self.lbl_opp_state.config(text="aucun")

    # ── PLATEAU : PROJECTION, CLUSTERING, SUIVI ────────────────────────────────

    def _project_to_plateau(self, angles, dists):
        world_angles = self.robot_theta + angles
        xs = self.robot_x + dists * np.sin(world_angles)
        ys = self.robot_y + dists * np.cos(world_angles)
        margin = max(float(MAP_VIEW_MARGIN_MM), float(OFFSET_BALISE_MM) + 20.0)
        mask = (
            (xs >= -margin) & (xs <= MAP_W_MM + margin)
            & (ys >= -margin) & (ys <= MAP_H_MM + margin)
        )
        return xs[mask], ys[mask], angles[mask]

    def _cluster_plateau_points(self, xs, ys, angles):
        if len(xs) == 0:
            return []

        order = np.argsort(angles)
        sx = xs[order]
        sy = ys[order]

        gaps = np.hypot(np.diff(sx), np.diff(sy))
        break_points = np.where(gaps > CLUSTER_GAP_MM)[0] + 1
        clusters_raw = list(np.split(np.column_stack((sx, sy)), break_points))

        if len(clusters_raw) >= 2:
            first = clusters_raw[0]
            last  = clusters_raw[-1]
            if len(first) > 0 and len(last) > 0:
                wrap_gap = math.hypot(first[0, 0] - last[-1, 0], first[0, 1] - last[-1, 1])
                if wrap_gap <= CLUSTER_GAP_MM:
                    clusters_raw[0] = np.vstack((last, first))
                    clusters_raw.pop()

        clusters = []
        for pts in clusters_raw:
            if len(pts) < CLUSTER_MIN_POINTS:
                continue

            cx     = float(np.mean(pts[:, 0]))
            cy     = float(np.mean(pts[:, 1]))
            radius = float(np.max(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)))
            clusters.append({
                "center": (cx, cy),
                "radius": radius,
                "count":  int(len(pts)),
            })
        return clusters

    def _update_tracks(self, clusters):
        unmatched_tracks = set(self.tracks.keys())

        for cluster in clusters:
            cx, cy = cluster["center"]
            best_id   = None
            best_dist = TRACK_MATCH_MM

            for tid in list(unmatched_tracks):
                tr = self.tracks[tid]
                d = math.hypot(cx - tr["x"], cy - tr["y"])
                if d <= best_dist:
                    best_dist = d
                    best_id   = tid

            if best_id is None:
                tid = self.next_track_id
                self.next_track_id += 1
                self.tracks[tid] = {
                    "x": cx, "y": cy, "missed": 0,
                    "history": deque([(cx, cy)], maxlen=TRACK_HISTORY_LEN),
                }
                cluster["track_id"] = tid
            else:
                tr = self.tracks[best_id]
                tr["x"] = cx
                tr["y"] = cy
                tr["missed"] = 0
                tr["history"].append((cx, cy))
                cluster["track_id"] = best_id
                unmatched_tracks.discard(best_id)

        for tid in list(unmatched_tracks):
            self.tracks[tid]["missed"] += 1
            if self.tracks[tid]["missed"] > TRACK_MAX_MISSED:
                del self.tracks[tid]

    def _track_color(self, track_id):
        return plt.cm.tab10((track_id - 1) % 10)

    def _draw_map_tracks(self, clusters):
        active_cluster_ids = set()

        for cluster in clusters:
            tid = int(cluster.get("track_id", 0))
            if tid <= 0:
                continue

            active_cluster_ids.add(tid)
            cx, cy = cluster["center"]
            rad    = max(35.0, cluster["radius"])
            color  = self._track_color(tid)

            artists = self.track_artist_pool.get(tid)
            if artists is None:
                circ = plt.Circle(
                    (cx, cy), rad, fill=False,
                    edgecolor=color, linewidth=1.3,
                    alpha=0.8, zorder=4
                )
                txt = self.ax_map.text(
                    cx, cy, f"T{tid}", color='white', fontsize=7,
                    ha='center', va='center', zorder=6
                )
                line, = self.ax_map.plot(
                    [], [], color=color,
                    linewidth=1.2, alpha=0.55, zorder=3
                )
                self.ax_map.add_patch(circ)
                self.track_artist_pool[tid] = {
                    "circle": circ,
                    "text":   txt,
                    "history": line,
                }
                artists = self.track_artist_pool[tid]

            circ = artists["circle"]
            txt  = artists["text"]
            circ.set_center((cx, cy))
            circ.set_radius(rad)
            circ.set_edgecolor(color)
            circ.set_visible(True)
            txt.set_position((cx, cy))
            txt.set_text(f"T{tid}")
            txt.set_visible(True)

        live_track_ids = set(self.tracks.keys())

        for tid in live_track_ids:
            artists = self.track_artist_pool.get(tid)
            if artists is None:
                color = self._track_color(tid)
                circ = plt.Circle(
                    (0.0, 0.0), 1.0, fill=False,
                    edgecolor=color, linewidth=1.3,
                    alpha=0.8, zorder=4, visible=False
                )
                txt = self.ax_map.text(
                    0.0, 0.0, f"T{tid}", color='white', fontsize=7,
                    ha='center', va='center', zorder=6, visible=False
                )
                line, = self.ax_map.plot(
                    [], [], color=color,
                    linewidth=1.2, alpha=0.55, zorder=3
                )
                self.ax_map.add_patch(circ)
                artists = {"circle": circ, "text": txt, "history": line}
                self.track_artist_pool[tid] = artists

            hist = list(self.tracks[tid]["history"])
            line = artists["history"]
            if len(hist) >= 2:
                hx = [p[0] for p in hist]
                hy = [p[1] for p in hist]
                line.set_data(hx, hy)
                line.set_color(self._track_color(tid))
                line.set_visible(True)
            else:
                line.set_data([], [])
                line.set_visible(False)

            is_active = tid in active_cluster_ids
            artists["circle"].set_visible(is_active)
            artists["text"].set_visible(is_active)

        stale_ids = [tid for tid in self.track_artist_pool if tid not in live_track_ids]
        for tid in stale_ids:
            artists = self.track_artist_pool.pop(tid)
            for artist in (artists["circle"], artists["text"], artists["history"]):
                try:
                    artist.remove()
                except Exception:
                    pass

    # ── AFFICHAGE CANDIDATS BALISES ───────────────────────────────────────────

    def _draw_beacon_candidates_on_map(self, beacon_cands):
        """
        Affiche les candidats-balises pre-filtres sur la carte du plateau.
        Les balises sont projetees depuis le repere lidar vers le repere monde
        en utilisant la pose courante du robot.
        """
        if not beacon_cands or not self.pose_localized:
            self.map_beacons_scatter.set_offsets(np.empty((0, 2)))
            self.scatter_beacons_polar.set_offsets(np.empty((0, 2)))
            return

        # Projection monde
        world_pts = []
        polar_pts = []

        for bc in beacon_cands:
            # Repere lidar → repere monde via rotation + translation
            xr = float(bc["x_r"])
            yr = float(bc["y_r"])
            ct = math.cos(self.robot_theta)
            st = math.sin(self.robot_theta)
            wx = self.robot_x + ct * xr - st * yr  # NB: selon convention axe
            wy = self.robot_y + st * xr + ct * yr
            world_pts.append([wx, wy])

            # Polar (angle, distance) pour le radar
            polar_pts.append([float(bc["angle"]), float(bc["distance"])])

        world_arr = np.array(world_pts, dtype=float)
        polar_arr = np.array(polar_pts, dtype=float)

        self.map_beacons_scatter.set_offsets(world_arr)
        self.scatter_beacons_polar.set_offsets(polar_arr)

        # Mise a jour compteur
        self.lbl_beacons_raw.config(text=str(len(beacon_cands)))

    # ── UPDATE PRINCIPAL ──────────────────────────────────────────────────────

    def _update_plot(self, frame):
        if self.auto_localization_enabled:
            loc_txt = "LOC" if self.pose_localized else "NO-LOC"
            self.status_var.set(
                f"{self.thread_status} | auto-balises {loc_txt} {self.pose_confidence:.2f}"
            )
        elif self.localization_enabled:
            loc_txt = "LOC" if self.pose_localized else "NO-LOC"
            self.status_var.set(f"{self.thread_status} | {loc_txt} {self.pose_confidence:.2f}")
        else:
            self.status_var.set(f"{self.thread_status} | localisation off")

        max_dist = self.range_var.get()
        min_qual = self.qual_var.get()
        self.lbl_range.config(text=f"{max_dist} mm")
        self.lbl_qual.config(text=f"≥ {min_qual}")

        # ── Recuperation des donnees ───────────────────────────────────────────
        data          = get_latest_scan_data()
        beacon_cands  = get_latest_beacon_candidates()  # pre-filtres par lidar_runtime

        if not data:
            self._update_pose_artists()
            self._refresh_pose_stats()
            return

        # ── Filtre UI (portee + qualite mini choisie par slider) ───────────────
        filtered = [(a, d, q) for a, d, q in data if q >= min_qual and d <= max_dist]

        if not filtered:
            self.scatter.set_offsets(np.empty((0, 2)))
            self.scatter.set_array(np.array([]))
            self.map_points_scatter.set_offsets(np.empty((0, 2)))
            self.map_clusters_scatter.set_offsets(np.empty((0, 2)))
            self.map_clusters_scatter.set_sizes(np.array([]))
            self.map_clusters_scatter.set_facecolor([])
            self.map_beacons_scatter.set_offsets(np.empty((0, 2)))
            self.scatter_beacons_polar.set_offsets(np.empty((0, 2)))
            self._update_tracks([])
            self._draw_map_tracks([])
            self.lbl_points.config(text="0")
            self.lbl_dmin.config(text="—")
            self.lbl_dmax.config(text="—")
            self.lbl_dmoy.config(text="—")
            self.lbl_clusters.config(text="0")
            self.lbl_tracks.config(text=str(len(self.tracks)))
            self.lbl_beacons_raw.config(text="0")
            self._update_pose_artists()
            self._refresh_pose_stats()
            return

        angles = np.array([p[0] for p in filtered])
        dists  = np.array([p[1] for p in filtered])
        quals  = np.array([p[2] for p in filtered])

        # ── Radar polaire ──────────────────────────────────────────────────────
        self.scatter.set_offsets(np.column_stack([angles, dists]))
        self.scatter.set_array(quals)
        self.scatter.set_clim(0, 15)
        self.ax.set_rlim(0, max_dist)

        self.lbl_points.config(text=str(len(filtered)))
        self.lbl_dmin.config(text=f"{dists.min():.0f} mm")
        self.lbl_dmax.config(text=f"{dists.max():.0f} mm")
        self.lbl_dmoy.config(text=f"{dists.mean():.0f} mm")

        # ── Localisation ───────────────────────────────────────────────────────
        pose_mask = (dists >= POSE_MIN_DIST_MM) & (dists <= POSE_MAX_DIST_MM)
        if int(np.count_nonzero(pose_mask)) >= 3:
            self._update_pose_from_scan(angles[pose_mask], dists[pose_mask], quals[pose_mask])
        else:
            self._update_pose_from_scan(angles, dists, quals)

        # ── Affichage candidats balises ────────────────────────────────────────
        self._draw_beacon_candidates_on_map(beacon_cands)
        confirmed = " ".join(self.pose_beacon_ids) if self.pose_beacon_ids else "aucune"
        self.lbl_beacons_confirmed.config(text=confirmed)

        # ── Projection et clustering sur la carte ──────────────────────────────
        map_x, map_y, map_a = self._project_to_plateau(angles, dists)
        if len(map_x):
            self.map_points_scatter.set_offsets(np.column_stack([map_x, map_y]))
        else:
            self.map_points_scatter.set_offsets(np.empty((0, 2)))

        clusters = self._cluster_plateau_points(map_x, map_y, map_a)
        self._update_tracks(clusters)
        self._draw_map_tracks(clusters)

        # ── Detection robot adverse ────────────────────────────────────────────
        self._detect_opponent(clusters)

        if clusters:
            centers = np.array([c["center"] for c in clusters], dtype=float)
            sizes   = np.array([max(70, min(260, c["count"] * 8)) for c in clusters], dtype=float)
            colors  = [self._track_color(c.get("track_id", 1)) for c in clusters]
            self.map_clusters_scatter.set_offsets(centers)
            self.map_clusters_scatter.set_sizes(sizes)
            self.map_clusters_scatter.set_facecolor(colors)
        else:
            self.map_clusters_scatter.set_offsets(np.empty((0, 2)))
            self.map_clusters_scatter.set_sizes(np.array([]))
            self.map_clusters_scatter.set_facecolor([])

        self.lbl_clusters.config(text=str(len(clusters)))
        self.lbl_tracks.config(text=str(len(self.tracks)))
        self._update_pose_artists()
        self._refresh_pose_stats()

    # ── LIDAR ─────────────────────────────────────────────────────────────────

    def _start_lidar(self):
        self._lidar_thread = start_lidar_thread(self._log, self._set_status_from_thread)

    def _on_close(self):
        stop_lidar_runtime()
        time.sleep(0.3)
        self.root.destroy()


# ── MAIN ──────────────────────────────────────────────────────────────────────
def run_gui():
    root = tk.Tk()
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('Horizontal.TScale',
                    background=BG2, troughcolor=GRID_COL,
                    sliderlength=14, sliderrelief='flat')
    app = LidarApp(root)
    root.mainloop()
    return app


if __name__ == '__main__':
    run_gui()