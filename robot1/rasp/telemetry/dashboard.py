"""Interface locale de supervision, indépendante du matériel du robot."""

import logging
import math
import threading
import tkinter as tk
from collections import deque
from collections.abc import Callable, Mapping
from datetime import datetime
from tkinter import ttk


SnapshotProvider = Callable[[], Mapping[str, object] | None]


class _DashboardLogHandler(logging.Handler):
    """Conserve les derniers messages pour les afficher dans l'interface."""

    def __init__(self, capacity: int = 150) -> None:
        super().__init__(level=logging.INFO)
        self._records: deque[logging.LogRecord] = deque(maxlen=capacity)
        self._records_lock = threading.Lock()

    def emit(self, record: logging.LogRecord) -> None:
        with self._records_lock:
            self._records.append(record)

    def get_records(self) -> list[logging.LogRecord]:
        with self._records_lock:
            return list(self._records)


class RobotDashboard:
    """Affiche l'état d'un robot fourni sous forme de dictionnaire."""

    def __init__(self, title: str = "Supervision robot — CDR Nantes") -> None:
        self.root = tk.Tk()
        self.root.title(title)
        self.root.configure(bg="#101820")
        self.root.minsize(720, 440)
        self.root.geometry("1024x600")
        self.root.attributes("-fullscreen", True)

        self._lock = threading.Lock()
        self._provider: SnapshotProvider | None = None
        self._status = "Démarrage"
        self._elapsed = 0.0
        self._remaining = 100.0
        self._closing = False
        self._last_log_lines: tuple[str, ...] = ()
        self._selected_team = "BLUE"
        self._team_ready = False
        self._team_confirmed = False
        self._team_confirmation = threading.Event()
        self._health_vars: dict[str, tk.StringVar] = {}
        self._health_labels: dict[str, tk.Label] = {}
        self._log_handler = _DashboardLogHandler()
        logging.getLogger().addHandler(self._log_handler)

        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Escape>", lambda _event: self.close())
        self.root.bind("<F11>", self._toggle_fullscreen)
        self._refresh()

    def _build_widgets(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(
            "Dashboard.TFrame",
            background="#101820",
        )
        style.configure(
            "Panel.TFrame",
            background="#192630",
        )
        style.configure(
            "Title.TLabel",
            background="#101820",
            foreground="#f2f5f7",
            font=("TkDefaultFont", 18, "bold"),
        )
        style.configure(
            "Subtitle.TLabel",
            background="#101820",
            foreground="#91a4b2",
            font=("TkDefaultFont", 10),
        )
        style.configure(
            "PanelTitle.TLabel",
            background="#192630",
            foreground="#91a4b2",
            font=("TkDefaultFont", 10, "bold"),
        )
        style.configure(
            "Metric.TLabel",
            background="#192630",
            foreground="#f2f5f7",
            font=("TkDefaultFont", 21, "bold"),
        )
        style.configure(
            "Detail.TLabel",
            background="#192630",
            foreground="#f2f5f7",
            font=("TkDefaultFont", 12),
        )

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=3)
        self.root.rowconfigure(2, weight=2)

        header = ttk.Frame(
            self.root,
            style="Dashboard.TFrame",
            padding=(18, 12),
        )
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)

        ttk.Label(
            header,
            text="CDR NANTES  /  SUPERVISION",
            style="Title.TLabel",
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            header,
            textvariable=self._clock_var(),
            style="Subtitle.TLabel",
        ).grid(row=0, column=1, padx=(12, 0), sticky="e")
        self._status_var = tk.StringVar(value=self._status)
        self._status_label = tk.Label(
            header,
            textvariable=self._status_var,
            bg="#255c48",
            fg="#e8fff3",
            padx=12,
            pady=6,
            font=("TkDefaultFont", 10, "bold"),
        )
        self._status_label.grid(row=0, column=2, padx=(16, 0), sticky="e")

        health_panel = ttk.Frame(
            header,
            style="Panel.TFrame",
            padding=(10, 6),
        )
        health_panel.grid(row=0, column=3, padx=(16, 0), sticky="e")
        ttk.Label(
            health_panel,
            text="DIAGNOSTIC LOGICIEL",
            style="PanelTitle.TLabel",
        ).grid(row=0, column=0, sticky="w")
        for row, component in enumerate(
            ("Teensy USB", "LiDAR", "GPIO", "Carte Teensy"),
            start=1,
        ):
            variable = tk.StringVar(value=f"{component} : non vérifié")
            label = tk.Label(
                health_panel,
                textvariable=variable,
                bg="#192630",
                fg="#91a4b2",
                anchor="w",
                font=("TkDefaultFont", 9),
                width=36,
            )
            label.grid(row=row, column=0, sticky="ew")
            self._health_vars[component] = variable
            self._health_labels[component] = label

        content = ttk.Frame(
            self.root,
            style="Dashboard.TFrame",
            padding=(12, 0, 12, 8),
        )
        content.grid(row=1, column=0, sticky="nsew")
        content.columnconfigure(0, weight=3, uniform="content")
        content.columnconfigure(1, weight=2, uniform="content")
        content.rowconfigure(0, weight=1)

        map_panel = ttk.Frame(content, style="Panel.TFrame", padding=10)
        map_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        map_panel.columnconfigure(0, weight=1)
        map_panel.rowconfigure(1, weight=1)
        ttk.Label(
            map_panel,
            text="POSITION SUR LE TERRAIN  ·  3000 × 2000 mm",
            style="PanelTitle.TLabel",
        ).grid(row=0, column=0, sticky="w", pady=(0, 6))
        self._field = tk.Canvas(
            map_panel,
            bg="#14251f",
            highlightthickness=0,
            height=230,
        )
        self._field.grid(row=1, column=0, sticky="nsew")
        self._field.bind("<Configure>", lambda _event: self._draw_field(None))

        data_panel = ttk.Frame(content, style="Panel.TFrame", padding=12)
        data_panel.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        data_panel.columnconfigure(0, weight=1)
        data_panel.columnconfigure(1, weight=1)
        ttk.Label(
            data_panel,
            text="TÉLÉMÉTRIE",
            style="PanelTitle.TLabel",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))
        self._team_var = tk.StringVar(value="Équipe : —")
        self._mode_var = tk.StringVar(value="Mode : —")
        ttk.Label(
            data_panel,
            textvariable=self._team_var,
            style="Detail.TLabel",
        ).grid(
            row=1, column=0, sticky="w", pady=2
        )
        self._change_team_button = tk.Button(
            data_panel,
            text="Changer d'équipe",
            command=self._toggle_team,
            bg="#294b61",
            fg="#f2f5f7",
            activebackground="#376785",
            activeforeground="#ffffff",
            relief="flat",
            padx=12,
            pady=9,
            font=("TkDefaultFont", 10, "bold"),
            state="disabled",
        )
        self._change_team_button.grid(
            row=1, column=1, sticky="ew", padx=(8, 0), pady=4
        )
        ttk.Label(
            data_panel,
            textvariable=self._mode_var,
            style="Detail.TLabel",
        ).grid(
            row=2, column=0, columnspan=2, sticky="w", pady=2
        )
        self._confirm_team_button = tk.Button(
            data_panel,
            text="Confirmer l'équipe",
            command=self.confirm_team,
            bg="#255c48",
            fg="#e8fff3",
            activebackground="#34785e",
            activeforeground="#ffffff",
            relief="flat",
            padx=12,
            pady=9,
            font=("TkDefaultFont", 10, "bold"),
            state="disabled",
        )
        self._confirm_team_button.grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(8, 4)
        )

        self._x_var = tk.StringVar(value="— mm")
        self._y_var = tk.StringVar(value="— mm")
        self._heading_var = tk.StringVar(value="— °")
        self._metric(data_panel, "X", self._x_var, 4, 0)
        self._metric(data_panel, "Y", self._y_var, 4, 1)
        self._metric(data_panel, "CAP", self._heading_var, 6, 0)

        self._elapsed_var = tk.StringVar(value="Écoulé   00:00")
        self._remaining_var = tk.StringVar(value="Restant   01:40")
        ttk.Label(
            data_panel,
            textvariable=self._elapsed_var,
            style="Detail.TLabel",
        ).grid(
            row=8, column=0, sticky="w", padx=(0, 4), pady=(10, 0)
        )
        ttk.Label(
            data_panel,
            textvariable=self._remaining_var,
            style="Detail.TLabel",
        ).grid(
            row=8, column=1, sticky="w", padx=(4, 0), pady=(10, 0)
        )

        logs_panel = ttk.Frame(
            self.root,
            style="Panel.TFrame",
            padding=(12, 8),
        )
        logs_panel.grid(row=2, column=0, sticky="nsew", padx=12, pady=(0, 12))
        logs_panel.columnconfigure(0, weight=1)
        logs_panel.rowconfigure(1, weight=1)
        ttk.Label(
            logs_panel,
            text="JOURNAL SYSTÈME",
            style="PanelTitle.TLabel",
        ).grid(
            row=0, column=0, sticky="w", pady=(0, 5)
        )
        self._logs = tk.Text(
            logs_panel,
            height=7,
            bg="#101820",
            fg="#d8e1e7",
            insertbackground="#d8e1e7",
            relief="flat",
            wrap="word",
            font=("TkFixedFont", 9),
            state="disabled",
        )
        self._logs.grid(row=1, column=0, sticky="nsew")
        self._logs.tag_configure("WARNING", foreground="#ffc857")
        self._logs.tag_configure("ERROR", foreground="#ff7777")
        self._logs.tag_configure("CRITICAL", foreground="#ff5555")

    def _clock_var(self) -> tk.StringVar:
        self._time_var = tk.StringVar(value="")
        return self._time_var

    @staticmethod
    def _metric(
        parent: ttk.Frame,
        label: str,
        variable: tk.StringVar,
        row: int,
        column: int,
    ) -> None:
        ttk.Label(parent, text=label, style="PanelTitle.TLabel").grid(
            row=row, column=column, sticky="w", padx=(0, 8), pady=(10, 0)
        )
        ttk.Label(parent, textvariable=variable, style="Metric.TLabel").grid(
            row=row + 1, column=column, sticky="w", padx=(0, 8)
        )

    def set_data_provider(self, provider: SnapshotProvider) -> None:
        """Branche une source d'état sans lier l'écran à une classe robot."""
        with self._lock:
            self._provider = provider

    def set_initial_team(self, team: str) -> None:
        """Initialise le choix tactile avec la couleur du switch physique."""
        team = team.upper()
        if team not in ("BLUE", "YELLOW"):
            raise ValueError(f"Équipe inconnue : {team}")
        with self._lock:
            if not self._team_confirmed:
                self._selected_team = team
                self._team_ready = True

    def _toggle_team(self) -> None:
        with self._lock:
            if not self._team_ready or self._team_confirmed:
                return
            self._selected_team = (
                "YELLOW" if self._selected_team == "BLUE" else "BLUE"
            )

    def confirm_team(self) -> None:
        """Fige l'équipe sélectionnée et autorise l'initialisation du robot."""
        with self._lock:
            if not self._team_ready or self._team_confirmed:
                return
            self._team_confirmed = True
            self._team_confirmation.set()

    def wait_for_team_confirmation(
        self,
        stop_event: threading.Event,
    ) -> str | None:
        """Attend la confirmation tactile ou l'annulation de l'application."""
        while not stop_event.is_set():
            if self._team_confirmation.wait(timeout=0.1):
                with self._lock:
                    return self._selected_team
        return None

    def set_status(self, status: str) -> None:
        """Met à jour le statut affiché depuis n'importe quel thread."""
        with self._lock:
            self._status = status

    def set_match_time(self, elapsed: float, remaining: float) -> None:
        """Met à jour les compteurs du match depuis la boucle de contrôle."""
        with self._lock:
            self._elapsed = max(0.0, elapsed)
            self._remaining = max(0.0, remaining)

    def _refresh(self) -> None:
        if self._closing:
            return

        self._time_var.set(datetime.now().strftime("%H:%M:%S"))
        with self._lock:
            status = self._status
            provider = self._provider
            elapsed = self._elapsed
            remaining = self._remaining

        self._status_var.set(status)
        self._status_label.configure(
            bg="#6b3434" if "Erreur" in status else
            "#66521f" if "attente" in status.lower() else "#255c48"
        )
        with self._lock:
            selected_team = self._selected_team
            team_ready = self._team_ready
            team_confirmed = self._team_confirmed
        self._team_var.set(
            f"Équipe : {selected_team if team_ready else 'Lecture du switch…'}"
        )
        self._change_team_button.configure(
            text=(
                f"Changer pour "
                f"{'JAUNE' if selected_team == 'BLUE' else 'BLEU'}"
            ),
            state=(
                "normal"
                if team_ready and not team_confirmed
                else "disabled"
            ),
        )
        self._confirm_team_button.configure(
            text=(
                "Équipe confirmée"
                if team_confirmed
                else "Confirmer l'équipe"
            ),
            state=(
                "normal"
                if team_ready and not team_confirmed
                else "disabled"
            ),
        )
        self._elapsed_var.set(f"Écoulé   {self._format_time(elapsed)}")
        self._remaining_var.set(f"Restant   {self._format_time(remaining)}")

        snapshot = provider() if provider is not None else None
        self._update_telemetry(snapshot)
        self._update_health(snapshot)
        self._draw_field(snapshot)
        self._update_logs()
        self.root.after(100, self._refresh)

    @staticmethod
    def _format_time(seconds: float) -> str:
        total_seconds = max(0, int(seconds))
        minutes, seconds_part = divmod(total_seconds, 60)
        return f"{minutes:02d}:{seconds_part:02d}"

    def _update_telemetry(self, snapshot: Mapping[str, object] | None) -> None:
        if snapshot is None:
            return

        self._team_var.set(f"Équipe : {snapshot.get('team', '—')}")
        self._mode_var.set(f"Mode : {snapshot.get('mode', '—')}")
        x = self._finite_number(snapshot.get("x"))
        y = self._finite_number(snapshot.get("y"))
        theta = self._finite_number(snapshot.get("theta"))
        self._x_var.set(f"{x:.0f} mm" if x is not None else "— mm")
        self._y_var.set(f"{y:.0f} mm" if y is not None else "— mm")
        if theta is None:
            heading = "— °"
        else:
            heading = f"{math.degrees(theta) % 360:.1f} °"
        self._heading_var.set(heading)

    def _update_health(self, snapshot: Mapping[str, object] | None) -> None:
        health = snapshot.get("health") if snapshot is not None else None
        colors = {
            "ok": "#8be0a4",
            "warning": "#ffc857",
            "error": "#ff7777",
            "info": "#8bc9e8",
            "simulated": "#c2a8ff",
            "unknown": "#91a4b2",
        }

        for component, variable in self._health_vars.items():
            result = (
                health.get(component)
                if isinstance(health, Mapping)
                else None
            )
            if isinstance(result, Mapping):
                state = str(result.get("state", "unknown"))
                detail = str(result.get("detail", "non vérifié"))
            else:
                state = "unknown"
                detail = "non vérifié"

            variable.set(f"{component} : {detail}")
            self._health_labels[component].configure(
                fg=colors.get(state, colors["unknown"])
            )

    def _draw_field(self, snapshot: Mapping[str, object] | None) -> None:
        canvas = self._field
        canvas.delete("all")
        width = max(canvas.winfo_width(), 2)
        height = max(canvas.winfo_height(), 2)
        margin = 24
        field_width = width - 2 * margin
        field_height = height - 2 * margin
        if field_width <= 0 or field_height <= 0:
            return

        scale = min(field_width / 3000, field_height / 2000)
        draw_width = 3000 * scale
        draw_height = 2000 * scale
        left = (width - draw_width) / 2
        top = (height - draw_height) / 2
        right = left + draw_width
        bottom = top + draw_height

        canvas.create_rectangle(
            left, top, right, bottom, outline="#73a88a", width=2
        )
        for x_mm in (750, 1500, 2250):
            x_pos = left + x_mm * scale
            canvas.create_line(x_pos, top, x_pos, bottom, fill="#294237")
        for y_mm in (500, 1000, 1500):
            y_pos = bottom - y_mm * scale
            canvas.create_line(left, y_pos, right, y_pos, fill="#294237")
        canvas.create_text(
            left, bottom + 12, text="0,0", fill="#91a4b2", anchor="w"
        )
        canvas.create_text(
            right, bottom + 12, text="3000,2000", fill="#91a4b2", anchor="e"
        )

        if snapshot is None:
            canvas.create_text(
                width / 2,
                height / 2,
                text="En attente de la télémétrie",
                fill="#91a4b2",
            )
            return

        x = self._finite_number(snapshot.get("x"))
        y = self._finite_number(snapshot.get("y"))
        theta = self._finite_number(snapshot.get("theta"))
        if x is None or y is None or theta is None:
            return

        center_x = left + min(max(x, 0), 3000) * scale
        center_y = bottom - min(max(y, 0), 2000) * scale
        radius = max(8, 90 * scale)
        points = []
        for angle, distance in (
            (theta, radius * 1.5),
            (theta + 2.45, radius),
            (theta - 2.45, radius),
        ):
            points.extend(
                (center_x + math.cos(angle) * distance,
                 center_y - math.sin(angle) * distance)
            )
        team = str(snapshot.get("team", "")).upper()
        color = "#ffd166" if team == "YELLOW" else "#55b8ff"
        canvas.create_polygon(points, fill=color, outline="#f2f5f7", width=2)

    def _update_logs(self) -> None:
        records = self._log_handler.get_records()
        lines = tuple(
            f"{datetime.fromtimestamp(record.created):%H:%M:%S} | "
            f"{record.levelname:<8} | "
            f"{record.getMessage()}"
            for record in records
        )
        if lines == self._last_log_lines:
            return

        self._last_log_lines = lines
        self._logs.configure(state="normal")
        self._logs.delete("1.0", "end")
        for record, line in zip(records, lines):
            self._logs.insert("end", line + "\n", record.levelname)
        self._logs.see("end")
        self._logs.configure(state="disabled")

    @staticmethod
    def _finite_number(value: object) -> float | None:
        if isinstance(value, (int, float)) and math.isfinite(value):
            return float(value)
        return None

    def _toggle_fullscreen(self, _event: tk.Event) -> None:
        fullscreen = self.root.attributes("-fullscreen") in (True, 1, "1")
        self.root.attributes("-fullscreen", not fullscreen)

    def close(self) -> None:
        """Détache le collecteur de journaux et ferme la fenêtre."""
        if self._closing:
            return
        self._closing = True
        logging.getLogger().removeHandler(self._log_handler)
        self._log_handler.close()
        self.root.destroy()
