import logging
import math
import threading
from types import SimpleNamespace

from robot1.rasp.telemetry.dashboard import (
    RobotDashboard,
    _DashboardLogHandler,
)


class _TextValue:
    def __init__(self) -> None:
        self.value = None

    def set(self, value: str) -> None:
        self.value = value


class _Label:
    def __init__(self) -> None:
        self.options = {}

    def configure(self, **options) -> None:
        self.options.update(options)


def test_telemetry_fields_format_pose_and_heading():
    dashboard = RobotDashboard.__new__(RobotDashboard)
    dashboard._team_var = _TextValue()
    dashboard._mode_var = _TextValue()
    dashboard._x_var = _TextValue()
    dashboard._y_var = _TextValue()
    dashboard._heading_var = _TextValue()

    dashboard._update_telemetry(
        {
            "team": "BLUE",
            "mode": "HARDWARE",
            "x": 1250.4,
            "y": 560,
            "theta": math.radians(370),
        }
    )

    assert dashboard._team_var.value == "Équipe : BLUE"
    assert dashboard._mode_var.value == "Mode : HARDWARE"
    assert dashboard._x_var.value == "1250 mm"
    assert dashboard._y_var.value == "560 mm"
    assert dashboard._heading_var.value == "10.0 °"


def test_invalid_pose_values_are_not_displayed_as_numbers():
    dashboard = RobotDashboard.__new__(RobotDashboard)
    dashboard._team_var = _TextValue()
    dashboard._mode_var = _TextValue()
    dashboard._x_var = _TextValue()
    dashboard._y_var = _TextValue()
    dashboard._heading_var = _TextValue()

    dashboard._update_telemetry(
        {"x": float("nan"), "y": float("inf"), "theta": "inconnu"}
    )

    assert dashboard._x_var.value == "— mm"
    assert dashboard._y_var.value == "— mm"
    assert dashboard._heading_var.value == "— °"


def test_health_panel_displays_status_and_marks_unverified_components():
    dashboard = RobotDashboard.__new__(RobotDashboard)
    dashboard._health_vars = {
        "Teensy USB": _TextValue(),
        "LiDAR": _TextValue(),
        "GPIO": _TextValue(),
        "Carte Teensy": _TextValue(),
    }
    dashboard._health_labels = {
        name: _Label() for name in dashboard._health_vars
    }

    dashboard._update_health(
        {
            "health": {
                "Teensy USB": {
                    "state": "ok",
                    "detail": "odométrie récente",
                },
                "LiDAR": {"state": "error", "detail": "absent"},
                "GPIO": {"state": "info", "detail": "entrée lue"},
            }
        }
    )

    assert dashboard._health_vars["Teensy USB"].value == (
        "Teensy USB : odométrie récente"
    )
    assert dashboard._health_vars["LiDAR"].value == "LiDAR : absent"
    assert dashboard._health_vars["Carte Teensy"].value == (
        "Carte Teensy : non vérifié"
    )
    assert dashboard._health_labels["LiDAR"].options["fg"] == "#ff7777"


def test_robot_snapshot_reports_only_observable_hardware_health(monkeypatch):
    from robot1.rasp import app

    robot = app.Robot.__new__(app.Robot)
    robot.lock = threading.Lock()
    robot.x = 100.0
    robot.y = 200.0
    robot.theta = 0.5
    robot.couleur = "BLUE"
    robot.mode = "HARDWARE"
    robot._last_odometry_at = 9.5
    robot._pin_tirette = SimpleNamespace(is_pressed=True)
    monkeypatch.setattr(app.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(app.lidar, "get_status", lambda: (True, ""))

    health = robot.dashboard_snapshot()["health"]

    assert health["Teensy USB"]["state"] == "ok"
    assert health["LiDAR"]["state"] == "ok"
    assert health["GPIO"]["state"] == "info"
    assert health["Carte Teensy"]["state"] == "unknown"
    assert "tirette insérée" in health["GPIO"]["detail"]


def test_robot_snapshot_marks_stale_teensy_odometry_as_error(monkeypatch):
    from robot1.rasp import app

    robot = app.Robot.__new__(app.Robot)
    robot.lock = threading.Lock()
    robot.x = 100.0
    robot.y = 200.0
    robot.theta = 0.5
    robot.couleur = "YELLOW"
    robot.mode = "HARDWARE"
    robot._last_odometry_at = 8.0
    robot._pin_tirette = SimpleNamespace(is_pressed=False)
    monkeypatch.setattr(app.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(app.lidar, "get_status", lambda: (False, ""))

    health = robot.dashboard_snapshot()["health"]

    assert health["Teensy USB"]["state"] == "error"
    assert health["LiDAR"]["state"] == "warning"


def test_team_can_be_changed_and_confirmed_only_before_initialization():
    dashboard = RobotDashboard.__new__(RobotDashboard)
    dashboard._lock = threading.Lock()
    dashboard._selected_team = "BLUE"
    dashboard._team_ready = False
    dashboard._team_confirmed = False
    dashboard._team_confirmation = threading.Event()

    dashboard.set_initial_team("YELLOW")
    dashboard._toggle_team()
    dashboard.confirm_team()

    assert dashboard.wait_for_team_confirmation(threading.Event()) == "BLUE"
    dashboard._toggle_team()
    dashboard.set_initial_team("BLUE")
    assert dashboard.wait_for_team_confirmation(threading.Event()) == "BLUE"


def test_team_selection_wait_can_be_cancelled():
    dashboard = RobotDashboard.__new__(RobotDashboard)
    dashboard._lock = threading.Lock()
    dashboard._selected_team = "BLUE"
    dashboard._team_ready = True
    dashboard._team_confirmed = False
    dashboard._team_confirmation = threading.Event()
    stop_event = threading.Event()
    stop_event.set()

    assert dashboard.wait_for_team_confirmation(stop_event) is None


def test_dashboard_log_handler_keeps_only_recent_records():
    handler = _DashboardLogHandler(capacity=2)
    logger = logging.getLogger("test_dashboard_handler")

    for message in ("premier", "deuxième", "troisième"):
        handler.emit(
            logger.makeRecord(
                logger.name,
                logging.INFO,
                __file__,
                1,
                message,
                (),
                None,
            )
        )

    assert [record.getMessage() for record in handler.get_records()] == [
        "deuxième",
        "troisième",
    ]
