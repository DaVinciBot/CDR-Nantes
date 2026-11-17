import logging
import math

from robot1.rasp.telemetry.dashboard import (
    RobotDashboard,
    _DashboardLogHandler,
)


class _TextValue:
    def __init__(self) -> None:
        self.value = None

    def set(self, value: str) -> None:
        self.value = value


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
