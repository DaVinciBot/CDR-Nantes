"""Entry point of the 2D simulator.

    python -m robot1.sim2d.run --scenario carre
    python -m robot1.sim2d.run --scenario match --color YELLOW --rerun

Run from the repository root (sim2d and replay launch from there, tools/ and
tests/ from robot1/rasp/).

Two scenarios in lot 1, and they answer two different questions:

    carre  the simulator itself. Drives the link directly, no pathfinding, no
           strategy. Checks the kinematics moves the robot and that the
           odometry drifts the way the model says it should.
    match  the production match loop, unchanged, over 100 simulated seconds.
           Checks it survives, and measures what it costs.

The scenarios print a verdict and return a non-zero exit code on failure, so
they can be chained in a campaign (lot 5) with no test framework in the way.
"""

import argparse
import logging
import math
import os
import struct
import sys
import time

# The mode has to be set before anything reads ROBOT_MODE, and reading it is
# the first thing comm/ does. Forcing it here is also what keeps a simulated
# run from ever touching a serial port by accident.
os.environ["ROBOT_MODE"] = "sim2d"

from usb_com import Messages                                          # noqa: E402

from robot1.rasp.comm import init_robot                               # noqa: E402
from robot1.rasp.world import FIELD_HEIGHT_MM, FIELD_WIDTH_MM         # noqa: E402

from . import transport, viz                                          # noqa: E402
from .clock import ScaledClock, VirtualClock                          # noqa: E402
from .robot import DriftModel, SimulatedRobot                         # noqa: E402

logger = logging.getLogger("SIM2D")

TICK_S = 0.05                 # 20 Hz, same rate as the real match loop
MATCH_DURATION_S = 100.0      # Eurobot regulation
SQUARE_SIDE_MM = 800.0
SQUARE_ORIGIN = (1000.0, 600.0)
LEG_TIMEOUT_S = 15.0
ARRIVED_MM = 15.0


def build_clock(speed: str):
    """Return the clock matching the requested speed.

    Args:
        speed: "max" for no pacing at all, or a float of simulated seconds
            per real second.

    Returns:
        VirtualClock or ScaledClock.
    """
    if speed == "max":
        return VirtualClock()
    return ScaledClock(speed=float(speed))


def build_robot(x, y, theta, seed, noise):
    """Build the simulated robot and hand it to the transport.

    Args:
        x: initial X, in mm.
        y: initial Y, in mm.
        theta: initial heading, in rad.
        seed: noise seed, for reproducibility.
        noise: keep the random walk. False leaves only the systematic scale
            error, which makes the drift check exact.

    Returns:
        SimulatedRobot: the robot, already deposited for Sim2dCom.
    """
    drift = DriftModel()
    if not noise:
        drift.translation_noise_mm_per_sqrt_m = 0.0
        drift.heading_noise_rad_per_sqrt_rad = 0.0
    sim = SimulatedRobot(x=x, y=y, theta=theta, drift=drift, seed=seed)
    transport.set_robot(sim)
    return sim


# ── Scenario: square ──────────────────────────────────────────────────────

def scenario_carre(args) -> int:
    """Drive a square through the simulated link and measure the drift."""
    x0, y0 = SQUARE_ORIGIN
    side = SQUARE_SIDE_MM
    corners = [
        (x0, y0),
        (x0 + side, y0),
        (x0 + side, y0 + side),
        (x0, y0 + side),
        (x0, y0),
    ]

    clock = build_clock(args.speed)
    sim = build_robot(x0, y0, 0.0, args.seed, args.noise)
    com, mode = init_robot(logger)
    print(f"Mode de communication : {mode}")

    # Same callback contract as app.py: the loop only ever sees the odometry.
    reported = {"pose": (x0, y0, 0.0), "frames": 0}

    def on_odometry(data: bytes) -> None:
        if len(data) >= 24:
            reported["pose"] = struct.unpack("<ddd", data[:24])
            reported["frames"] += 1

    com.add_callback(on_odometry, Messages.UPDATE_ROLLING_BASIS.value)

    send_pose(com, Messages.SET_ODOMETRIE, x0, y0, 0.0)

    if args.rerun:
        viz.init()

    legs_done = 0
    for index in range(1, len(corners)):
        target_x, target_y = corners[index]
        previous_x, previous_y = corners[index - 1]
        heading = math.atan2(target_y - previous_y, target_x - previous_x)
        send_pose(com, Messages.SET_TARGET_POSITION, target_x, target_y,
                  heading)

        deadline = clock.now() + LEG_TIMEOUT_S
        while clock.now() < deadline:
            sim.step(TICK_S)
            com.publish_odometry(clock.now())
            if args.rerun:
                viz.publish(clock.now(), sim)
            clock.sleep(TICK_S)
            true_x, true_y, _ = sim.true_pose()
            if math.hypot(target_x - true_x, target_y - true_y) < ARRIVED_MM:
                legs_done += 1
                break
        else:
            print(f"ECHEC : cote {index} non atteint en {LEG_TIMEOUT_S:.0f} s")

    return report_carre(sim, reported, legs_done, len(corners) - 1, args)


def report_carre(sim, reported, legs_done, legs_total, args) -> int:
    """Print the square verdict and return the exit code.

    The drift is checked on the DISTANCE the odometry believes it travelled,
    not on the final position error. On a closed square a symmetric scale error
    cancels: the outward legs overestimate in one direction, the return legs in
    the other. Checking the final position would therefore pass whatever the
    scale error is, which is the opposite of a test.
    """
    true_mm = sim.distance_travelled_mm
    odom_mm = sim.odom_distance_mm
    expected_scale = sim.drift.translation_scale
    measured_scale = (odom_mm / true_mm - 1.0) if true_mm > 0.0 else 0.0

    angle_deg = math.degrees(sim.angle_travelled_rad)
    heading_deg = math.degrees(sim.heading_error_rad())
    expected_heading_deg = sim.drift.heading_scale * angle_deg

    print()
    print(f"Trajets termines        : {legs_done}/{legs_total}")
    print(f"Distance vraie          : {true_mm / 1000.0:.3f} m")
    print(f"Distance mesuree        : {odom_mm / 1000.0:.3f} m")
    print(f"Angle parcouru          : {angle_deg:.0f} deg")
    print(f"Trames d'odometrie      : {reported['frames']}")
    print()
    print(f"Echelle attendue        : {expected_scale * 100:+.3f} %")
    print(f"Echelle mesuree         : {measured_scale * 100:+.3f} %")
    print(f"Erreur de cap attendue  : {expected_heading_deg:+.3f} deg")
    print(f"Erreur de cap mesuree   : {heading_deg:+.3f} deg")
    print(f"Erreur de position max  : {sim.max_position_error_mm:.1f} mm")
    print(f"Erreur de position fin  : {sim.position_error_mm():.1f} mm "
          f"(un carre ferme annule une erreur d'echelle symetrique)")

    failures = []
    if legs_done != legs_total:
        failures.append("le robot n'a pas boucle le carre")
    if reported["frames"] == 0:
        failures.append("aucune trame d'odometrie recue par le callback")

    # The tolerance comes from the noise model itself, not from a guessed
    # percentage. The per-step noises are independent, so their variances add:
    # sigma_total = noise_coefficient * sqrt(total travelled). A fixed
    # percentage would fail perfectly correct runs, since the random walk is
    # here about half the size of the systematic error over a 3 m square.
    drift = sim.drift
    sigma_mm = drift.translation_noise_mm_per_sqrt_m * math.sqrt(
        true_mm / 1000.0
    )
    sigma_scale = (sigma_mm / true_mm) if true_mm > 0.0 else 0.0
    sigma_heading_deg = math.degrees(
        drift.heading_noise_rad_per_sqrt_rad
        * math.sqrt(sim.angle_travelled_rad)
    )

    # Floor: with the noise off the check is exact, only rounding remains.
    band_scale = max(3.0 * sigma_scale, 0.005 * expected_scale)
    band_heading = max(3.0 * sigma_heading_deg,
                       0.005 * abs(expected_heading_deg))

    print(f"Bande acceptee (3 sigma): echelle +/-{band_scale * 100:.3f} %, "
          f"cap +/-{band_heading:.3f} deg")

    if abs(measured_scale - expected_scale) > band_scale:
        failures.append(
            f"derive de distance hors bande : {measured_scale * 100:+.3f} % "
            f"contre {expected_scale * 100:+.3f} % +/-{band_scale * 100:.3f}"
        )
    if abs(heading_deg - expected_heading_deg) > band_heading:
        failures.append(
            f"derive de cap hors bande : {heading_deg:+.3f} deg contre "
            f"{expected_heading_deg:+.3f} +/-{band_heading:.3f}"
        )

    print()
    if failures:
        for item in failures:
            print(f"ECHEC : {item}")
        return 1
    print("CARRE : OK — distance et cap derivent comme le modele le dit.")
    return 0


# ── Scenario: full match ──────────────────────────────────────────────────

def scenario_match(args) -> int:
    """Run the production match loop over 100 simulated seconds."""
    from robot1.rasp import app
    from robot1.rasp.comm import set_fake_button
    from robot1.rasp.vision import detection

    # The colour switch is read, not wired: pressed means BLUE (app.py).
    set_fake_button(app.PIN_COULEUR, args.color.upper() == "BLUE")
    # No LiDAR to open, and no acquisition thread: lot 2 will push real scans
    # here. Until then the match runs without opponent detection, which is
    # honest -- it is not pretending to test avoidance.
    detection.use_pushed_scans()

    couleur = app.lire_couleur_equipe()
    clock = build_clock(args.speed)

    from robot1.rasp.world import Terrain

    start_x, start_y, start_theta = Terrain(couleur).get_start_position()
    sim = build_robot(start_x, start_y, start_theta, args.seed, args.noise)

    if args.rerun:
        viz.init()

    robot = app.Robot(couleur, clock=clock)
    com = robot.com
    com.publish_odometry(clock.now(), force=True)
    robot.attendre_tirette()

    wall_start = time.perf_counter()
    time_in_sim = 0.0
    time_in_update = 0.0
    ticks = 0
    start_pose = sim.true_pose()

    begin = clock.now()
    while clock.now() - begin < MATCH_DURATION_S:
        t0 = time.perf_counter()
        sim.step(TICK_S)
        com.publish_odometry(clock.now())
        t1 = time.perf_counter()
        robot.update()
        t2 = time.perf_counter()

        if args.rerun:
            viz.publish(clock.now(), sim)

        time_in_sim += t1 - t0
        time_in_update += t2 - t1
        ticks += 1
        clock.sleep(TICK_S)

    wall_elapsed = time.perf_counter() - wall_start
    robot.stopper_tout()
    return report_match(sim, com, start_pose, ticks, wall_elapsed,
                        time_in_sim, time_in_update, args)


def report_match(sim, com, start_pose, ticks, wall_elapsed, time_in_sim,
                 time_in_update, args) -> int:
    """Print the match verdict and return the exit code."""
    true_x, true_y, true_theta = sim.true_pose()
    odom_x, odom_y, _ = sim.odometry_pose()
    moved = math.hypot(true_x - start_pose[0], true_y - start_pose[1])

    print()
    print(f"Ticks simules           : {ticks} "
          f"({MATCH_DURATION_S:.0f} s a {1 / TICK_S:.0f} Hz)")
    print(f"Trames d'odometrie      : {com.frames_sent}")
    print(f"Pose vraie finale       : "
          f"({true_x:.0f}, {true_y:.0f}, {math.degrees(true_theta):.0f} deg)")
    print(f"Pose odometrique finale : ({odom_x:.0f}, {odom_y:.0f})")
    print(f"Distance parcourue      : "
          f"{sim.distance_travelled_mm / 1000.0:.2f} m")
    print(f"Derive finale           : {sim.position_error_mm():.1f} mm")
    print(f"Deplacement net         : {moved:.0f} mm")
    print()
    print(f"Temps mur total         : {wall_elapsed:.2f} s")
    print(f"  dont simulation       : {time_in_sim:.2f} s "
          f"({time_in_sim / ticks * 1000:.2f} ms/tick)")
    print(f"  dont robot.update()   : {time_in_update:.2f} s "
          f"({time_in_update / ticks * 1000:.2f} ms/tick)")

    failures = []
    if com.frames_sent == 0:
        failures.append("aucune trame d'odometrie envoyee")
    if moved < 100.0:
        failures.append(f"le robot n'a pas bouge ({moved:.0f} mm)")
    if not (-500.0 <= true_x <= FIELD_WIDTH_MM + 500.0
            and -500.0 <= true_y <= FIELD_HEIGHT_MM + 500.0):
        failures.append("le robot est sorti tres au large de la table")

    print()
    if args.speed == "max":
        budget = args.budget
        verdict = "OK" if wall_elapsed <= budget else "NON TENU"
        print(f"Budget de temps ({budget:.1f} s) : {verdict}")
        if wall_elapsed > budget:
            print("  Cause connue : app.py::update() replanifie en A* a "
                  "chaque tick.")
            print("  A traiter dans la re-API du pathfinder, pas ici.")

    if failures:
        for item in failures:
            print(f"ECHEC : {item}")
        return 1
    print("MATCH : OK — la boucle de production a tenu 100 s simulees.")
    return 0


# ── Helpers ───────────────────────────────────────────────────────────────

def send_pose(com, message, x: float, y: float, theta: float) -> None:
    """Send a pose message the way app.py builds it.

    Args:
        com: the transport.
        message: a usb_com.Messages member taking a `<ddd` payload.
        x: X in mm.
        y: Y in mm.
        theta: heading in rad.
    """
    com.send_bytes(message.to_bytes() + struct.pack("<ddd", x, y, theta))


def main() -> int:
    """Parse the command line and run the requested scenario."""
    parser = argparse.ArgumentParser(
        description="Simulateur 2D cinematique — CDR-Nantes",
    )
    parser.add_argument("--scenario", choices=["carre", "match"],
                        default="carre", help="scenario a jouer")
    parser.add_argument("--color", choices=["BLUE", "YELLOW", "blue",
                                            "yellow"],
                        default="BLUE", help="couleur d'equipe (match)")
    parser.add_argument("--seed", type=int, default=1,
                        help="graine du bruit, pour reproduire un run")
    parser.add_argument("--speed", default="max",
                        help="'max' ou secondes simulees par seconde reelle")
    parser.add_argument("--no-noise", dest="noise", action="store_false",
                        help="derive systematique seule, sans marche aleatoire")
    parser.add_argument("--rerun", action="store_true",
                        help="ouvrir la vue Rerun")
    parser.add_argument("--budget", type=float, default=1.0,
                        help="budget de temps mur pour un match, en secondes")
    parser.add_argument("--verbose", action="store_true",
                        help="logs de la boucle de production")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(levelname)s | %(name)s | %(message)s",
    )

    print(f"Scenario '{args.scenario}' — graine {args.seed}, "
          f"vitesse {args.speed}, bruit "
          f"{'actif' if args.noise else 'desactive'}")
    print()

    if args.scenario == "carre":
        return scenario_carre(args)
    return scenario_match(args)


if __name__ == "__main__":
    sys.exit(main())
