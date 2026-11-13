"""Entry point of the 2D simulator.

    python -m robot1.sim2d.run --scenario carre
    python -m robot1.sim2d.run --scenario match --color YELLOW --rerun

Run from the repository root (sim2d and replay launch from there, tools/ and
tests/ from robot1/rasp/).

Three scenarios, and they answer different questions:

    carre  the simulator itself. Drives the link directly, no pathfinding, no
           strategy. Checks the kinematics moves the robot and that the
           odometry drifts the way the model says it should.
    ligne  the control law of the Teensy. Three straight lines (0, 30 and
           90 degrees) driven by a trapezoidal profile sent in SET_TARGET
           (pose and velocity, every tick). Checks the base reaches 1 m/s and
           stops without overshooting past the dead zone of the firmware.
    match  the production match loop, unchanged, over 100 simulated seconds.
           Checks it survives, and measures what it costs.
    tournee
           the production match loop over 100 s, its strategy replaced by a
           loop of targets that never ends, with a scripted opponent circling
           in our half. Checks the planning budget with a robot that moves
           for the whole match, and how the replanning behaves.

The scenarios print a verdict and return a non-zero exit code on failure, so
they can be chained in a campaign of runs with no test framework in the way.
"""

import argparse
import logging
import math
import os
import sys
import time

# The mode has to be set before anything reads ROBOT_MODE, and reading it is
# the first thing comm/ does. Forcing it here is also what keeps a simulated
# run from ever touching a serial port by accident.
os.environ["ROBOT_MODE"] = "sim2d"

from usb_com import (                                                 # noqa: E402
    Messages,
    pack_message,
    payload_size,
    unpack_message,
)

from robot1.rasp.comm import init_robot                               # noqa: E402
from robot1.rasp.world import (                                      # noqa: E402
    FIELD_HEIGHT_MM,
    FIELD_WIDTH_MM,
    MATCH_DURATION_S,
    START_POSE,
    mirror_for_team,
)

from . import transport, viz                                          # noqa: E402
from .clock import ScaledClock, VirtualClock                          # noqa: E402
from .kinematics import ACCEL_MM_S2, MAX_SPEED_MM_S                   # noqa: E402
from .robot import DriftModel, SimulatedRobot                         # noqa: E402
from .teensy import ARRIVED_DISTANCE_MM                               # noqa: E402

logger = logging.getLogger("SIM2D")

TICK_S = 0.05                 # 20 Hz, same rate as the real match loop
SQUARE_SIDE_MM = 800.0
SQUARE_ORIGIN = (1000.0, 600.0)
LEG_TIMEOUT_S = 15.0
ARRIVED_MM = 15.0
# Straight lines: (start x, start y, direction in degrees, length mm). 0 and
# 90 degrees are the best and the worst case of the per-wheel limiter (one
# wheel carries the whole speed at 90); 30 degrees is as bad as 90.
LINES = (
    (700.0, 1000.0, 0.0, 1600.0),
    (700.0, 400.0, 30.0, 1200.0),
    (1500.0, 400.0, 90.0, 1200.0),
)
# Profile acceleration. 10 % under ACCEL_MM_S2: the per-wheel limiter
# (320 rpm/s) allows 1005 mm/s^2 at 90 degrees, so a profile at 1000 leaves
# the position loop no authority during the ramps. Measured with KP = 1:
# overshoot 19 mm at 1000 mm/s^2, 6 mm at 900; with KP = 4, 120 mm and 10 mm.
LINE_ACCEL_MM_S2 = 900.0
LINE_SETTLE_S = 1.5           # time left after the profile ends
LINE_SPEED_RATIO = 0.98       # share of MAX_SPEED_MM_S to reach
LINE_OVERSHOOT_MM = ARRIVED_DISTANCE_MM   # inside the firmware dead zone


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
        if len(data) == payload_size(Messages.ODOMETRY):
            reported["pose"] = unpack_message(Messages.ODOMETRY, data)[:3]
            reported["frames"] += 1

    com.add_callback(on_odometry, Messages.ODOMETRY.value)

    com.send_bytes(pack_message(Messages.SET_ODOMETRIE, x0, y0, 0.0))

    if args.rerun:
        viz.init()

    legs_done = 0
    for index in range(1, len(corners)):
        target_x, target_y = corners[index]
        previous_x, previous_y = corners[index - 1]
        heading = math.atan2(target_y - previous_y, target_x - previous_x)
        # Plain pose target, as app.py sends it: no trajectory velocity.
        com.send_bytes(pack_message(Messages.SET_TARGET, target_x,
                                    target_y, heading, 0.0, 0.0, 0.0))

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


# ── Scenario: straight line ───────────────────────────────────────────────

def trapezoid(t: float, length: float, v_max: float, accel: float) -> tuple:
    """Position and speed along a trapezoidal profile at time t.

    Stand-in for the profile generator of the match loop, written only for
    this scenario. Falls back to a triangle when the length is too short to
    reach v_max.

    Returns:
        tuple: (position_mm, speed_mm_s, total_duration_s).
    """
    v_peak = min(v_max, math.sqrt(accel * length))
    t_ramp = v_peak / accel
    d_ramp = 0.5 * accel * t_ramp ** 2
    t_cruise = (length - 2.0 * d_ramp) / v_peak
    total = 2.0 * t_ramp + t_cruise
    if t <= 0.0:
        return 0.0, 0.0, total
    if t < t_ramp:
        return 0.5 * accel * t ** 2, accel * t, total
    if t < t_ramp + t_cruise:
        return d_ramp + v_peak * (t - t_ramp), v_peak, total
    if t < total:
        left = total - t
        return length - 0.5 * accel * left ** 2, accel * left, total
    return length, 0.0, total


def scenario_ligne(args) -> int:
    """Drive straight lines at 1 m/s with the velocity in SET_TARGET."""
    clock = build_clock(args.speed)
    x0, y0, _, _ = LINES[0]
    sim = build_robot(x0, y0, 0.0, args.seed, args.noise)
    com, mode = init_robot(logger)
    print(f"Mode de communication : {mode}")
    print(f"Acceleration du profil : {args.accel:.0f} mm/s^2")

    frames = {"count": 0}
    com.add_callback(lambda _data: frames.update(count=frames["count"] + 1),
                     Messages.ODOMETRY.value)
    if args.rerun:
        viz.init()

    failures = []
    print()
    print("Dir.  Duree   V max   Depass.  Lat. max  Arret odo.  Arret vrai")
    for x0, y0, direction_deg, length in LINES:
        ux = math.cos(math.radians(direction_deg))
        uy = math.sin(math.radians(direction_deg))
        # The robot is placed at the start, at rest (SET_TARGET then holds it).
        com.send_bytes(pack_message(Messages.SET_ODOMETRIE, x0, y0, 0.0))
        com.send_bytes(pack_message(Messages.SET_TARGET, x0, y0, 0.0,
                                    0.0, 0.0, 0.0))
        sim.max_speed_mm_s = 0.0
        _, _, duration = trapezoid(0.0, length, MAX_SPEED_MM_S, args.accel)
        t_start = sim.time
        max_along = 0.0
        max_lateral = 0.0

        while sim.time - t_start < duration + LINE_SETTLE_S:
            sim.step(TICK_S)
            com.publish_odometry(clock.now())
            odom_x, odom_y, _ = sim.odometry_pose()
            along = (odom_x - x0) * ux + (odom_y - y0) * uy
            lateral = -(odom_x - x0) * uy + (odom_y - y0) * ux
            max_along = max(max_along, along)
            max_lateral = max(max_lateral, abs(lateral))

            # Pose of the profile now, velocity half a tick ahead: the
            # Teensy extrapolates the pose at that velocity until the next
            # message, and the mid-period velocity makes that exact under a
            # constant acceleration. Sending the velocity of now instead runs
            # the robot ~25 mm/s too fast all along the braking ramp: 23 mm of
            # overshoot at 0 degrees with KP = 1.
            tau = sim.time - t_start
            position, _, _ = trapezoid(tau, length, MAX_SPEED_MM_S,
                                       args.accel)
            _, speed, _ = trapezoid(tau + 0.5 * TICK_S, length,
                                    MAX_SPEED_MM_S, args.accel)
            com.send_bytes(pack_message(
                Messages.SET_TARGET, x0 + position * ux, y0 + position * uy,
                0.0, speed * ux, speed * uy, 0.0,
            ))
            if args.rerun:
                viz.publish(clock.now(), sim)
            clock.sleep(TICK_S)

        overshoot = max_along - length
        odom_x, odom_y, _ = sim.odometry_pose()
        true_x, true_y, _ = sim.true_pose()
        stop_odom = (odom_x - x0) * ux + (odom_y - y0) * uy - length
        stop_true = (true_x - x0) * ux + (true_y - y0) * uy - length
        print(f"{direction_deg:3.0f}   {duration:4.2f} s  "
              f"{sim.max_speed_mm_s:4.0f}   {overshoot:+5.1f} mm  "
              f"{max_lateral:5.1f} mm   {stop_odom:+5.1f} mm   "
              f"{stop_true:+5.1f} mm")

        if sim.max_speed_mm_s < LINE_SPEED_RATIO * MAX_SPEED_MM_S:
            failures.append(
                f"{direction_deg:.0f} deg : vitesse max "
                f"{sim.max_speed_mm_s:.0f} mm/s, sous "
                f"{LINE_SPEED_RATIO * MAX_SPEED_MM_S:.0f}")
        if overshoot > LINE_OVERSHOOT_MM:
            failures.append(
                f"{direction_deg:.0f} deg : depassement de {overshoot:.1f} "
                f"mm (tolere {LINE_OVERSHOOT_MM:.0f}, zone morte Teensy)")
        if abs(stop_odom) > LINE_OVERSHOOT_MM:
            failures.append(
                f"{direction_deg:.0f} deg : arret a {stop_odom:+.1f} mm "
                f"du but")

    print()
    print("Depassement et arrets mesures sur l'odometrie, ce que la Teensy "
          "asservit ;")
    print("l'arret vrai inclut la derive d'odometrie.")
    print(f"Trames d'odometrie      : {frames['count']}")

    print()
    if failures:
        for item in failures:
            print(f"ECHEC : {item}")
        return 1
    print("LIGNE : OK — 1 m/s atteint, arret sans depassement hors zone morte.")
    return 0


# ── Scenario: full match ──────────────────────────────────────────────────

def scenario_match(args) -> int:
    """Run the production match loop over 100 simulated seconds."""
    from robot1.rasp import app
    from robot1.rasp.comm import set_fake_button
    from robot1.rasp.vision import detection

    # The colour switch is read, not wired: pressed means BLUE (app.py).
    set_fake_button(app.PIN_COULEUR, args.color.upper() == "BLUE")
    # No LiDAR to open, and no acquisition thread: simulated scans will be
    # pushed here. Until then the match runs without opponent detection, which is
    # honest -- it is not pretending to test avoidance.
    detection.use_pushed_scans()

    couleur = app.lire_couleur_equipe()
    clock = build_clock(args.speed)

    # Same start pose as StratManager.get_start_position(), whose robot does
    # not exist yet: the simulated robot must be placed before app.Robot().
    start_x, start_y, start_theta = mirror_for_team(START_POSE, couleur)
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
            print("  Regarder le nombre de plans A* : il ne devrait "
                  "replanifier que sur evenement.")

    if failures:
        for item in failures:
            print(f"ECHEC : {item}")
        return 1
    print("MATCH : OK — la boucle de production a tenu 100 s simulees.")
    return 0


# ── Scenario: tour of the table ───────────────────────────────────────────

# Targets for YELLOW, mirrored for BLUE. Clear of the quarries (all assumed
# full) and of the borders; the middle crossing passes between C5 and C6.
TOUR_TARGETS = (
    (1000.0, 1000.0), (2000.0, 1000.0), (2600.0, 1000.0), (2000.0, 1650.0),
    (1000.0, 1650.0), (700.0, 1000.0), (1000.0, 350.0), (2000.0, 350.0),
)
# Scripted opponent: a circle in our half, slowly.
TOUR_OPPONENT_CENTRE = (900.0, 1000.0)
TOUR_OPPONENT_RADIUS_MM = 350.0
TOUR_OPPONENT_RAD_S = 0.25          # ~90 mm/s along the circle
TOUR_MIN_TARGETS = 6


def scenario_tournee(args) -> int:
    """Run the match loop over 100 s on a loop of targets, with an opponent."""
    from robot1.rasp import app
    from robot1.rasp.comm import set_fake_button
    from robot1.rasp.strategy import Action, TypeAction
    from robot1.rasp.vision import detection
    from robot1.rasp.world import mirror_for_team

    set_fake_button(app.PIN_COULEUR, args.color.upper() == "BLUE")
    detection.use_pushed_scans()
    couleur = app.lire_couleur_equipe()
    clock = build_clock(args.speed)

    from robot1.rasp.strategy import StratManager
    start = StratManager(couleur).get_start_position()
    sim = build_robot(*start, args.seed, args.noise)

    # The opponent the detection would report. Stands in for synthetic scans
    # until the simulator produces them; app.py reads it through its adapter.
    centre = mirror_for_team(TOUR_OPPONENT_CENTRE, couleur)
    opponent = {"x": centre[0], "y": centre[1]}

    def scripted_opponent():
        return opponent["x"], opponent["y"], 1.0

    detection.get_opponent = scripted_opponent

    if args.rerun:
        viz.init()

    robot = app.Robot(couleur, clock=clock)
    targets = [mirror_for_team(t, couleur) for t in TOUR_TARGETS]
    robot.strategie.liste_actions = [
        Action(TypeAction.DEPLACEMENT, cible_x=x, cible_y=y)
        for _ in range(20) for x, y in targets
    ]

    # Time spent in the A*, measured around the planner of the loop.
    finder = robot.cerveau
    plan_time = {"s": 0.0, "fails": 0}
    original_plan = finder.plan

    def timed_plan(*a, **k):
        t0 = time.perf_counter()
        result = original_plan(*a, **k)
        plan_time["s"] += time.perf_counter() - t0
        if not result.ok:
            plan_time["fails"] += 1
        return result

    finder.plan = timed_plan

    com = robot.com
    com.publish_odometry(clock.now(), force=True)
    robot.attendre_tirette()

    contact = app.OPPONENT_RADIUS_MM + finder.robot_radius
    min_gap = math.inf
    ticks_in_contact = 0
    wall_start = time.perf_counter()
    begin = clock.now()
    while clock.now() - begin < MATCH_DURATION_S:
        angle = TOUR_OPPONENT_RAD_S * (clock.now() - begin)
        opponent["x"] = centre[0] + TOUR_OPPONENT_RADIUS_MM * math.cos(angle)
        opponent["y"] = centre[1] + TOUR_OPPONENT_RADIUS_MM * math.sin(angle)

        sim.step(TICK_S)
        com.publish_odometry(clock.now())
        robot.update()

        tx, ty, _ = sim.true_pose()
        gap = math.hypot(tx - opponent["x"], ty - opponent["y"])
        min_gap = min(min_gap, gap)
        if gap < contact:
            ticks_in_contact += 1
        if args.rerun:
            viz.publish(clock.now(), sim)
        clock.sleep(TICK_S)

    wall_elapsed = time.perf_counter() - wall_start
    robot.stopper_tout()
    reached = robot.strategie.etape_actuelle
    nav = robot.navigation

    print()
    print(f"Cibles atteintes        : {reached}")
    print(f"Distance parcourue      : "
          f"{sim.distance_travelled_mm / 1000.0:.2f} m")
    print(f"Plans A*                : {nav.plans} "
          f"({plan_time['fails']} echecs), {plan_time['s'] * 1000:.0f} ms "
          f"au total")
    print(f"Ecart min a l'adversaire: {min_gap:.0f} mm entre centres "
          f"(contact sous {contact:.0f})")
    print(f"Ticks au contact        : {ticks_in_contact}")
    print(f"Temps mur total         : {wall_elapsed:.2f} s")

    failures = []
    if reached < TOUR_MIN_TARGETS:
        failures.append(f"{reached} cibles atteintes, "
                        f"moins de {TOUR_MIN_TARGETS}")
    true_x, true_y, _ = sim.true_pose()
    if not (0.0 <= true_x <= FIELD_WIDTH_MM
            and 0.0 <= true_y <= FIELD_HEIGHT_MM):
        failures.append("le robot est sorti de la table")
    if args.speed == "max" and wall_elapsed > args.budget:
        failures.append(f"budget de temps depasse ({wall_elapsed:.2f} s "
                        f"pour {args.budget:.1f} s)")

    print()
    if ticks_in_contact:
        print("NOTE : contact avec l'adversaire. Le planificateur seul ne "
              "garantit pas l'evitement (arret de securite a venir).")
    if failures:
        for item in failures:
            print(f"ECHEC : {item}")
        return 1
    print("TOURNEE : OK — 100 s de deplacements, budget tenu.")
    return 0


# ── Helpers ───────────────────────────────────────────────────────────────

def main() -> int:
    """Parse the command line and run the requested scenario."""
    parser = argparse.ArgumentParser(
        description="Simulateur 2D cinematique — CDR-Nantes",
    )
    parser.add_argument("--scenario",
                        choices=["carre", "ligne", "match", "tournee"],
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
    parser.add_argument("--accel", type=float, default=LINE_ACCEL_MM_S2,
                        help="acceleration du profil, mm/s^2 (ligne)")
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
    if args.scenario == "ligne":
        return scenario_ligne(args)
    if args.scenario == "tournee":
        return scenario_tournee(args)
    return scenario_match(args)


if __name__ == "__main__":
    sys.exit(main())
