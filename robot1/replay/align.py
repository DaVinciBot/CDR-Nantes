"""Put the Raspberry Pi and the Teensy on one clock.

    python -m robot1.replay.align logs/2027-01-06_1432_match

Every frame of the Teensy carries t_us, the micros() of the board when its data
was measured; the logger dates its arrival with t_rasp_ns. For one frame:

    t_rasp - t_teensy = offset + drift * t_teensy + latency,    latency >= 0

The latency is never negative, so the lowest values of that difference, over a
window of about a second, come from the frames that went through fastest: they
draw the offset. A straight line through those minima also gives the drift
between the two quartz. No ping is needed, the frames of the run are enough.

The common time "t" is in seconds, in the time base of the Teensy: a Teensy
line keeps its own measurement time, a line of the Pi is moved into that base.

t_us is a u32 that wraps every 71.6 minutes, longer than a session can be. It
is unrolled in the order the frames arrived: a step back of more than a second
is a wrap when the Pi saw as much time go by, and a restart of the Teensy
otherwise (LOG_EVENT BOOT says so too). After a restart t_us starts again from
zero: the run is cut in segments, each aligned on its own, and the later ones
are carried into the time base of the first through the clock of the Pi, which
never restarts. "t" stays monotonic across a restart.

A run without any Teensy frame (log off on the link, dummy mode) gets
t = time of the Pi since the start of the run, method "rasp_only".

log.jsonl and raw_usb.bin are never modified: the result is added to the lines
in memory, and the command line writes it to align.json next to them.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from usb_com.python.messages import LogEvent

from robot1.rasp.telemetry import log_reader

ALIGN_FILE = "align.json"

METHOD_ENVELOPE = "envelope"
METHOD_RASP_ONLY = "rasp_only"

# Width of the windows whose minimum draws the lower envelope.
WINDOW_S = 1.0
WRAP_US = 1 << 32
# A step back of t_us larger than this is a wrap or a restart. Smaller ones are
# frames received out of order: STATUS is sent from loop(), the per-tick
# frames from the tick, so their t_us interleave.
BACKWARD_TOLERANCE_US = 1_000_000
# A step back is a wrap when the Pi saw this close to the time it implies.
WRAP_MATCH_S = 2.0
# After a first fit, a window minimum this many robust sigmas off the line is
# dropped before the refit. Above: a window where every frame was late (a busy
# Pi). Below: a frame dated on another clock than the others, which no latency
# explains (the first frame of a sim2d match, forced before the first step).
OUTLIER_SIGMAS = 3.0
# Floor of that robust sigma, s: minima that sit exactly on the line (sim2d)
# must not make every other point an outlier.
OUTLIER_FLOOR_S = 1e-5

_BOOT = LogEvent.BOOT.value


@dataclass
class Segment:
    """One stretch of the run between two restarts of the Teensy.

    t_rasp = offset_s + (1 + drift_ppm * 1e-6) * t_teensy, both in seconds,
    t_teensy being the unrolled micros() of the board.
    """

    index: int
    frames: int
    first_t_rasp_s: float
    last_t_rasp_s: float
    offset_s: float
    drift_ppm: float
    windows: int
    residual_us: float
    latency_ms: dict = field(default_factory=dict)

    @property
    def rate(self) -> float:
        """Seconds of the Pi per second of the Teensy."""
        return 1.0 + self.drift_ppm * 1e-6

    def teensy_to_rasp(self, t_teensy_s):
        """Time of the Pi at which the Teensy read t_teensy_s (no latency)."""
        return self.offset_s + self.rate * t_teensy_s


@dataclass
class Alignment:
    """Result of align_lines(): how to move any line onto the common time t."""

    method: str
    segments: list = field(default_factory=list)
    rasp_origin_s: float = 0.0

    @property
    def restarts(self) -> int:
        """Restarts of the Teensy seen during the run."""
        return max(len(self.segments) - 1, 0)

    def rasp_to_t(self, t_rasp_s):
        """Common time of an instant of the Pi, in seconds."""
        if self.method == METHOD_RASP_ONLY:
            return t_rasp_s - self.rasp_origin_s
        first = self.segments[0]
        return (t_rasp_s - first.offset_s) / first.rate

    def teensy_to_t(self, segment: int, t_teensy_s):
        """Common time of a measurement of the Teensy, in seconds."""
        if segment == 0:
            return t_teensy_s
        return self.rasp_to_t(self.segments[segment].teensy_to_rasp(t_teensy_s))

    def to_dict(self) -> dict:
        """Plain dict, for align.json and report.json."""
        return {
            "method": self.method,
            "restarts": self.restarts,
            "rasp_origin_s": self.rasp_origin_s,
            "segments": [asdict(segment) for segment in self.segments],
        }


# ── Unrolling t_us ────────────────────────────────────────────────────────

def is_anchor(line: dict) -> bool:
    """Tell whether a line is a Teensy frame dated by the Teensy."""
    return line.get("src") == "teensy" and isinstance(line.get("t_us"), int)


def unwrap_teensy_times(lines: list[dict]) -> tuple[list, list]:
    """Unroll t_us and cut the run at each restart of the Teensy.

    Args:
        lines: the lines of log.jsonl, in file order.

    Returns:
        tuple: (segment, t_teensy_s), one entry per line. Both are None for a
        line that is not a Teensy frame dated by the Teensy.
    """
    segments = [None] * len(lines)
    times = [None] * len(lines)
    segment = -1
    wraps = 0
    last_raw = None
    last_rasp_s = None

    for index, line in enumerate(lines):
        if not is_anchor(line):
            continue
        raw = line["t_us"]
        rasp_s = line["t_rasp_ns"] * 1e-9
        boot = line.get("stream") == "event" and line.get("event") == _BOOT

        restart = last_raw is None or boot
        wrapped = False
        if not restart and raw < last_raw - BACKWARD_TOLERANCE_US:
            if raw + WRAP_US - last_raw < WRAP_US // 2:
                ahead_s = (raw + WRAP_US - last_raw) * 1e-6
                wrapped = abs(ahead_s - (rasp_s - last_rasp_s)) < WRAP_MATCH_S
            restart = not wrapped
        if restart:
            segment += 1
            wraps = 0
            last_raw = raw
        elif wrapped:
            wraps += 1
            last_raw = raw

        value = raw + wraps * WRAP_US
        if not wrapped and raw > last_raw + WRAP_US // 2 and wraps:
            # Late frame from before the last wrap.
            value -= WRAP_US
        elif raw > last_raw:
            last_raw = raw
        last_rasp_s = rasp_s
        segments[index] = segment
        times[index] = value * 1e-6
    return segments, times


# ── Estimating offset and drift ───────────────────────────────────────────

def fit_envelope(t_teensy, t_rasp, window_s: float = WINDOW_S) -> dict:
    """Fit the lower envelope of t_rasp - t_teensy.

    Args:
        t_teensy: measurement times of the Teensy, s (unrolled).
        t_rasp: reception times of the Pi, s, same length.
        window_s: width of the windows whose minimum is kept.

    Returns:
        dict: offset_s, drift_ppm, windows, residual_us, latency_ms
        (p50, p95, max of the latency left once the line is removed).
    """
    t_teensy = np.asarray(t_teensy, dtype=float)
    t_rasp = np.asarray(t_rasp, dtype=float)
    diff = t_rasp - t_teensy

    bins = np.floor((t_teensy - t_teensy.min()) / window_s).astype(np.int64)
    order = np.lexsort((diff, bins))
    _, first = np.unique(bins[order], return_index=True)
    chosen = order[first]
    x = t_teensy[chosen]
    y = diff[chosen]

    offset, slope = float(y.min()), 0.0
    keep = np.ones(len(x), dtype=bool)
    if len(x) >= 2 and np.ptp(x) > 0.0:
        slope, offset = np.polyfit(x, y, 1)
        # A few passes: an outlier far off pulls the first line enough to
        # hide a smaller one.
        for _ in range(3):
            residual = y - (offset + slope * x)
            centre = np.median(residual[keep])
            sigma = 1.4826 * np.median(np.abs(residual[keep] - centre))
            new_keep = (np.abs(residual - centre)
                        <= OUTLIER_SIGMAS * max(sigma, OUTLIER_FLOOR_S))
            if new_keep.sum() < 2 or np.ptp(x[new_keep]) <= 0.0:
                break
            keep = new_keep
            slope, offset = np.polyfit(x[keep], y[keep], 1)

    fitted = offset + slope * x[keep]
    residual_us = float(np.std(y[keep] - fitted) * 1e6) if keep.any() else 0.0
    latency = (diff - (offset + slope * t_teensy)) * 1e3
    return {
        "offset_s": float(offset),
        "drift_ppm": float(slope * 1e6),
        "windows": int(len(x)),
        "residual_us": residual_us,
        "latency_ms": {
            "p50": float(np.percentile(latency, 50)),
            "p95": float(np.percentile(latency, 95)),
            "max": float(latency.max()),
        },
    }


# ── Whole run ─────────────────────────────────────────────────────────────

def estimate(lines: list[dict], meta: dict | None = None) -> tuple:
    """Estimate the alignment of a run, without touching the lines.

    Args:
        lines: the lines of log.jsonl, in file order.
        meta: meta.json of the run, for the origin of a run without Teensy.

    Returns:
        tuple: (Alignment, segment per line, unrolled t_teensy per line).
    """
    segments, times = unwrap_teensy_times(lines)
    count = max((s for s in segments if s is not None), default=-1) + 1

    if count == 0:
        origin_ns = (meta or {}).get("t_start_rasp_ns")
        if origin_ns is None:
            origin_ns = lines[0]["t_rasp_ns"] if lines else 0
        return (Alignment(METHOD_RASP_ONLY, [], origin_ns * 1e-9),
                segments, times)

    fitted = []
    for index in range(count):
        members = [i for i, s in enumerate(segments) if s == index]
        t_teensy = [times[i] for i in members]
        t_rasp = [lines[i]["t_rasp_ns"] * 1e-9 for i in members]
        fit = fit_envelope(t_teensy, t_rasp)
        fitted.append(Segment(
            index=index, frames=len(members),
            first_t_rasp_s=min(t_rasp), last_t_rasp_s=max(t_rasp), **fit,
        ))
    return Alignment(METHOD_ENVELOPE, fitted), segments, times


def align_lines(lines: list[dict], meta: dict | None = None) -> Alignment:
    """Estimate the alignment and write "t" and "segment" into every line.

    A Teensy frame dated by the Teensy gets its own measurement time. Every
    other line (the Pi, and the few Teensy messages without t_us) gets the
    time of the Pi moved into the common base, latency included. "segment"
    is the Teensy segment current at that point of the file.

    Args:
        lines: the lines of log.jsonl, changed in place.
        meta: meta.json of the run.

    Returns:
        Alignment: the estimate.
    """
    alignment, segments, times = estimate(lines, meta)
    current = 0
    for line, segment, t_teensy in zip(lines, segments, times):
        if segment is not None:
            current = segment
            line["t"] = float(alignment.teensy_to_t(segment, t_teensy))
        else:
            line["t"] = float(alignment.rasp_to_t(line["t_rasp_ns"] * 1e-9))
        line["segment"] = current
    return alignment


def align_run(folder) -> tuple[list[dict], Alignment]:
    """Read log.jsonl of a run and align it.

    Returns:
        tuple: (lines with "t" and "segment", Alignment).
    """
    meta = log_reader.read_meta(folder)
    lines = log_reader.read_lines(folder)
    return lines, align_lines(lines, meta)


def describe(alignment: Alignment) -> list[str]:
    """Lines of text that tell the alignment, for a terminal."""
    if alignment.method == METHOD_RASP_ONLY:
        return ["Alignement : aucune trame datee par la Teensy, "
                "temps de la Rasp seul."]
    text = [f"Alignement : {len(alignment.segments)} segment(s), "
            f"{alignment.restarts} redemarrage(s) de la Teensy."]
    for segment in alignment.segments:
        latency = segment.latency_ms
        text.append(
            f"  segment {segment.index} : {segment.frames} trames, "
            f"decalage {segment.offset_s:+.6f} s, "
            f"derive {segment.drift_ppm:+.1f} ppm, "
            f"residu {segment.residual_us:.0f} us, "
            f"latence p50 {latency['p50']:.2f} ms / p95 {latency['p95']:.2f} ms"
            f" / max {latency['max']:.2f} ms"
        )
    return text


def main() -> int:
    """Align a run folder from the command line and write align.json."""
    parser = argparse.ArgumentParser(
        description="Aligne les horloges Teensy et Rasp d'un dossier de run.",
    )
    parser.add_argument("folder", help="dossier du run")
    args = parser.parse_args()

    folder = Path(args.folder)
    _, alignment = align_run(folder)
    for text in describe(alignment):
        print(text)
    target = folder / ALIGN_FILE
    target.write_text(json.dumps(alignment.to_dict(), indent=2) + "\n",
                      encoding="utf-8")
    print(f"Ecrit : {target}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
