"""Read a run log back, and tell whether it is whole.

    python -m robot1.rasp.telemetry.log_reader logs/2027-01-06_1432_match
    python -m robot1.rasp.telemetry.log_reader logs/2027-01-06_1432_match --rebuild

"Whole" means four things, and check_run() verifies each:

  * the run was closed (meta.json has its end time and its counters);
  * raw_usb.bin ends on a complete record;
  * decoding raw_usb.bin again gives, line for line, the link lines of
    log.jsonl: nothing was lost or altered between the two files;
  * the logger dropped nothing, and the sequence numbers of the Teensy have
    no gap.

--rebuild writes log.jsonl again from raw_usb.bin, for a run recorded by a
decoder that has been fixed since. The lines the Raspberry Pi emitted itself
are kept.

Pure standard library, no thread: usable on the robot and on a PC.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

from . import log_format
from .logger import LOG_FILE, META_FILE, RAW_FILE


@dataclass
class RunCheck:
    """Result of check_run()."""

    folder: Path
    meta: dict = field(default_factory=dict)
    messages: int = 0
    lines: int = 0
    streams: dict = field(default_factory=dict)
    problems: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        """True when the run is whole."""
        return not self.problems

    def summary(self) -> str:
        """One line for a report."""
        if self.ok:
            return (f"LOG : OK — {self.messages} messages du lien, "
                    f"{self.lines} lignes, relu sans perte ({self.folder.name}).")
        return f"LOG : ECHEC — {'; '.join(self.problems)} ({self.folder.name})."


def read_meta(folder) -> dict:
    """Return meta.json of a run."""
    return json.loads((Path(folder) / META_FILE).read_text(encoding="utf-8"))


def read_raw(folder) -> list[tuple[int, int, bytes]]:
    """Return the records of raw_usb.bin: (t_ns, direction, payload)."""
    return list(log_format.iter_raw((Path(folder) / RAW_FILE).read_bytes()))


def read_lines(folder, name: str = LOG_FILE) -> list[dict]:
    """Return the lines of a JSONL file of a run, in order.

    A last line cut by a power loss is dropped; a damaged line elsewhere
    raises, because that is not what an interrupted run looks like.
    """
    text = (Path(folder) / name).read_text(encoding="utf-8")
    raw_lines = text.split("\n")
    complete = raw_lines[:-1]            # after the last "\n": empty, or cut
    return [json.loads(line) for line in complete if line]


def stream_lines(lines: list[dict], stream: str) -> list[dict]:
    """Return the lines of one stream."""
    return [line for line in lines if line.get("stream") == stream]


def check_run(folder) -> RunCheck:
    """Verify that a run log is whole. See the module docstring.

    Args:
        folder: the run folder.

    Returns:
        RunCheck: counters, and the list of problems (empty when whole).
    """
    folder = Path(folder)
    check = RunCheck(folder=folder)

    try:
        check.meta = read_meta(folder)
    except (OSError, ValueError) as error:
        check.problems.append(f"meta.json illisible ({error})")
        return check
    stats = check.meta.get("stats")
    if check.meta.get("t_end_rasp_ns") is None or stats is None:
        check.problems.append("run non clos (coupure avant la fin ?)")
        stats = stats or {}
    if check.meta.get("error"):
        check.problems.append(f"log arrete en cours de run : {check.meta['error']}")

    raw_bytes = (folder / RAW_FILE).read_bytes()
    records = list(log_format.iter_raw(raw_bytes))
    check.messages = len(records)
    trailing = log_format.raw_trailing_bytes(raw_bytes)
    if trailing:
        check.problems.append(f"{trailing} octets incomplets en fin de {RAW_FILE}")

    try:
        lines = read_lines(folder)
    except ValueError as error:
        check.problems.append(f"{LOG_FILE} abime ({error})")
        return check
    check.lines = len(lines)
    for line in lines:
        stream = line.get("stream")
        check.streams[stream] = check.streams.get(stream, 0) + 1

    # Compared as text: a NaN sent by the Teensy is not equal to itself.
    expected = [
        log_format.dumps(line)
        for t_ns, direction, payload in records
        for line in log_format.decode_message(t_ns, direction, payload)
    ]
    found = [log_format.dumps(line) for line in lines
             if log_format.is_link_line(line)]
    if expected != found:
        check.problems.append(
            f"{LOG_FILE} et {RAW_FILE} ne concordent pas "
            f"({len(found)} lignes du lien pour {len(expected)} attendues)"
        )

    if stats.get("dropped"):
        check.problems.append(f"{stats['dropped']} messages jetes par le logger")
    if stats.get("seq_gaps"):
        check.problems.append(f"{stats['seq_gaps']} trames Teensy manquantes")
    if stats.get("decode_errors"):
        check.problems.append(f"{stats['decode_errors']} lignes non ecrites")
    if stats and stats.get("lines") is not None:
        scans = stats.get("scans", 0)
        if stats["lines"] - scans != check.lines:
            check.problems.append(
                f"{check.lines} lignes dans {LOG_FILE}, "
                f"{stats['lines'] - scans} annoncees"
            )
    return check


def rebuild_log(folder) -> int:
    """Write log.jsonl again from raw_usb.bin, keeping the lines of the Pi.

    The link lines are decoded again; the lines the Raspberry Pi emitted are
    taken from the existing log.jsonl and merged back in time order (a link
    line first when two share the same instant, as the logger writes them).

    Args:
        folder: the run folder.

    Returns:
        int: number of lines written.
    """
    folder = Path(folder)
    link = [
        line
        for t_ns, direction, payload in read_raw(folder)
        for line in log_format.decode_message(t_ns, direction, payload)
    ]
    try:
        own = [line for line in read_lines(folder)
               if not log_format.is_link_line(line)]
    except (OSError, ValueError):
        own = []

    merged = sorted(
        [(line["t_rasp_ns"], 0, index, line) for index, line in enumerate(link)]
        + [(line["t_rasp_ns"], 1, index, line) for index, line in enumerate(own)],
        key=lambda item: item[:3],
    )
    target = folder / LOG_FILE
    temporary = target.with_suffix(".tmp")
    with open(temporary, "w", encoding="utf-8", newline="\n") as handle:
        for _, _, _, line in merged:
            handle.write(log_format.dumps(line))
            handle.write("\n")
    temporary.replace(target)
    return len(merged)


def main() -> int:
    """Check a run folder from the command line."""
    parser = argparse.ArgumentParser(
        description="Relit un dossier de log et verifie qu'il est complet.",
    )
    parser.add_argument("folder", help="dossier du run")
    parser.add_argument("--rebuild", action="store_true",
                        help="reecrire log.jsonl a partir de raw_usb.bin")
    args = parser.parse_args()

    if args.rebuild:
        count = rebuild_log(args.folder)
        print(f"{LOG_FILE} reecrit : {count} lignes.")

    check = check_run(args.folder)
    for stream, count in sorted(check.streams.items()):
        print(f"  {stream:<12} {count}")
    print(check.summary())
    return 0 if check.ok else 1


if __name__ == "__main__":
    sys.exit(main())
