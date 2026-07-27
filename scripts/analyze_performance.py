#!/usr/bin/env python3
"""
Performance analysis and optimization script for CNC machining simulations.

Analyzes output files (.csv, .txt) from CAM processors to:
- Calculate total cut time, distance, and feed utilization
- Detect rapid vs cutting time ratios
- Identify inefficient move patterns (short segments, excessive retracts)
- Generate optimization suggestions

Usage:
    python scripts/analyze_performance.py <output_file>
    python scripts/analyze_performance.py --batch <directory>
"""

import csv
import os
import sys
import math
import argparse
from pathlib import Path
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class MoveStats:
    """Statistics for a single move."""
    gcode: str
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    feed: Optional[float] = None
    distance: float = 0.0
    time_seconds: float = 0.0
    is_rapid: bool = False
    is_coolant_on: bool = False
    line_number: int = 0


@dataclass
class PerformanceReport:
    """Aggregated performance metrics for a simulation."""
    filename: str = ""
    total_moves: int = 0
    rapid_moves: int = 0
    cutting_moves: int = 0
    total_distance_mm: float = 0.0
    rapid_distance_mm: float = 0.0
    cutting_distance_mm: float = 0.0
    total_time_seconds: float = 0.0
    rapid_time_seconds: float = 0.0
    cutting_time_seconds: float = 0.0
    max_feed_used: float = 0.0
    avg_cutting_feed: float = 0.0
    feed_utilization_pct: float = 0.0  # actual feed / max possible
    coolant_on_distance_mm: float = 0.0
    short_segment_count: int = 0  # moves < 0.1mm
    retract_count: int = 0
    tool_changes: int = 0
    warnings: list = field(default_factory=list)
    suggestions: list = field(default_factory=list)


class GCodeParser:
    """Parse G-code output files and extract move data."""

    # G-codes that represent motion
    MOTION_CODES = {"G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"}
    RAPID_CODES = {"G0", "G00"}
    LINEAR_CODES = {"G1", "G01"}
    ARC_CODES = {"G2", "G02", "G3", "G03"}
    COOLANT_ON = {"M8", "M08", "M7", "M07"}
    COOLANT_OFF = {"M9", "M09"}
    TOOL_CHANGE = {"M6", "M06"}

    @staticmethod
    def parse_csv(filepath: str) -> list[MoveStats]:
        """Parse a CSV output file with columns like: Line, Gcode, X, Y, Z, F."""
        moves = []
        try:
            with open(filepath, "r") as f:
                reader = csv.DictReader(f)
                for i, row in enumerate(reader):
                    gcode = row.get("Gcode", row.get("Command", "")).strip()
                    move = MoveStats(
                        gcode=gcode,
                        x=_safe_float(row.get("X")),
                        y=_safe_float(row.get("Y")),
                        z=_safe_float(row.get("Z")),
                        feed=_safe_float(row.get("F", row.get("Feed"))),
                        line_number=i + 1,
                    )
                    moves.append(move)
        except Exception as e:
            print(f"Error parsing CSV {filepath}: {e}")
        return moves

    @staticmethod
    def parse_txt(filepath: str) -> list[MoveStats]:
        """Parse a text file with one G-code command per line."""
        moves = []
        try:
            with open(filepath, "r") as f:
                for i, line in enumerate(f, 1):
                    line = line.strip()
                    if not line or line.startswith("(") or line.startswith("%"):
                        continue
                    move = MoveStats(gcode=line, line_number=i)
                    # Extract coordinates from the line
                    move.x = _extract_param(line, "X")
                    move.y = _extract_param(line, "Y")
                    move.z = _extract_param(line, "Z")
                    move.feed = _extract_param(line, "F")
                    moves.append(move)
        except Exception as e:
            print(f"Error parsing TXT {filepath}: {e}")
        return moves

    @classmethod
    def parse(cls, filepath: str) -> list[MoveStats]:
        """Auto-detect format and parse."""
        ext = Path(filepath).suffix.lower()
        if ext == ".csv":
            return cls.parse_csv(filepath)
        else:
            return cls.parse_txt(filepath)


class PerformanceAnalyzer:
    """Analyze parsed moves and generate performance report."""

    SHORT_SEGMENT_THRESHOLD_MM = 0.1

    def __init__(self, moves: list[MoveStats], max_machine_feed: float = 30000.0):
        self.moves = moves
        self.max_machine_feed = max_machine_feed
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0
        self.coolant_active = False

    def analyze(self, filename: str = "") -> PerformanceReport:
        """Run full analysis and return report."""
        report = PerformanceReport(filename=filename)
        report.total_moves = len(self.moves)

        for move in self.moves:
            self._classify_move(move)
            self._compute_distance(move)
            self._compute_time(move)
            self._update_totals(report, move)

        self._compute_averages(report)
        self._detect_issues(report)
        self._generate_suggestions(report)
        return report

    def _classify_move(self, move: MoveStats):
        """Determine move type from G-code."""
        gcode_upper = move.gcode.upper()
        first_word = gcode_upper.split()[0] if gcode_upper else ""

        move.is_rapid = first_word in self.RAPID_CODES

        # Track coolant state
        for word in gcode_upper.split():
            if word in self.COOLANT_ON:
                self.coolant_active = True
            elif word in self.COOLANT_OFF:
                self.coolant_active = False
        move.is_coolant_on = self.coolant_active

    def _compute_distance(self, move: MoveStats):
        """Calculate Euclidean distance for this move."""
        x = move.x if move.x is not None else self.prev_x
        y = move.y if move.y is not None else self.prev_y
        z = move.z if move.z is not None else self.prev_z

        dx = x - self.prev_x
        dy = y - self.prev_y
        dz = z - self.prev_z
        move.distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        self.prev_x = x
        self.prev_y = y
        self.prev_z = z

    def _compute_time(self, move: MoveStats):
        """Calculate time for this move based on distance and feed."""
        if move.distance < 1e-6:
            move.time_seconds = 0.0
            return

        feed = move.feed
        if feed is None or feed <= 0:
            # Rapids are at machine max rate
            feed = self.max_machine_feed if move.is_rapid else 0.0

        if feed > 0:
            # Feed is in mm/min, distance in mm
            move.time_seconds = (move.distance / feed) * 60.0
        else:
            move.time_seconds = 0.0

    def _update_totals(self, report: PerformanceReport, move: MoveStats):
        """Accumulate totals in the report."""
        report.total_distance_mm += move.distance
        report.total_time_seconds += move.time_seconds

        if move.is_rapid:
            report.rapid_moves += 1
            report.rapid_distance_mm += move.distance
            report.rapid_time_seconds += move.time_seconds
        else:
            report.cutting_moves += 1
            report.cutting_distance_mm += move.distance
            report.cutting_time_seconds += move.time_seconds

        if move.feed and move.feed > report.max_feed_used:
            report.max_feed_used = move.feed

        if move.is_coolant_on and not move.is_rapid:
            report.coolant_on_distance_mm += move.distance

        if move.distance < self.SHORT_SEGMENT_THRESHOLD_MM and not move.is_rapid:
            report.short_segment_count += 1

        if move.is_rapid and move.distance > 1.0:
            # Count significant rapid moves (potential retracts/repositions)
            report.retract_count += 1

        gcode_upper = move.gcode.upper().split()[0]
        if gcode_upper in self.TOOL_CHANGE:
            report.tool_changes += 1

    def _compute_averages(self, report: PerformanceReport):
        """Calculate average metrics."""
        if report.cutting_moves > 0:
            total_cut_feed = sum(
                m.feed for m in self.moves
                if m.feed and not m.is_rapid
            )
            report.avg_cutting_feed = total_cut_feed / report.cutting_moves

        if report.total_distance_mm > 0:
            report.feed_utilization_pct = (
                report.cutting_distance_mm / report.total_distance_mm
            ) * 100.0

    def _detect_issues(self, report: PerformanceReport):
        """Detect performance issues and add warnings."""
        # High rapid ratio
        if report.total_time_seconds > 0:
            rapid_pct = (report.rapid_time_seconds / report.total_time_seconds) * 100
            if rapid_pct > 40:
                report.warnings.append(
                    f"High rapid time ratio: {rapid_pct:.1f}% "
                    f"({report.rapid_time_seconds:.1f}s of {report.total_time_seconds:.1f}s)"
                )

        # Excessive short segments
        if report.short_segment_count > 100:
            report.warnings.append(
                f"Excessive short segments: {report.short_segment_count} moves "
                f"< {self.SHORT_SEGMENT_THRESHOLD_MM}mm (potential arc fitting opportunity)"
            )

        # Many retracts
        if report.retract_count > report.cutting_moves * 0.3 and report.cutting_moves > 0:
            report.warnings.append(
                f"High retract count: {report.retract_count} retracts for "
                f"{report.cutting_moves} cutting moves"
            )

        # Low feed utilization
        if report.feed_utilization_pct < 50 and report.total_distance_mm > 0:
            report.warnings.append(
                f"Low cutting distance ratio: {report.feed_utilization_pct:.1f}% "
                f"({report.cutting_distance_mm:.1f}/{report.total_distance_mm:.1f} mm)"
            )

        # No coolant during cutting
        if report.cutting_distance_mm > 0 and report.coolant_on_distance_mm == 0:
            report.warnings.append(
                "No coolant detected during cutting moves"
            )

    def _generate_suggestions(self, report: PerformanceReport):
        """Generate optimization suggestions based on analysis."""
        if report.short_segment_count > 50:
            report.suggestions.append(
                "Enable arc fitting/breaking to consolidate short linear segments "
                "into arcs. This reduces G-code size and improves controller smoothing."
            )

        if report.retract_count > 10:
            report.suggestions.append(
                "Consider optimizing retract heights and entry/exit moves. "
                "Reduce retract height if clearance allows."
            )

        if report.feed_utilization_pct < 60:
            report.suggestions.append(
                "Review cutting parameters - feed rate may be conservative. "
                "Consider increasing feed for roughing operations."
            )

        rapid_pct = 0
        if report.total_time_seconds > 0:
            rapid_pct = (report.rapid_time_seconds / report.total_time_seconds) * 100
        if rapid_pct > 35:
            report.suggestions.append(
                "Optimize toolpath ordering to minimize rapid traverse distance. "
                "Consider sorting operations by proximity."
            )

        if report.tool_changes > 3:
            report.suggestions.append(
                f"Multiple tool changes ({report.tool_changes}) detected. "
                "Consider grouping operations by tool to reduce change count."
            )


def _safe_float(val) -> Optional[float]:
    """Safely convert a value to float."""
    if val is None or val == "":
        return None
    try:
        return float(val)
    except (ValueError, TypeError):
        return None


def _extract_param(gcode_line: str, param: str) -> Optional[float]:
    """Extract a parameter value from a G-code line (e.g., X, Y, Z, F)."""
    import re
    pattern = rf"{param}([-+]?\d*\.?\d+)"
    match = re.search(pattern, gcode_line, re.IGNORECASE)
    if match:
        try:
            return float(match.group(1))
        except ValueError:
            return None
    return None


def format_report(report: PerformanceReport) -> str:
    """Format a PerformanceReport as human-readable text."""
    lines = []
    lines.append("=" * 60)
    lines.append(f"  PERFORMANCE REPORT: {report.filename}")
    lines.append("=" * 60)
    lines.append("")

    lines.append("  SUMMARY")
    lines.append("  " + "-" * 40)
    lines.append(f"  Total moves:        {report.total_moves}")
    lines.append(f"  Cutting moves:      {report.cutting_moves}")
    lines.append(f"  Rapid moves:        {report.rapid_moves}")
    lines.append(f"  Tool changes:       {report.tool_changes}")
    lines.append("")

    lines.append("  DISTANCE")
    lines.append("  " + "-" * 40)
    lines.append(f"  Total distance:     {report.total_distance_mm:.2f} mm")
    lines.append(f"  Cutting distance:   {report.cutting_distance_mm:.2f} mm")
    lines.append(f"  Rapid distance:     {report.rapid_distance_mm:.2f} mm")
    lines.append("")

    lines.append("  TIME")
    lines.append("  " + "-" * 40)
    _fmt_time = lambda s: f"{int(s//60):02d}:{s%60:05.2f}"
    lines.append(f"  Total time:         {_fmt_time(report.total_time_seconds)}")
    lines.append(f"  Cutting time:       {_fmt_time(report.cutting_time_seconds)}")
    lines.append(f"  Rapid time:         {_fmt_time(report.rapid_time_seconds)}")
    lines.append("")

    lines.append("  FEED")
    lines.append("  " + "-" * 40)
    lines.append(f"  Max feed used:      {report.max_feed_used:.1f} mm/min")
    lines.append(f"  Avg cutting feed:   {report.avg_cutting_feed:.1f} mm/min")
    lines.append(f"  Cutting ratio:      {report.feed_utilization_pct:.1f}%")
    lines.append("")

    if report.short_segment_count > 0 or report.retract_count > 0:
        lines.append("  EFFICIENCY INDICATORS")
        lines.append("  " + "-" * 40)
        lines.append(f"  Short segments:     {report.short_segment_count}")
        lines.append(f"  Retract moves:      {report.retract_count}")
        lines.append("")

    if report.warnings:
        lines.append("  WARNINGS")
        lines.append("  " + "-" * 40)
        for w in report.warnings:
            lines.append(f"  ! {w}")
        lines.append("")

    if report.suggestions:
        lines.append("  OPTIMIZATION SUGGESTIONS")
        lines.append("  " + "-" * 40)
        for i, s in enumerate(report.suggestions, 1):
            lines.append(f"  {i}. {s}")
        lines.append("")

    lines.append("=" * 60)
    return "\n".join(lines)


def analyze_file(filepath: str, max_feed: float = 30000.0) -> PerformanceReport:
    """Analyze a single file and return the report."""
    moves = GCodeParser.parse(filepath)
    if not moves:
        report = PerformanceReport(filename=os.path.basename(filepath))
        report.warnings.append("No moves found in file")
        return report

    analyzer = PerformanceAnalyzer(moves, max_machine_feed=max_feed)
    return analyzer.analyze(filename=os.path.basename(filepath))


def analyze_batch(directory: str, max_feed: float = 30000.0) -> list[PerformanceReport]:
    """Analyze all supported files in a directory."""
    reports = []
    dir_path = Path(directory)
    if not dir_path.is_dir():
        print(f"Error: {directory} is not a directory")
        return reports

    extensions = {".csv", ".txt", ".nc", ".gcode", ".tap", ".ngc"}
    files = sorted(
        f for f in dir_path.iterdir()
        if f.suffix.lower() in extensions and f.is_file()
    )

    if not files:
        print(f"No supported files found in {directory}")
        return reports

    for filepath in files:
        report = analyze_file(str(filepath), max_feed)
        reports.append(report)
        print(format_report(report))
        print()

    # Print summary comparison if multiple files
    if len(reports) > 1:
        print("\n" + "=" * 60)
        print("  BATCH SUMMARY")
        print("=" * 60)
        print(f"  {'File':<25} {'Time':>10} {'Distance':>12} {'Cut%':>8}")
        print("  " + "-" * 55)
        for r in reports:
            _fmt_time = lambda s: f"{int(s//60):02d}:{s%60:05.2f}"
            print(
                f"  {r.filename:<25} "
                f"{_fmt_time(r.total_time_seconds):>10} "
                f"{r.total_distance_mm:>10.1f}mm "
                f"{r.feed_utilization_pct:>6.1f}%"
            )
        print("=" * 60)

    return reports


def main():
    parser = argparse.ArgumentParser(
        description="Analyze CNC machining simulation output for performance metrics."
    )
    parser.add_argument(
        "input",
        help="Input file (.csv, .txt) or directory (with --batch)"
    )
    parser.add_argument(
        "--batch", action="store_true",
        help="Analyze all files in the given directory"
    )
    parser.add_argument(
        "--max-feed", type=float, default=30000.0,
        help="Maximum machine feed rate in mm/min (default: 30000)"
    )
    parser.add_argument(
        "--output", "-o",
        help="Write report to file instead of stdout"
    )

    args = parser.parse_args()

    if args.batch:
        reports = analyze_batch(args.input, max_feed=args.max_feed)
    else:
        if not os.path.isfile(args.input):
            print(f"Error: File not found: {args.input}")
            sys.exit(1)
        report = analyze_file(args.input, max_feed=args.max_feed)
        output = format_report(report)
        if args.output:
            with open(args.output, "w") as f:
                f.write(output)
            print(f"Report written to {args.output}")
        else:
            print(output)


if __name__ == "__main__":
    main()
