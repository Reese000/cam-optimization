# CAM Optimization

**G-code performance analysis and optimization for CNC machining simulations.**

A single-script tool that parses CAM processor output files (CSV, TXT) and computes performance metrics: cut time, distance, feed utilization, rapid-vs-cutting ratios, and inefficient move patterns.

## What It Does

- Parses G-code output files (.csv, .txt) from CAM processors
- Calculates total cut time, distance, and feed utilization
- Detects rapid vs. cutting time ratios
- Identifies inefficient patterns (short segments, excessive retracts, unoptimized rapid moves)
- Generates structured optimization suggestions
- Batch analysis mode for directory-wide reports

## Usage

```bash
# Single file
python scripts/analyze_performance.py output.csv

# Batch directory
python scripts/analyze_performance.py --batch ./results/

# Custom max feed rate
python scripts/analyze_performance.py output.csv --max-feed 25000

# Write to file
python scripts/analyze_performance.py output.csv -o report.txt
```

## Tech Stack

- **Python 3.10+** — Standard library only (csv, argparse, dataclasses, math)
- **No external dependencies** — Ready to run with any Python 3.10+

## Status

**Alpha.** Single-script tool, fully functional for basic G-code CSV analysis. No test suite. Edge cases (non-standard G-code dialects, multi-spindle) not yet handled. Ready for use and iteration.
