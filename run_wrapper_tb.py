"""Compile and run the nemo_top_wrapper integration testbench with ModelSim/Questa.

Usage (from anywhere):
    python run_wrapper_tb.py          # batch run, log -> tb/nemo_top_wrapper_tb_log.txt
    python run_wrapper_tb.py -g       # open the GUI with all top-level signals
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent          # NEMO_RTL/
TOP = "nemo_top_wrapper_tb"
LOG = ROOT / "tb" / f"{TOP}_log.txt"


def run(cmd):
    print("Running:", " ".join(cmd))
    try:
        return subprocess.run(cmd, cwd=ROOT).returncode == 0
    except FileNotFoundError:
        print(f"Error: {cmd[0]} not found - add the ModelSim/Questa bin folder to PATH")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-g", "--gui", action="store_true", help="open the simulator GUI")
    args = ap.parse_args()

    os.chdir(ROOT)
    if not (ROOT / "work").exists() and not run(["vlib", "work"]):
        return 1
    if not run(["vlog", "-sv", "-work", "work", "-f", "tb/nemo_top_wrapper_tb.f"]):
        print("[X] Compile failed")
        return 1

    if args.gui:
        run(["vsim", "-voptargs=+acc", f"work.{TOP}", "-do", "add wave -r /*; run -all"])
        return 0

    run(["vsim", "-c", f"work.{TOP}", "-l", str(LOG), "-do", "run -all; quit -f"])
    text = LOG.read_text(errors="replace") if LOG.exists() else ""
    for line in text.splitlines():
        if "SUMMARY:" in line or "RESULT:" in line or "FAIL" in line:
            print(line)
    ok = "RESULT: PASS" in text
    print(f"\n{'[OK] PASS' if ok else '[X] FAIL'} - full log: {LOG}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
