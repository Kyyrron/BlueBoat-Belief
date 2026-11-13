"""Belief beside a recorded simulation (VALIDATION_REPORT §5.2 / §5.3 as one command; dev-only).

It runs the superproject's ``tools/sim_record/record_run.py`` (the MCS launcher and the GCS,
offscreen, unmodified) with the installed ``belief_map`` node attached and recording its inputs,
then reports what Belief saw and replays the recorded inputs offline::

    python3 test/live_sim_check.py --world ~/worlds/<path>/<world> --trajectory <MCS name> \
        --out ~/ros2_ws/sim_runs/belief_runs/<name> [--sensor geometric] [--extra k:=v ...] \
        [--max-wall-min 20] [--allow-stale]

Rules it enforces, because a run launches Gazebo on the live ROS domain:
- **the owner must have agreed** (superproject skill ``sim-run-recording``);
- it refuses to start while any simulation, GCS or sonar processor is running (it reads
  ``/proc``; it starts no ``pgrep`` whose command line would itself look like a simulation);
- ``--out`` must not exist; nothing is overwritten (CM-7);
- Belief must be built in ``~/ros2_ws/install``.

With Perception's dummy detector (the only detector that exists today) the expected result is:
alignment valid, every image record unsearched, every ping geometry-only, coverage 0. The
offline replay with ``--no-require-detector`` then shows the swath geometry (labelled ungated).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HOME = Path.home()
WS = HOME / "ros2_ws"
SUPER = WS / "src" / "BlueBoat-SideScanSonar"
PKG = SUPER / "BlueBoat-Belief" / "blueboat_belief"
ROS = "source /opt/ros/jazzy/setup.bash && source %s/install/setup.bash" % WS
# what record_run.py itself treats as "a simulation is running" (its SIM_PATTERN), read from /proc
BUSY = re.compile(r"gz sim|sss_sim_node|master_control|sss_processor_node|blueboat_gcs\.main|"
                  r"gcs_driver\.py|mcs_driver\.py|record_run\.py")
INPUTS = ("/sss_processor/processed", "/sss_ai/detections", "/sss_ai/image_records",
          "/blueboat/odom", "/mavros/global_position/global")


def busy_processes() -> list:
    out = []
    me = os.getpid()
    for p in Path("/proc").iterdir():
        if not p.name.isdigit() or int(p.name) == me:
            continue
        try:
            cmd = (p / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace").strip()
        except OSError:
            continue
        if cmd and BUSY.search(cmd) and "live_sim_check" not in cmd:
            out.append(f"{p.name}: {cmd[:110]}")
    return out


def ros(cmd: str, timeout: float = 20.0) -> str:
    """Run a ros2 CLI command in a clean, ROS-sourced shell (no venv); return its output."""
    env = {"HOME": str(HOME), "LANG": "C.UTF-8", "PATH": "/usr/bin:/bin"}
    try:
        r = subprocess.run(["bash", "--noprofile", "--norc", "-c", f"{ROS} && {cmd}"], env=env,
                           capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return "TIMEOUT"


def sample(out: Path, stop: threading.Event, pid_file: Path) -> None:
    """During the run: the endpoints of Belief's inputs once, then /diagnostics every 20 s."""
    time.sleep(60.0)
    if stop.is_set():
        return
    info = {t: ros(f"ros2 topic info -v {t}") for t in INPUTS + ("/belief_map/belief_grid",)}
    (out / "topic_info.json").write_text(json.dumps(info, indent=1))
    k = 0
    while not stop.wait(20.0):
        k += 1
        diag = ros("ros2 topic echo --once /diagnostics", timeout=15.0)
        cpu = ""
        if pid_file.exists():
            cpu = subprocess.run(["ps", "-o", "pcpu=,rss=", "-p", pid_file.read_text().strip()],
                                 capture_output=True, text=True).stdout.strip()
        with open(out / "diagnostics_samples.txt", "a", encoding="utf-8") as fh:
            fh.write(f"=== sample {k} at {time.strftime('%H:%M:%S')} cpu%/rss_kB: {cpu}\n{diag}\n")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", required=True)
    ap.add_argument("--trajectory", required=True)
    ap.add_argument("--out", required=True, help="run folder to create (must not exist)")
    ap.add_argument("--sensor", default="geometric")
    ap.add_argument("--extra", action="append", default=[], help="passed to record_run.py --extra")
    ap.add_argument("--config", default="", help="Belief parameter YAML (default: the installed belief.yaml)")
    ap.add_argument("--max-wall-min", type=float, default=20.0)
    ap.add_argument("--allow-stale", action="store_true", help="record_run.py --allow-stale (label the run)")
    args = ap.parse_args(argv)

    out = Path(args.out).expanduser()
    world = Path(args.world).expanduser()
    busy = busy_processes()
    if busy:
        print("refused: a simulation, GCS or sonar processor is running:\n  " + "\n  ".join(busy))
        return 3
    if not (WS / "install" / "blueboat_belief").is_dir():
        print("refused: blueboat_belief is not in ~/ros2_ws/install (colcon build it first)")
        return 3
    if out.exists():
        print(f"refused: {out} exists (never overwritten)")
        return 3
    if not (world / "metadata.yaml").is_file():
        print(f"refused: {world} is not a world folder (no metadata.yaml)")
        return 3
    out.mkdir(parents=True)
    env = {"HOME": str(HOME), "LANG": "C.UTF-8", "PATH": "/usr/bin:/bin", "BLUEBOAT_DATA_DIR": str(out)}

    # ---- Belief first: its process names do not look like a simulation to record_run.py
    cfg = f" config:={args.config}" if args.config else ""
    blog = open(out / "belief_launch.log", "w", encoding="utf-8")
    belief = subprocess.Popen(
        ["bash", "--noprofile", "--norc", "-c",
         f"{ROS} && exec ros2 launch blueboat_belief belief_launch.py area_file:={world} "
         f"use_sim_time:=true record_inputs:=true data_dir:={out}{cfg}"],
        env=env, stdout=blog, stderr=subprocess.STDOUT, start_new_session=True)
    t_end = time.monotonic() + 40.0
    started = False
    while time.monotonic() < t_end and belief.poll() is None and not started:
        time.sleep(0.5)
        started = "snapshots ->" in (out / "belief_launch.log").read_text(errors="replace")
    if not started:
        print("refused: the Belief node did not start; see", out / "belief_launch.log")
        if belief.poll() is None:
            os.killpg(belief.pid, signal.SIGINT)
        return 1
    pid_file = out / "belief_node.pid"
    m = re.search(r"\[belief_node-\d+\]: process started with pid \[(\d+)\]",
                  (out / "belief_launch.log").read_text(errors="replace"))
    if m:
        pid_file.write_text(m.group(1))
    print(f"Belief is up (pid {m.group(1) if m else '?'}); starting the recorded simulation")

    # ---- the recorded simulation (unmodified superproject tool)
    cmd = [sys.executable, "tools/sim_record/record_run.py", "--world", str(world),
           "--trajectory", args.trajectory, "--sensor", args.sensor, "--name", "run",
           "--out", str(out), "--private-data-root", "--max-wall-min", str(args.max_wall_min)]
    for e in args.extra:
        cmd += ["--extra", e]
    if args.allow_stale:
        cmd.append("--allow-stale")
    stop = threading.Event()
    th = threading.Thread(target=sample, args=(out, stop, pid_file), daemon=True)
    th.start()
    t0 = time.monotonic()
    with open(out / "record_run.log", "w", encoding="utf-8") as rlog:
        rc = subprocess.call(cmd, cwd=str(SUPER), env=dict(os.environ, BLUEBOAT_DATA_DIR=str(out)),
                             stdout=rlog, stderr=subprocess.STDOUT)
    wall = time.monotonic() - t0
    stop.set()
    time.sleep(3.0)
    os.killpg(belief.pid, signal.SIGINT)
    try:
        brc = belief.wait(timeout=60.0)
    except subprocess.TimeoutExpired:
        os.killpg(belief.pid, signal.SIGTERM)
        brc = f"SIGTERM needed ({belief.wait(timeout=20.0)})"

    # ---- report
    rep = {"record_run_exit": rc, "wall_s": round(wall, 1), "belief_launch_exit": brc,
           "world": str(world), "trajectory": args.trajectory, "sensor": args.sensor,
           "extra": args.extra, "allow_stale": args.allow_stale}
    run_json = out / "run" / "run.json"
    if run_json.is_file():
        rj = json.loads(run_json.read_text())
        rep["run"] = {k: rj.get(k) for k in ("status", "left_running", "launch_log_counts")}
        rep["run"]["gcs"] = {k: (rj.get("gcs") or {}).get(k) for k in ("pings", "seabed_images")}
    bd = out / "data" / "Belief_data"
    finals = sorted(bd.glob("*/snapshot_final.json")) if bd.is_dir() else []
    bags = sorted(bd.glob("*-inputs")) if bd.is_dir() else []
    if finals:
        man = json.loads(finals[0].read_text())
        rep["belief_live"] = {"stats": man["stats"], "summary": man["summary"],
                              "counters": {k: v for k, v in man["counters"].items() if v},
                              "coverage_source": man["model_params"].get("coverage_source"),
                              "calibrated_observed": man["model_params"].get("detector_calibrated_observed"),
                              "alignment_source": man["model_params"].get("alignment_source")}
        import numpy as np

        z = np.load(finals[0].with_suffix(".npz"))
        rep["belief_live"]["odom_to_map"] = [float(z["odom_to_map_x"]), float(z["odom_to_map_y"]),
                                             float(z["odom_to_map_sigma_m"])]
        rep["belief_live"]["grid"] = [int(z["width"]), int(z["height"])]
        rep["belief_live"]["cells_with_a_look"] = int((z["n_looks"] > 0).sum())
        rep["belief_live"]["max_n_aspects"] = int(z["n_aspects"].max())
    if bags and (bags[0] / "metadata.yaml").is_file():
        meta = (bags[0] / "metadata.yaml").read_text()
        rep["bag_topics"] = dict(re.findall(r"name: (\S+)\n(?:.*\n){0,6}?\s*message_count: (\d+)", meta))
        venv = str(WS / ".venv" / "bin" / "python3")
        renv = {"HOME": str(HOME), "LANG": "C.UTF-8", "PATH": "/usr/bin:/bin", "PYTHONDONTWRITEBYTECODE": "1"}
        digests, ungated = [], None
        for name, extra in (("replay1", []), ("replay2", []), ("replay_ungated", ["--no-require-detector"])):
            r = subprocess.run([venv, "-B", "-m", "blueboat_belief.cli.replay", str(bags[0]), "--area", str(world),
                                "--out", str(out / name)] + extra, cwd=str(PKG), env=renv, capture_output=True, text=True)
            if r.returncode != 0:
                rep[name] = "failed: " + r.stderr[-300:]
                continue
            s = json.loads(r.stdout)
            if extra:
                ungated = {"stats": s["stats"], "summary": s["summary"]}
            else:
                digests.append(hashlib.sha256(Path(s["snapshot"]).read_bytes()).hexdigest())
        rep["replay_identical"] = len(digests) == 2 and digests[0] == digests[1]
        rep["replay_ungated"] = ungated
        if finals and digests:
            import numpy as np

            a = np.load(finals[0].with_suffix(".npz"))
            b = np.load(sorted((out / "replay1").glob("*/snapshot_final.npz"))[0])
            diff = [k for k in a.files if k != "grid_seq"
                    and not np.array_equal(a[k], b[k], equal_nan=a[k].dtype.kind == "f")]
            rep["live_vs_replay_differing_members"] = diff
    (out / "belief_live_check.json").write_text(json.dumps(rep, indent=1, sort_keys=True, default=str))
    print(json.dumps(rep, indent=1, sort_keys=True, default=str))
    left = busy_processes()
    if left:
        print("WARNING: still running after the run:\n  " + "\n  ".join(left))
    return 0 if rc == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
