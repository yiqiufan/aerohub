#!/usr/bin/env python3
import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple


DRONES = (1, 2, 3, 4)
PROCESS_NAMES = {
    "exploration_node",
    "traj_server",
    "mtsp_node",
    "poscmd_2_odom",
    "odom_visualization",
    "pcl_render_node",
    "map_pub",
}

METRIC_RE = re.compile(r"Drone (\d), \[([^,]+),([^,]+),([^,]+),([^\]]+)\]")
NO_GRID_RE = re.compile(r"\[exploration_node_(\d+)\]: No grid")
COVERAGE_RE = re.compile(
    r"\[exploration_node_(\d+)\]: Map coverage percent: ([0-9.]+) "
    r"known_cells=(\d+) total_cells=(\d+)"
)
FATAL_RE = re.compile(
    r"process has died|Segmentation|terminate called|can't subtract|Failed to run|/usr/local/bin/LKH|Traceback|Exception"
)


@dataclass
class CoverageSample:
    percent: float
    known_cells: int
    total_cells: int


@dataclass
class RunLog:
    metrics: Dict[int, Tuple[float, float, float, float]] = field(default_factory=dict)
    no_grid: Set[int] = field(default_factory=set)
    coverage: Dict[int, CoverageSample] = field(default_factory=dict)
    fatal_lines: List[str] = field(default_factory=list)
    last_no_grid_wall: Optional[float] = None
    last_metric_change_wall: Optional[float] = None

    def feed(self, line: str) -> None:
        metric = METRIC_RE.search(line)
        if metric:
            drone_id = int(metric.group(1))
            value = tuple(float(metric.group(i)) for i in range(2, 6))
            if self.metrics.get(drone_id) != value:
                self.last_metric_change_wall = time.monotonic()
            self.metrics[drone_id] = value

        no_grid = NO_GRID_RE.search(line)
        if no_grid:
            self.no_grid.add(int(no_grid.group(1)))
            self.last_no_grid_wall = time.monotonic()

        coverage = COVERAGE_RE.search(line)
        if coverage:
            drone_id = int(coverage.group(1))
            self.coverage[drone_id] = CoverageSample(
                percent=float(coverage.group(2)),
                known_cells=int(coverage.group(3)),
                total_cells=int(coverage.group(4)),
            )

        if FATAL_RE.search(line):
            self.fatal_lines.append(line.strip())

    def completion_mode(self, no_grid_grace_sec: float, metrics_stable_sec: float) -> Optional[str]:
        if set(DRONES) <= self.no_grid and set(DRONES) <= self.metrics.keys():
            if self.last_no_grid_wall is not None:
                if time.monotonic() - self.last_no_grid_wall >= no_grid_grace_sec:
                    return "no_grid"

        if set(DRONES) <= self.metrics.keys() and len(self.no_grid) >= 3:
            if self.last_metric_change_wall is not None:
                if time.monotonic() - self.last_metric_change_wall >= metrics_stable_sec:
                    return "stable_metrics"
        return None

    def complete(self, no_grid_grace_sec: float, metrics_stable_sec: float) -> bool:
        return self.completion_mode(no_grid_grace_sec, metrics_stable_sec) is not None

    def missing_no_grid(self) -> List[int]:
        if set(DRONES) - self.no_grid:
            return sorted(set(DRONES) - self.no_grid)
        return []

    def missing_coverage(self) -> List[int]:
        if set(DRONES) - self.coverage.keys():
            return sorted(set(DRONES) - self.coverage.keys())
        return []

    def coverage_summary(self) -> Tuple[float, float, float]:
        values = [self.coverage[drone_id].percent for drone_id in sorted(self.coverage)]
        if not values:
            return 0.0, 0.0, 0.0
        return sum(values) / len(values), min(values), max(values)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run clean ROS2 RACER pillar benchmark trials.")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--trigger-delay-sec", type=float, default=10.0)
    parser.add_argument("--completion-grace-sec", type=float, default=3.0)
    parser.add_argument("--metrics-stable-sec", type=float, default=20.0)
    parser.add_argument("--coverage", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--coverage-report-interval-sec", type=float, default=1.0)
    parser.add_argument("--output", default="benchmark_results/pillar_runs.csv")
    parser.add_argument("--log-dir", default="benchmark_results/logs")
    return parser.parse_args()


def matching_pids() -> List[int]:
    pids: List[int] = []
    self_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == self_pid:
            continue
        try:
            comm = (entry / "comm").read_text().strip()
        except OSError:
            continue
        if comm in PROCESS_NAMES:
            pids.append(pid)
    return pids


def cleanup_processes(timeout_sec: float = 5.0) -> None:
    for pid in matching_pids():
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if not matching_pids():
            return
        time.sleep(0.2)

    for pid in matching_pids():
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def start_launch(log_path: Path, run_log: RunLog, args: argparse.Namespace) -> subprocess.Popen:
    log_file = log_path.open("w", buffering=1)
    launch_args = ["ros2", "launch", "exploration_manager", "racer_pillar.launch.py", "rviz:=false"]
    if args.coverage:
        launch_args.extend(
            [
                "coverage_metrics:=true",
                f"coverage_report_interval_sec:={args.coverage_report_interval_sec}",
            ]
        )
    proc = subprocess.Popen(
        launch_args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid,
    )

    def reader() -> None:
        assert proc.stdout is not None
        with log_file:
            for line in proc.stdout:
                log_file.write(line)
                run_log.feed(line)

    threading.Thread(target=reader, daemon=True).start()
    return proc


def stop_launch(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=5)
    cleanup_processes()


def publish_trigger(trigger_log_path: Path) -> subprocess.Popen:
    msg = (
        "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, "
        "orientation: {w: 1.0}}}"
    )
    trigger_log = trigger_log_path.open("w")
    return subprocess.Popen(
        [
            "timeout",
            "18",
            "ros2",
            "topic",
            "pub",
            "-r",
            "10",
            "/move_base_simple/goal",
            "geometry_msgs/msg/PoseStamped",
            msg,
        ],
        stdout=trigger_log,
        stderr=subprocess.STDOUT,
    )


def run_once(run_id: int, args: argparse.Namespace) -> Dict[str, Any]:
    cleanup_processes()
    subprocess.run(["ros2", "daemon", "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    run_log = RunLog()
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    launch_log = log_dir / f"pillar_run_{run_id}.log"
    trigger_log = log_dir / f"pillar_run_{run_id}_trigger.log"
    proc = start_launch(launch_log, run_log, args)
    trigger_proc: Optional[subprocess.Popen] = None

    try:
        time.sleep(args.trigger_delay_sec)
        trigger_wall = time.monotonic()
        trigger_proc = publish_trigger(trigger_log)
        deadline = trigger_wall + args.timeout_sec

        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError(f"launch exited early with code {proc.returncode}")
            if run_log.fatal_lines:
                raise RuntimeError("fatal runtime log: " + run_log.fatal_lines[-1])
            if run_log.complete(args.completion_grace_sec, args.metrics_stable_sec):
                break
            time.sleep(0.2)
        else:
            missing = run_log.missing_no_grid()
            raise TimeoutError(f"run {run_id} timed out; missing No grid from drones {missing}")

        if trigger_proc.poll() is None:
            trigger_proc.terminate()
            trigger_proc.wait(timeout=5)

        metrics = run_log.metrics
        missing_coverage = run_log.missing_coverage()
        if args.coverage and missing_coverage:
            raise RuntimeError(f"run {run_id} missing coverage metrics from drones {missing_coverage}")

        exploration_time = max(metrics[drone_id][0] for drone_id in DRONES)
        total_length = sum(metrics[drone_id][1] for drone_id in DRONES)
        completion_mode = run_log.completion_mode(args.completion_grace_sec, args.metrics_stable_sec)
        coverage_avg, coverage_min, coverage_max = run_log.coverage_summary()
        row: Dict[str, Any] = {
            "run": run_id,
            "exploration_time_sec": exploration_time,
            "total_path_length_m": total_length,
            "wall_time_sec": time.monotonic() - trigger_wall,
            "completion_mode": completion_mode or "unknown",
            "coverage_percent": coverage_avg,
            "coverage_min_percent": coverage_min,
            "coverage_max_percent": coverage_max,
        }
        for drone_id in DRONES:
            row[f"coverage_drone_{drone_id}_percent"] = run_log.coverage[drone_id].percent
        return row
    finally:
        if trigger_proc is not None and trigger_proc.poll() is None:
            trigger_proc.terminate()
            try:
                trigger_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                trigger_proc.kill()
        stop_launch(proc)


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(
            output,
            fieldnames=[
                "run",
                "exploration_time_sec",
                "total_path_length_m",
                "wall_time_sec",
                "completion_mode",
                "coverage_percent",
                "coverage_min_percent",
                "coverage_max_percent",
                "coverage_drone_1_percent",
                "coverage_drone_2_percent",
                "coverage_drone_3_percent",
                "coverage_drone_4_percent",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    rows: List[Dict[str, Any]] = []
    for run_id in range(1, args.runs + 1):
        row = run_once(run_id, args)
        rows.append(row)
        write_csv(Path(args.output), rows)
        print(
            "run={run} exploration_time_sec={exploration_time_sec:.3f} "
            "total_path_length_m={total_path_length_m:.3f} wall_time_sec={wall_time_sec:.3f} "
            "completion_mode={completion_mode} coverage_percent={coverage_percent:.3f}".format(**row),
            flush=True,
        )

    avg_time = sum(row["exploration_time_sec"] for row in rows) / len(rows)
    avg_length = sum(row["total_path_length_m"] for row in rows) / len(rows)
    avg_coverage = sum(row["coverage_percent"] for row in rows) / len(rows)
    print(f"average_exploration_time_sec={avg_time:.3f}")
    print(f"average_total_path_length_m={avg_length:.3f}")
    print(f"average_coverage_percent={avg_coverage:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
