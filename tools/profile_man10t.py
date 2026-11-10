"""Orchestrate the native MAN10T profiling experiment on Linux/WSL2."""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import statistics
import subprocess
import tempfile
import threading
import time


def command(*args):
    return subprocess.check_output(args, text=True, timeout=10).strip()


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def annotate_gpu(row, samples):
    start, end = row["start_s"], row["end_s"]
    segment = [s for s in samples if start <= s[0] <= end]
    row.update(gpu_n=len(segment), gpu_mean_w=None, gpu_peak_w=None,
               gpu_busy_pct=None, gpu_energy_j=None)
    if segment:
        row.update(gpu_mean_w=statistics.mean(s[1] for s in segment),
                   gpu_peak_w=max(s[1] for s in segment),
                   gpu_busy_pct=statistics.mean(s[2] for s in segment))
    if len(segment) >= 2:
        points = [(start, segment[0][1]), *[(s[0], s[1]) for s in segment], (end, segment[-1][1])]
        row["gpu_energy_j"] = sum((b[0] - a[0]) * (a[1] + b[1]) / 2 for a, b in zip(points, points[1:]))


def max_error(first, second):
    return max(math.dist(a, b) for a, b in zip(first, second, strict=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ("ply", "reference", "out", "clean", "instrumented"):
        parser.add_argument(f"--{option}", type=Path, required=True)
    parser.add_argument("--power-plan", default=None)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    profile_dir = out / "profiles"
    profile_dir.mkdir(exist_ok=True)
    routes = args.reference / "routes.json"
    shutil.copyfile(routes, out / "routes.json")
    baseline = json.loads((args.reference / "metrics.json").read_text(encoding="utf-8"))
    shutil.copyfile(args.reference / "metrics.json", out / "python-baseline.json")
    gpu = shutil.which("nvidia-smi") or "/usr/lib/wsl/lib/nvidia-smi"
    gpu_name = command(gpu, "-i", "0", "--query-gpu=name,driver_version", "--format=csv,noheader")
    logical = os.cpu_count()
    cpu = next(s.partition(":")[2].strip() for s in Path("/proc/cpuinfo").read_text().splitlines()
               if s.startswith("model name"))
    source = sorted([*root.glob("include/fusionforce/*.hpp"), *root.glob("src/*.cpp"),
                     *root.glob("tools/profile_man10t.*"), *root.glob("viewer/assets/data/vehicle/MAN/**/*.json"),
                     *root.glob("viewer/assets/data/vehicle/MAN/meshes/*_chassis_col.obj")])
    environment = dict(platform=platform.platform(), cpu=cpu, logical_cpus=logical,
                       compiler=command("g++", "--version").splitlines()[0], build="Release (-O3 -DNDEBUG)",
                       gpu=gpu_name, power_plan=args.power_plan, cpu_electrical_power=None,
                       binary_sha256={k: digest(getattr(args, k)) for k in ("clean", "instrumented")},
                       source_sha256={str(p.relative_to(root)): digest(p) for p in source})
    samples = []
    sampler = subprocess.Popen([gpu, "-i", "0", "--query-gpu=power.draw,utilization.gpu,temperature.gpu",
                                "--format=csv,noheader,nounits", "-lms", "200"],
                               stdout=subprocess.PIPE, text=True)

    def read_gpu():
        for line in sampler.stdout:
            try:
                values = tuple(map(float, line.split(",")))
            except ValueError:
                continue
            if len(values) == 3:
                samples.append((time.monotonic(), *values))

    reader = threading.Thread(target=read_gpu)
    reader.start()
    with tempfile.TemporaryDirectory(prefix="fusionforce-man10t-") as cache:
        common = [str(root / "viewer"), str(args.ply.resolve()), str(routes.resolve()), cache]
        try:
            with subprocess.Popen([str(args.clean.resolve()), *common], stdout=subprocess.PIPE, text=True) as proc:
                header = json.loads(proc.stdout.readline())
                clean = []
                for line in proc.stdout:
                    row = json.loads(line)
                    clean.append(row)
                    print(f'{row["id"]} clean {row["wall_s"]:.3f} s', flush=True)
                if proc.wait() != 0 or len(clean) != 10:
                    raise RuntimeError("Incomplete clean run")
        finally:
            sampler.terminate()
            sampler.wait()
            reader.join()
        idle = [s[1] for s in samples if header["idle_start_s"] <= s[0] <= header["idle_end_s"]]
        by_id = {row["id"]: row for row in baseline["clean"]}
        for row in clean:
            annotate_gpu(row, samples)
            row["cpu_cores"] = row["cpu_s"] / row["wall_s"]
            row["cpu_pct"] = row["cpu_cores"] * 100 / logical
            original = by_id[row["id"]]
            row["max_xy_error_m"] = max_error(row["xy"], original["xy"])
            row["same_decisions"] = all(row[k] == original[k] for k in ("status", "samples", "warnings"))
        metrics = dict(environment=environment, **header, clean=clean,
                       idle_gpu_mean_w=statistics.mean(idle) if idle else None, idle_gpu_n=len(idle))
        metrics_file = out / "metrics.json"
        metrics_file.write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
        with (out / "gpu.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["monotonic_s", "gpu_w", "gpu_busy_pct", "gpu_c"])
            writer.writerows(samples)
        profiled = []
        profiles = []
        for row in clean:
            route = row["id"]
            with tempfile.TemporaryDirectory(dir=cache) as profile_run:
                result = json.loads(subprocess.check_output(
                    [str(args.instrumented.resolve()), *common, route], cwd=profile_run, text=True))
                target = profile_dir / f"{route}.gmon"
                shutil.move(Path(profile_run) / "gmon.out", target)
                profiles.append(str(target))
            error = max_error(row["xy"], result["xy"])
            if error > 1e-6 or any(row[k] != result[k] for k in ("status", "samples", "warnings")):
                raise RuntimeError(f"{route} profile does not match clean run")
            profiled.append(dict(id=route, wall_s=result["wall_s"], cpu_s=result["cpu_s"], max_xy_error_m=error))
            print(f'{route} gprof {result["wall_s"]:.3f} s', flush=True)
        with (out / "gprof.txt").open("w", encoding="utf-8") as stream:
            subprocess.run(["gprof", "-b", "-p", str(args.instrumented.resolve()), *profiles],
                           check=True, stdout=stream)
        metrics["profiled"] = profiled
        metrics_file.write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
