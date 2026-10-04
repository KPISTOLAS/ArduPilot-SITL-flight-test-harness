"""Start and stop ArduPilot SITL.

Backends (auto preference order):
1. sim_vehicle / local arducopter binary when ~/ardupilot or ARDUPILOT_HOME exists
2. dronekit-sitl (old Copter 3.3 — OK for smoke tests, flaky for missions)
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import harness.compat  # noqa: F401


def find_ardupilot_home() -> Optional[str]:
    """Return an ArduPilot checkout path if available."""
    candidates = []
    env = os.environ.get("ARDUPILOT_HOME")
    if env:
        candidates.append(env)
    candidates.append(str(Path.home() / "ardupilot"))
    for path in candidates:
        sim_vehicle = Path(path) / "Tools" / "autotest" / "sim_vehicle.py"
        if sim_vehicle.is_file():
            return str(Path(path).resolve())
    return None


@dataclass
class SitlConfig:
    vehicle: str = "copter"  # copter | plane | rover
    speedup: float = 1.0
    home: str = "-35.363261,149.165230,584,353"
    mavlink_port: int = 5760
    # "dronekit" | "sim_vehicle" | "auto"
    backend: str = "auto"
    extra_args: Optional[List[str]] = None
    ready_timeout_s: float = 90.0
    cleanup_stale: bool = True


def kill_stale_sitl() -> None:
    """Best-effort cleanup of leftover SITL binaries holding TCP 5760."""
    patterns = (
        "build/sitl/bin/arducopter",
        "bin/arducopter",
        "arducopter --model",
        "sitl-linux-copter",
        "Tools/autotest/sim_vehicle.py",
    )
    for pat in patterns:
        try:
            subprocess.run(
                ["pkill", "-f", pat],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            break
    time.sleep(1.0)


def wait_for_listen(port: int, timeout_s: float = 90.0) -> None:
    """Wait until TCP port is LISTENING (do not connect — SITL is single-client)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            import psutil

            for conn in psutil.net_connections(kind="tcp"):
                if (
                    conn.laddr
                    and conn.status == psutil.CONN_LISTEN
                    and int(conn.laddr.port) == int(port)
                ):
                    return
        except Exception:
            # Fallback: ss
            try:
                out = subprocess.check_output(
                    ["ss", "-ltn"], text=True, stderr=subprocess.DEVNULL
                )
                if f":{port} " in out or f":{port}\n" in out:
                    return
            except Exception:
                pass
        time.sleep(0.5)
    raise TimeoutError(f"SITL TCP port {port} not listening after {timeout_s}s")


class SitlInstance:
    """Context-manager wrapper around a SITL process."""

    def __init__(self, config: SitlConfig | None = None) -> None:
        self.config = config or SitlConfig()
        self._proc: subprocess.Popen | None = None
        self._dk_sitl = None
        self._backend_used: str | None = None
        self._log_file = None

    def start(self) -> None:
        if self._proc is not None or self._dk_sitl is not None:
            return

        if self.config.cleanup_stale:
            kill_stale_sitl()

        backend = self._resolve_backend()
        self._backend_used = backend
        print(f"[sitl] backend={backend}")

        if backend == "dronekit":
            self._start_dronekit_sitl()
        else:
            self._start_modern_sitl()
            wait_for_listen(self.config.mavlink_port, self.config.ready_timeout_s)
            time.sleep(2.0)

    def stop(self) -> None:
        if self._dk_sitl is not None:
            try:
                self._dk_sitl.stop()
            except Exception:
                pass
            self._dk_sitl = None

        if self._proc is not None:
            self._terminate_process_tree(self._proc)
            self._proc = None

        if self._log_file is not None:
            try:
                self._log_file.close()
            except Exception:
                pass
            self._log_file = None

        if self.config.cleanup_stale:
            kill_stale_sitl()

    @property
    def connection_string(self) -> str:
        if self._dk_sitl is not None:
            return self._dk_sitl.connection_string()
        return f"tcp:127.0.0.1:{self.config.mavlink_port}"

    def __enter__(self) -> "SitlInstance":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    def _resolve_backend(self) -> str:
        choice = (self.config.backend or "auto").lower()
        if choice == "auto":
            if find_ardupilot_home():
                return "sim_vehicle"
            return "dronekit"
        if choice not in ("dronekit", "sim_vehicle"):
            raise ValueError(f"Unknown SITL backend: {choice}")
        return choice

    def _start_dronekit_sitl(self) -> None:
        try:
            import dronekit_sitl
        except ImportError as exc:
            raise RuntimeError(
                "dronekit-sitl is not installed. Run: pip install dronekit-sitl"
            ) from exc

        vehicle_map = {"copter": "copter", "plane": "plane", "rover": "rover"}
        system = vehicle_map.get(self.config.vehicle.lower())
        if system is None:
            raise ValueError(f"Unsupported vehicle for dronekit-sitl: {self.config.vehicle}")

        sitl = dronekit_sitl.SITL()
        sitl.download(system, "3.3", verbose=True)
        model = "quad" if system == "copter" else system
        args = ["--model", model, "--home", self.config.home]
        if self.config.speedup and self.config.speedup != 1.0:
            args += ["--speedup", str(int(self.config.speedup))]
        sitl.launch(args, await_ready=True, restart=True, verbose=True)
        self._dk_sitl = sitl
        try:
            cs = sitl.connection_string()
            self.config.mavlink_port = int(cs.rsplit(":", 1)[-1])
        except Exception:
            pass
        time.sleep(2.0)

    def _start_modern_sitl(self) -> None:
        ap_home = find_ardupilot_home()
        if not ap_home:
            raise RuntimeError(
                "No ArduPilot tree found. Set ARDUPILOT_HOME or install dronekit-sitl."
            )

        vehicle_map = {
            "copter": ("ArduCopter", "arducopter"),
            "plane": ("ArduPlane", "arduplane"),
            "rover": ("Rover", "ardurover"),
        }
        mapped = vehicle_map.get(self.config.vehicle.lower())
        if mapped is None:
            raise ValueError(f"Unsupported vehicle: {self.config.vehicle}")
        vehicle_name, binary_name = mapped

        binary = Path(ap_home) / "build" / "sitl" / "bin" / binary_name
        # Prefer project-local logs/ when cwd is the repo; otherwise /tmp.
        repo_logs = Path(__file__).resolve().parents[1] / "logs"
        log_dir = repo_logs if repo_logs.parent.is_dir() else Path("/tmp")
        log_path = log_dir / "sitl_stdout.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = open(log_path, "w", encoding="utf-8")

        if binary.is_file():
            # Fast path: run the built SITL binary directly.
            cmd = [
                str(binary),
                "--model",
                "+" if self.config.vehicle.lower() == "copter" else self.config.vehicle.lower(),
                "--speedup",
                str(self.config.speedup),
                "--home",
                self.config.home,
                "-I",
                "0",
            ]
            if self.config.extra_args:
                cmd.extend(self.config.extra_args)
            print(f"[sitl] launching binary: {' '.join(cmd)}")
            self._proc = subprocess.Popen(
                cmd,
                cwd=ap_home,
                stdout=self._log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            return

        # Slower path: sim_vehicle.py (can rebuild if needed)
        sim_vehicle = Path(ap_home) / "Tools" / "autotest" / "sim_vehicle.py"
        cmd = [
            "python3",
            str(sim_vehicle),
            "-v",
            vehicle_name,
            "--no-mavproxy",
            "--no-rebuild",
            "-l",
            self.config.home,
            "--speedup",
            str(self.config.speedup),
            "-I",
            "0",
        ]
        if self.config.extra_args:
            cmd.extend(self.config.extra_args)
        print(f"[sitl] launching sim_vehicle: {' '.join(cmd)}")
        self._proc = subprocess.Popen(
            cmd,
            cwd=ap_home,
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    @staticmethod
    def _terminate_process_tree(proc: subprocess.Popen, timeout_s: float = 5.0) -> None:
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, AttributeError):
            proc.terminate()
        try:
            proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError, AttributeError):
                proc.kill()
            proc.wait(timeout=2.0)
