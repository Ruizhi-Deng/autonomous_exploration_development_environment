#!/usr/bin/env python3
"""
ros2_log_prettify.py

用法示例:
  ./ros2_log_prettify.py "ros2 launch simulator real.lbl1f.grid_hpp_pv.launch.py"

功能:
  - 以子进程运行给定的命令（常用于 ros2 命令）
  - 实时捕获 stdout/stderr，解析并简化 ROS 2 日志行（如 [INFO] [node]: message）
  - 颜色化并按重要性过滤日志（可通过 --min-severity 指定）
  - 消除连续重复行，显示重复计数

注意: 命令参数作为一个整体字符串传入（脚本内部使用 shell=True 来执行），请确保信任该字符串。
"""

from __future__ import annotations
import argparse
import shlex
import subprocess
import re
import sys
import signal
import os
from typing import Optional

SEVERITY_ORDER = {
    "DEBUG": 0,
    "INFO": 1,
    "WARN": 2,
    "ERROR": 3,
    "FATAL": 4,
}

ANSI = {
    "RESET": "\033[0m",
    "DIM": "\033[2m",
    "BOLD": "\033[1m",
    "DEBUG": "\033[36m",   # cyan
    "INFO": "\033[32m",    # green
    "WARN": "\033[33m",    # yellow
    "ERROR": "\033[31m",   # red
    "FATAL": "\033[41;97m", # white on red
}

LOG_RE = re.compile(r"\[(DEBUG|INFO|WARN|ERROR|FATAL)\](?:\s*\[([^\]]+)\])?\s*\[([^\]]+)\]:\s*(.*)")
# fallback: detect severity anywhere and node in following [node]: pattern
SEV_ANY_RE = re.compile(r"\[(DEBUG|INFO|WARN|ERROR|FATAL)\]")
NODE_RE = re.compile(r"\[([^\]]+)\]:\s*(.*)")


def _forward_signal_to_child_group(proc: subprocess.Popen, sig: int) -> None:
    """Best-effort forward of signal to the child's process group."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except Exception:
        pass


def _graceful_stop_child(proc: subprocess.Popen) -> None:
    """Stop child process group with escalation: SIGINT -> SIGTERM -> SIGKILL."""
    if proc.poll() is not None:
        return

    _forward_signal_to_child_group(proc, signal.SIGINT)
    try:
        proc.wait(timeout=5)
        return
    except Exception:
        pass

    _forward_signal_to_child_group(proc, signal.SIGTERM)
    try:
        proc.wait(timeout=3)
        return
    except Exception:
        pass

    _forward_signal_to_child_group(proc, signal.SIGKILL)
    try:
        proc.wait(timeout=2)
    except Exception:
        pass


def running_in_wsl() -> bool:
    """Return True when running inside Windows Subsystem for Linux."""
    if "WSL_INTEROP" in os.environ or "WSL_DISTRO_NAME" in os.environ:
        return True
    try:
        with open("/proc/version", "r", encoding="utf-8") as f:
            return "microsoft" in f.read().lower()
    except Exception:
        return False


def cleanup_stale_runtime(plain: bool = False):
    """Best-effort cleanup for stale Gazebo/ROS simulator processes and Fast DDS SHM files."""
    patterns = [
        r"gzserver",
        r"gzclient",
        r"spawn_entity.py",
        r"vehicle_simulator/lib/vehicle_simulator/vehicleSimulator",
        r"ros2 launch .*vehicle_simulator/launch/system_.*\\.launch",
    ]

    for pattern in patterns:
        try:
            subprocess.run(
                ["pkill", "-9", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        except Exception:
            pass

    removed_shm = 0
    shm_dir = "/dev/shm"
    if os.path.isdir(shm_dir):
        for name in os.listdir(shm_dir):
            if not (
                name.startswith("fastrtps_")
                or name.startswith("fastdds_")
                or name.startswith("dds_shm_")
            ):
                continue
            path = os.path.join(shm_dir, name)
            try:
                if os.path.isfile(path):
                    os.remove(path)
                    removed_shm += 1
            except Exception:
                pass

    msg = f"[launcher] Cleanup done: stale processes signaled, removed {removed_shm} SHM files."
    print(f"{ANSI['DIM']}{msg}{ANSI['RESET']}" if not plain else msg)


def parse_line(line: str):
    """解析一行输出，返回 (severity, node, message) 或 (None, None, raw)"""
    m = LOG_RE.search(line)
    if m:
        sev, ts, node, msg = m.group(1), m.group(2), m.group(3), m.group(4)
        return sev, node, msg
    # try looser matching
    m2 = SEV_ANY_RE.search(line)
    if m2:
        sev = m2.group(1)
        mnode = NODE_RE.search(line)
        if mnode:
            node, msg = mnode.group(1), mnode.group(2)
            return sev, node, msg
        # no node, just severity
        # take rest of line after severity
        after = line.split(m2.group(0), 1)[1].strip()
        return sev, None, after
    # fallback: not a recognized ROS log line
    return None, None, line.rstrip()


def colorize(sev: Optional[str], node: Optional[str], msg: str, plain=False):
    """Format a log line: bold the node name, remove colon, and separate node and message with a tab."""
    if sev is None:
        # Not a ROS log line
        return msg
    color = ANSI.get(sev, "")
    reset = ANSI["RESET"]
    if node:
        # Bold node name (unless plain mode), no trailing colon, use a tab for alignment
        node_fmt = node if plain else f"{ANSI['BOLD']}{node}{ANSI['RESET']}"
        return f"{color}[{sev}]{reset} {node_fmt}\t{msg}"
    else:
        return f"{color}[{sev}]{reset} {msg}"


def main():
    parser = argparse.ArgumentParser(description="Run a command and prettify ROS2 logs in real time.")
    parser.add_argument("command", help="Launch command or launch file path. Only 'ros2 launch <package> <launch-file>' invocations or files ending with '.launch.py' or '.launch.xml' are accepted.")
    parser.add_argument("--min-severity", choices=SEVERITY_ORDER.keys(), default="DEBUG",
                        help="Minimum severity to display (default: DEBUG)")
    parser.add_argument("--plain", action="store_true", help="Do not use colors")
    parser.add_argument("--show-raw", action="store_true", help="Also print non-ROS lines in raw form")
    parser.add_argument("--quiet", action="store_true", help="Print only lines that match ROS log format")
    parser.add_argument("--cleanup", action="store_true", help="Clean stale Gazebo/ROS processes and Fast DDS SHM files before launch")
    args = parser.parse_args()

    min_level = SEVERITY_ORDER[args.min_severity]

    proc: Optional[subprocess.Popen] = None
    interrupted = False
    signal_count = 0

    def _handle_signal(signum, frame):
        nonlocal interrupted, proc, signal_count
        signal_count += 1
        interrupted = True
        if proc is not None:
            # 1st Ctrl+C: graceful stop request; repeated Ctrl+C: hard stop request.
            if signal_count == 1:
                _forward_signal_to_child_group(proc, signum)
            else:
                _forward_signal_to_child_group(proc, signal.SIGKILL)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # Validate the provided command: accept only ros2 launch invocations with a launch file
    raw_cmd = args.command.strip()

    def is_launch_string(s: str) -> bool:
        return s.endswith('.launch.py') or s.endswith('.launch.xml') or '.launch' in s

    command_to_run = None
    if raw_cmd.startswith('ros2 launch'):
        # parse tokens and ensure the launch file token looks like a launch file
        tokens = shlex.split(raw_cmd)
        if len(tokens) < 3:
            print("Error: 'ros2 launch' must include package and launch file.", file=sys.stderr)
            sys.exit(2)
        # require the launch token to contain '.launch' or be a path ending with .launch.py/.launch.xml
        if not is_launch_string(tokens[2]):
            print("Error: 'ros2 launch' invocation must reference a launch file (e.g., ends with .launch.py or contains '.launch').", file=sys.stderr)
            sys.exit(2)
        command_to_run = raw_cmd
    else:
        # treat input as a path to a launch file
        path = os.path.expanduser(raw_cmd)
        if os.path.isfile(path) and (path.endswith('.launch.py') or path.endswith('.launch.xml') or path.endswith('.launch')):
            # run as 'ros2 launch <path>'
            command_to_run = f"ros2 launch {shlex.quote(path)}"
        else:
            print("Error: This tool accepts only launch files or 'ros2 launch' invocations that reference a launch file.", file=sys.stderr)
            sys.exit(2)

    if args.cleanup:
        cleanup_stale_runtime(plain=args.plain)

    # Run command without an extra shell layer for more reliable signal handling.
    print(f"Running: {command_to_run}")

    launch_env = os.environ.copy()
    if running_in_wsl() and "FASTDDS_BUILTIN_TRANSPORTS" not in launch_env:
        launch_env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
        print(f"{ANSI['DIM']}[launcher] WSL detected: set FASTDDS_BUILTIN_TRANSPORTS=UDPv4 to avoid SHM lock issues.{ANSI['RESET']}" if not args.plain else "[launcher] WSL detected: set FASTDDS_BUILTIN_TRANSPORTS=UDPv4 to avoid SHM lock issues.")

    try:
        proc = subprocess.Popen(
            shlex.split(command_to_run),
            shell=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True,
            start_new_session=True,
            env=launch_env,
        )
    except Exception as e:
        print(f"Failed to start command: {e}", file=sys.stderr)
        sys.exit(2)

    prev_line = None
    repeat_count = 0
    try:
        assert proc.stdout is not None
        for raw in proc.stdout:
            # Normalize newline trimmed
            line = raw.rstrip("\n")
            sev, node, msg = parse_line(line)
            if sev is not None:
                if SEVERITY_ORDER.get(sev, 0) < min_level:
                    # severity too low, skip
                    continue
                pretty = colorize(sev, node, msg, plain=args.plain)
                out_line = pretty
            else:
                if args.quiet:
                    continue
                if not args.show_raw:
                    # print dimmed raw line
                    out_line = f"{ANSI['DIM']}{line}{ANSI['RESET']}" if not args.plain else line
                else:
                    out_line = line

            # simple dedup: if same as previous printed, increment counter
            if out_line == prev_line:
                repeat_count += 1
                continue
            else:
                # flush pending repeat count
                if repeat_count > 0:
                    print(f"{ANSI['DIM']}... (repeated {repeat_count} times){ANSI['RESET']}" if not args.plain else f"... (repeated {repeat_count} times)")
                    repeat_count = 0
                print(out_line)
                prev_line = out_line

        # finished streaming, flush any repeats
        if repeat_count > 0:
            print(f"{ANSI['DIM']}... (repeated {repeat_count} times){ANSI['RESET']}" if not args.plain else f"... (repeated {repeat_count} times)")

        proc.wait()
        rc = proc.returncode
        if rc != 0:
            print(f"Command exited with code {rc}", file=sys.stderr)
        sys.exit(rc)

    except KeyboardInterrupt:
        interrupted = True
    finally:
        if repeat_count > 0:
            print(f"{ANSI['DIM']}... (repeated {repeat_count} times){ANSI['RESET']}" if not args.plain else f"... (repeated {repeat_count} times)")

        if proc is not None:
            try:
                _graceful_stop_child(proc)
            except KeyboardInterrupt:
                # If user presses Ctrl+C again during teardown, force kill immediately.
                _forward_signal_to_child_group(proc, signal.SIGKILL)
                try:
                    proc.wait(timeout=1)
                except Exception:
                    pass

        if interrupted:
            sys.exit(130)


if __name__ == "__main__":
    main()
