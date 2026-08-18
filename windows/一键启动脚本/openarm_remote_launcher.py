#!/usr/bin/env python3
import argparse
import getpass
import queue
import shlex
import sys
import threading
import time

import paramiko


ROS = "source /opt/ros/humble/setup.bash; source /home/nuc/ros2_ws/install/setup.bash"
DISPLAY = (
    "export DISPLAY=:0; "
    "export XAUTHORITY=$(find /run/user/1000 -maxdepth 1 "
    "-name '.mutter-Xwaylandauth.*' -print -quit); "
    "if [ -z \"$XAUTHORITY\" ]; then export XAUTHORITY=/home/nuc/.Xauthority; fi; "
    "export QT_QPA_PLATFORM=xcb"
)


def connect(args):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        args.host,
        username=args.user,
        password=args.password,
        timeout=10,
        banner_timeout=10,
        auth_timeout=10,
    )
    return client


def run_once(args, command, timeout=30):
    client = connect(args)
    try:
        wrapped = "bash -lc " + shlex.quote(command)
        _, stdout, stderr = client.exec_command(wrapped, timeout=timeout)
        code = stdout.channel.recv_exit_status()
        output = stdout.read().decode("utf-8", "replace")
        error = stderr.read().decode("utf-8", "replace")
        return code, output, error
    finally:
        client.close()


def stream_process(args, name, command, events):
    client = None
    try:
        client = connect(args)
        wrapped = "bash -lc " + shlex.quote(command)
        transport = client.get_transport()
        channel = transport.open_session()
        channel.get_pty(term="xterm", width=160, height=40)
        channel.exec_command(wrapped)
        events.put((name, "STARTED"))
        buffer = b""
        while True:
            if channel.recv_ready():
                buffer += channel.recv(4096)
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    text = line.decode("utf-8", "replace").rstrip("\r")
                    if text:
                        events.put((name, text))
            if channel.recv_stderr_ready():
                text = channel.recv_stderr(4096).decode("utf-8", "replace").strip()
                if text:
                    events.put((name, text))
            if channel.exit_status_ready():
                if buffer:
                    events.put((name, buffer.decode("utf-8", "replace").strip()))
                events.put((name, "EXIT=" + str(channel.recv_exit_status())))
                break
            time.sleep(0.05)
    except Exception as exc:
        events.put((name, "ERROR: " + str(exc)))
    finally:
        if client is not None:
            client.close()


def start(args, name, command, events):
    thread = threading.Thread(
        target=stream_process,
        args=(args, name, command, events),
        daemon=True,
    )
    thread.start()
    return thread


def wait_until(args, command, timeout, label):
    deadline = time.time() + timeout
    last_error = ""
    while time.time() < deadline:
        try:
            code, _, error = run_once(args, command, timeout=8)
            if code == 0:
                print(label + " ready")
                return
            last_error = error.strip()
        except Exception as exc:
            last_error = str(exc)
        time.sleep(2)
    raise RuntimeError(label + " did not become ready: " + last_error)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="172.16.13.202")
    parser.add_argument("--user", default="nuc")
    parser.add_argument(
        "--password",
        default=None,
        help="SSH密码；不提供时运行中安全提示输入，不写入命令行或仓库",
    )
    parser.add_argument("--windows-ip", default="172.16.13.1")
    parser.add_argument("--real-motion", action="store_true")
    parser.add_argument("--no-clean", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    if args.password is None:
        args.password = getpass.getpass(f"请输入 {args.user}@{args.host} 的SSH密码: ")

    print("Connecting to NUC " + args.host + " as " + args.user)
    code, output, error = run_once(args, "hostname; test -f /opt/ros/humble/setup.bash")
    if code != 0:
        raise RuntimeError("NUC preflight failed: " + error)
    print("NUC connected: " + output.splitlines()[0])
    if args.check_only:
        print("SSH password authentication and ROS preflight passed.")
        return

    health = "curl -fsS --max-time 5 http://" + args.windows_ip + ":8765/health"
    code, output, error = run_once(args, health, timeout=10)
    if code != 0:
        raise RuntimeError(
            "NUC cannot access YOLOE at " + args.windows_ip + ":8765: " + error.strip()
        )
    print("YOLOE is reachable from NUC: " + output.strip())

    if not args.no_clean:
        cleanup = (
            "pkill -INT -f '[s]emantic_grasp.launch.py' || true; "
            "pkill -INT -f '[s]emantic_step_grasp' || true; "
            "pkill -INT -f '[d]ual_realsense_grasp.launch.py' || true; "
            "pkill -INT -f '[d]emo.launch.py' || true; "
            "pkill -INT -f '[s]emantic_confirm_ui' || true; "
            "pkill -INT -f '[r]qt_image_view' || true; sleep 6; "
            "pkill -TERM -f '[s]emantic_grasp.launch.py' || true; "
            "pkill -TERM -f '[s]emantic_step_grasp' || true; "
            "pkill -TERM -f '[d]ual_realsense_grasp.launch.py' || true; "
            "pkill -TERM -f '[d]emo.launch.py' || true; "
            "pkill -TERM -f '[r]os2 run openarm_vision_grasp semantic_confirm_ui' || true; "
            "pkill -TERM -f '[r]os2 run rqt_image_view' || true; "
            "pkill -TERM -f '[m]oveit_ros_move_group/move_group' || true; "
            "pkill -TERM -f '[c]ontroller_manager/ros2_control_node' || true; "
            "pkill -TERM -f '[r]obot_state_publisher' || true; "
            "pkill -TERM -f '[o]penarm_vision_grasp/lib/openarm_vision_grasp' || true; "
            "pkill -TERM -f '[r]ealsense2_camera_node' || true; "
            "pkill -TERM -f '[s]tatic_transform_publisher.*head_d435i' || true; "
            "pkill -TERM -f '[s]tatic_transform_publisher.*left_wrist_d415' || true; "
            "sleep 4; "
            "pkill -KILL -f '[s]emantic_grasp.launch.py|[d]ual_realsense_grasp.launch.py|[d]emo.launch.py' || true; "
            "pkill -KILL -f '[s]emantic_step_grasp' || true; "
            "pkill -KILL -f '[m]oveit_ros_move_group/move_group|[c]ontroller_manager/ros2_control_node' || true; "
            "pkill -KILL -f '[r]obot_state_publisher' || true; "
            "pkill -KILL -f '[o]penarm_vision_grasp/lib/openarm_vision_grasp|[r]ealsense2_camera_node' || true; "
            "sleep 3; "
            "if pgrep -f '[s]emantic_grasp.launch.py|[s]emantic_step_grasp|[d]ual_realsense_grasp.launch.py|[d]emo.launch.py|[c]ontroller_manager/ros2_control_node|[m]oveit_ros_move_group/move_group|[o]penarm_vision_grasp/lib/openarm_vision_grasp' >/dev/null; then exit 20; fi"
        )
        code, _, error = run_once(args, cleanup, timeout=25)
        if code != 0:
            raise RuntimeError("Old MoveIt processes could not be stopped: " + error)

    events = queue.Queue()
    threads = []
    motion = "true" if args.real_motion else "false"

    moveit = (
        ROS + "; sudo -n -v || true; ~/init_robot.sh; " + DISPLAY + "; "
        "ros2 launch openarm_bimanual_moveit_config demo.launch.py "
        "hardware_type:=real right_can_interface:=can0 left_can_interface:=can1"
    )
    threads.append(start(args, "MOVEIT", moveit, events))
    wait_until(
        args,
        ROS + "; ros2 node list 2>/dev/null | grep -qx /move_group && "
        "rate=$(timeout 4 ros2 topic hz /joint_states 2>/dev/null | "
        "awk '/average rate:/ {print $3; exit}'); "
        "awk -v rate=\"$rate\" 'BEGIN {exit !(rate >= 20.0)}'",
        50,
        "MoveIt and joint states",
    )

    cameras = ROS + "; ros2 launch openarm_vision_grasp dual_realsense_grasp.launch.py"
    threads.append(start(args, "CAMERAS", cameras, events))
    wait_until(
        args,
        ROS + "; timeout 4 ros2 topic echo --once "
        "/openarm_vision/head_d435i/color/camera_info >/dev/null 2>&1",
        35,
        "Head camera",
    )

    backend = (
        ROS + "; export PYTHONNOUSERSITE=1; "
        "ros2 launch openarm_vision_grasp semantic_grasp.launch.py "
        "start_cameras:=false show_ui:=false allow_motion:=" + motion
    )
    threads.append(start(args, "SEMANTIC", backend, events))
    wait_until(
        args,
        ROS + "; ros2 node list 2>/dev/null | grep -qx /openarm_semantic_grasp_task",
        30,
        "Semantic grasp server",
    )
    code, output, error = run_once(
        args,
        ROS + "; if ros2 node list 2>/dev/null | grep -qx /openarm_semantic_step_grasp; then exit 21; fi",
        timeout=10,
    )
    if code != 0:
        raise RuntimeError("旧 semantic_step_grasp 节点仍在运行，拒绝启动以避免重复控制夹爪。")

    viewer = (
        ROS + "; " + DISPLAY + "; "
        "ros2 run rqt_image_view rqt_image_view /openarm_vision/head_yoloe_overlay"
    )
    ui = (
        ROS + "; export PYTHONNOUSERSITE=1; " + DISPLAY + "; "
        "ros2 run openarm_vision_grasp semantic_confirm_ui"
    )
    threads.append(start(args, "VIEWER", viewer, events))
    threads.append(start(args, "UI", ui, events))

    # 把关键 ROS 话题直接汇总到 Windows 启动窗口，避免用户再开多个 SSH 终端查日志。
    status_log = (
        ROS + "; ros2 topic echo --full-length "
        "/openarm_vision/semantic_status"
    )
    depth_log = (
        ROS + "; ros2 topic echo "
        "/openarm_vision/wrist_depth_distance"
    )
    error_log = (
        ROS + "; ros2 topic echo /rosout | "
        "grep --line-buffered -E "
        "'semantic_grasp_task|GRIPPER|APPROACH|WRIST|PLAN_FAILED|失败|异常|超时'"
    )
    threads.append(start(args, "STATUS", status_log, events))
    threads.append(start(args, "WRIST_DEPTH", depth_log, events))
    threads.append(start(args, "ERROR_LOG", error_log, events))

    print("Mode: " + ("REAL MOTION" if args.real_motion else "PREVIEW ONLY"))
    print("All remote processes were started. Keep this window open.")
    print("Logs: STATUS=semantic task, WRIST_DEPTH=D415 distance, ERROR_LOG=抓取错误")
    try:
        while True:
            try:
                name, message = events.get(timeout=1.0)
                print("[" + name + "] " + message, flush=True)
            except queue.Empty:
                if not any(thread.is_alive() for thread in threads):
                    break
    except KeyboardInterrupt:
        print("Stopping monitor. Remote ROS launch processes will receive SSH disconnect.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("STARTUP FAILED: " + str(exc), file=sys.stderr, flush=True)
        print("No semantic grasp UI was started. Real motion remains unavailable.",
              file=sys.stderr, flush=True)
        sys.exit(1)
