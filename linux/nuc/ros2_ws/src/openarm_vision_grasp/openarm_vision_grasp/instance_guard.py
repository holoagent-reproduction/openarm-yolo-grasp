"""新视觉包的单实例保护，避免重复启动节点互相覆盖ROS话题。"""
import fcntl
import os
import sys

_locks = []


def acquire_singleton(name):
    path = f"/tmp/openarm_vision_grasp_{name}.lock"
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(f"[openarm_vision_grasp] 已有 {name} 实例运行，本实例退出。", flush=True)
        os.close(fd)
        return False
    os.write(fd, f"pid={os.getpid()}\n".encode())
    _locks.append(fd)
    return True

