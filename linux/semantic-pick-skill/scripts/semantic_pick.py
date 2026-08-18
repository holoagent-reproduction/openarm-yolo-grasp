#!/usr/bin/env python3
"""调用 OpenArm Semantic Pick Skill HTTP 接口。"""
import argparse
import json
import urllib.request


def request(url, method="GET", payload=None):
    body = None if payload is None else json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(url, data=body, method=method, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as response:
        return json.loads(response.read().decode("utf-8"))


def main():
    parser = argparse.ArgumentParser(description="OpenArm 语义抓取 Skill 客户端")
    parser.add_argument("--url", default="http://127.0.0.1:8780")
    sub = parser.add_subparsers(dest="command", required=True)
    start = sub.add_parser("start"); start.add_argument("instruction"); start.add_argument("--execute", action="store_true")
    status = sub.add_parser("status"); status.add_argument("task_id")
    confirm = sub.add_parser("confirm"); confirm.add_argument("task_id")
    cancel = sub.add_parser("cancel"); cancel.add_argument("task_id")
    args = parser.parse_args(); root = args.url.rstrip("/")
    if args.command == "start":
        result = request(f"{root}/skills/semantic_pick", "POST",
                         {"instruction": args.instruction, "preview_only": not args.execute})
    elif args.command == "status":
        result = request(f"{root}/skills/semantic_pick/{args.task_id}")
    else:
        result = request(f"{root}/skills/semantic_pick/{args.task_id}/{args.command}", "POST", {})
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
