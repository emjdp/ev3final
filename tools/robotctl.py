#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EV3 튜닝 서버에 한 번 연결해 명령 하나를 보내는 비대화형 CLI.

와이어 프로토콜: newline-delimited JSON, 요청 1줄 -> 응답 1줄.
명세: docs/specs/00_infra_dashboard.md 6.1, 6.2
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
from typing import Any


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_TIMEOUT = 3.0


class RobotctlError(Exception):
    """사람에게 그대로 보여줄 수 있는 CLI 오류."""


def parse_json_value(text: str) -> Any:
    """가능하면 JSON 타입으로, 아니면 문자열로 해석한다."""

    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return text


def parse_key_values(items: list[str]) -> dict[str, Any]:
    args: dict[str, Any] = {}
    for item in items:
        if "=" not in item:
            raise RobotctlError("do 인자는 k=v 형식이어야 합니다: {}".format(item))
        key, value = item.split("=", 1)
        if not key:
            raise RobotctlError("do 인자의 키가 비어 있습니다: {}".format(item))
        args[key] = parse_json_value(value)
    return args


def send_request(
    request: dict[str, Any],
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    timeout: float = DEFAULT_TIMEOUT,
) -> dict[str, Any]:
    """튜닝 서버에 요청 하나를 보내고 응답 객체 하나를 돌려준다.

    목 서버/데모 서버 테스트가 쉽도록 CLI 출력과 분리된 순수 I/O 함수로 둔다.
    """

    line = json.dumps(request, ensure_ascii=False, separators=(",", ":")) + "\n"
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall(line.encode("utf-8"))
            with sock.makefile("rb") as reader:
                raw = reader.readline()
    except ConnectionRefusedError as exc:
        raise RobotctlError(
            "연결 실패: {}:{} 에서 서버가 거부했습니다. "
            "브릭 stage 실행과 SSH 터널을 확인하세요. ({})".format(host, port, exc)
        ) from exc
    except socket.timeout as exc:
        raise RobotctlError(
            "타임아웃: {}:{} 에서 {:.1f}초 안에 응답이 없습니다.".format(
                host, port, timeout
            )
        ) from exc
    except OSError as exc:
        raise RobotctlError(
            "연결 실패: {}:{} 에 연결할 수 없습니다. ({})".format(host, port, exc)
        ) from exc

    if not raw:
        raise RobotctlError("응답 없음: 서버가 연결을 닫았습니다.")

    try:
        response = json.loads(raw.decode("utf-8"))
    except UnicodeDecodeError as exc:
        raise RobotctlError("응답 디코딩 실패: UTF-8 이 아닙니다.") from exc
    except json.JSONDecodeError as exc:
        raise RobotctlError("응답 파싱 실패: JSON 한 줄이 아닙니다. ({})".format(exc)) from exc

    if not isinstance(response, dict):
        raise RobotctlError("응답 형식 오류: JSON 객체가 아닙니다.")
    if "ok" not in response:
        raise RobotctlError("응답 형식 오류: ok 필드가 없습니다.")
    return response


def build_request(args: argparse.Namespace) -> dict[str, Any]:
    if args.command == "get":
        request: dict[str, Any] = {"cmd": "get"}
        if args.name is not None:
            request["name"] = args.name
        return request
    if args.command == "set":
        return {"cmd": "set", "name": args.name, "value": parse_json_value(args.value)}
    if args.command == "stop":
        return {"cmd": "stop", "source": "network"}
    if args.command == "pause":
        return {"cmd": "pause", "paused": True, "source": "network"}
    if args.command == "resume":
        return {"cmd": "pause", "paused": False, "source": "network"}
    if args.command == "do":
        return {
            "cmd": "do",
            "action": args.action,
            "args": parse_key_values(args.kv),
        }
    if args.command == "save":
        return {"cmd": "save"}
    if args.command == "rollback":
        return {"cmd": "rollback"}
    if args.command == "latest":
        return {"cmd": "get_latest"}
    if args.command == "describe":
        return {"cmd": "describe"}
    raise RobotctlError("알 수 없는 명령입니다: {}".format(args.command))


def json_pretty(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True)


def format_scalar(value: Any) -> str:
    if isinstance(value, (dict, list)):
        return json_pretty(value)
    return json.dumps(value, ensure_ascii=False)


def print_response(command: str, request: dict[str, Any], response: dict[str, Any]) -> None:
    if not response.get("ok"):
        msg = response.get("error") or response.get("msg") or json_pretty(response)
        raise RobotctlError("거부됨: {}".format(msg))

    if command == "get":
        name = request.get("name")
        if name is None:
            value = response.get("params", response.get("value", response))
        else:
            value = response.get("value", response)
        if name is not None and not isinstance(value, (dict, list)):
            print("{} = {}".format(name, format_scalar(value)))
        else:
            print(json_pretty(value))
        if "rev" in response:
            print("rev: {}".format(response["rev"]))
        return

    if command == "set":
        value = response.get("value", request.get("value"))
        print("OK: {} = {}".format(request["name"], format_scalar(value)))
        if "rev" in response:
            print("rev: {}".format(response["rev"]))
        return

    if command == "stop":
        print("OK: stop requested")
        return

    if command in ("pause", "resume"):
        paused = bool(response.get("paused"))
        print("OK: {}".format("paused" if paused else "resumed"))
        return

    if command == "do":
        queued = response.get("queued")
        if queued is None:
            print("OK: do {}".format(request["action"]))
        else:
            print("OK: queued {}".format(queued))
        extras = {k: v for k, v in response.items() if k not in {"ok", "queued"}}
        if extras:
            print(json_pretty(extras))
        return

    if command == "save":
        if "saved" in response:
            print("OK: saved {}".format(response["saved"]))
        else:
            print("OK: saved")
        return

    if command == "rollback":
        print("OK: rollback")
        if "rev" in response:
            print("rev: {}".format(response["rev"]))
        return

    if command == "latest":
        latest = response.get("latest", response)
        print(json_pretty(latest))
        return

    if command == "describe":
        print(json_pretty(response))
        return

    print(json_pretty(response))


def add_connection_options(parser: argparse.ArgumentParser, *, defaults: bool) -> None:
    default_value: Any = None if defaults else argparse.SUPPRESS
    host_help = "튜닝 서버 호스트 (기본: %(default)s)" if defaults else "튜닝 서버 호스트"
    port_help = "튜닝 서버 포트 (기본: %(default)s)" if defaults else "튜닝 서버 포트"
    timeout_help = (
        "연결/응답 타임아웃 초 (기본: %(default)s)"
        if defaults else
        "연결/응답 타임아웃 초"
    )
    parser.add_argument("--host", default=DEFAULT_HOST if defaults else default_value,
                        help=host_help)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT if defaults else default_value,
                        help=port_help)
    parser.add_argument("--timeout", type=float,
                        default=DEFAULT_TIMEOUT if defaults else default_value,
                        help=timeout_help)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="EV3 튜닝 서버 newline-JSON CLI"
    )
    add_connection_options(parser, defaults=True)

    common = argparse.ArgumentParser(add_help=False)
    add_connection_options(common, defaults=False)

    subparsers = parser.add_subparsers(dest="command", required=True)

    p_get = subparsers.add_parser("get", parents=[common], help="param 조회")
    p_get.add_argument("name", nargs="?", help="생략하면 전체 조회")

    p_set = subparsers.add_parser("set", parents=[common], help="param 변경")
    p_set.add_argument("name")
    p_set.add_argument("value")

    subparsers.add_parser("stop", parents=[common], help="network stop 요청")
    subparsers.add_parser("pause", parents=[common], help="속도 0 일시정지 요청")
    subparsers.add_parser("resume", parents=[common], help="일시정지 해제 요청")

    p_do = subparsers.add_parser("do", parents=[common], help="단일 동작 트리거")
    p_do.add_argument("action")
    p_do.add_argument("kv", nargs="*", metavar="k=v")

    subparsers.add_parser("save", parents=[common], help="현재 params 저장")
    subparsers.add_parser("rollback", parents=[common], help="마지막 저장값으로 복귀")
    subparsers.add_parser("latest", parents=[common], help="최신 telemetry 조회")
    subparsers.add_parser("describe", parents=[common], help="스테이지 params/actions 메타 조회")

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        request = build_request(args)
        response = send_request(
            request,
            host=args.host,
            port=args.port,
            timeout=args.timeout,
        )
        print_response(args.command, request, response)
    except RobotctlError as exc:
        print("robotctl: {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
