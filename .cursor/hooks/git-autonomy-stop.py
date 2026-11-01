#!/usr/bin/env python3
"""에이전트 종료 때 커밋되지 않은 변경이 있으면 커밋·푸시를 한 번 요구한다."""

from __future__ import annotations

import json
import subprocess
import sys


def main() -> int:
    sys.stdin.read()
    try:
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        print("{}")
        return 0

    pending = []
    for line in status.stdout.splitlines():
        path = line[3:].strip()
        if " -> " in path:
            path = path.split(" -> ", 1)[1]
        if path.startswith(".cache/") or path.startswith("docs/") or path.startswith(".env"):
            continue
        pending.append(path)

    if not pending:
        print("{}")
        return 0

    print(
        json.dumps(
            {
                "followup_message": (
                    "이 저장소의 git autonomy 종료 훅이다. "
                    "커밋되지 않은 변경이 있다. 이 대화에서 직접 만들거나 수정한 논리 단위만 "
                    "커밋하고 git push origin main 하라. "
                    "대화 시작 전부터 있던 untracked evidence, .cache, docs, .env, 시크릿은 넣지 마라. "
                    "이번 대화의 변경이 없으면 커밋하지 말고 종료하라. "
                    "--force 는 금지다. 서로 다른 Gate를 한 커밋에 섞지 마라."
                )
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
