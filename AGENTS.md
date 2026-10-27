# AGENTS.md

## 언어

대화, 주석, 문서는 한글로 작성한다. 토픽·타입·API 이름은 영어 그대로 둔다.

## Git 자율화

정본: `.cursor/rules/git-autonomy.mdc`. 글로벌 `~/.cursor/rules/git-autonomy.mdc`보다 이 파일이 우선한다.

- 원격은 GitHub **public** (`private: false`). `git push origin main`은 작업 단위가 끝나면 확인 없이 한다.
- 트리거: 「자율 커밋」, 「자율 푸시」, 「자율 커밋 푸시」, 「커밋」, 「푸시」, 「진행」
- ROS 2 Humble이 있는 호스트에서 C++·패키지·런치를 바꾸면 `colcon test --packages-select vision_path_generator robot_controller` 통과 후 커밋한다. Humble이 없는 호스트에서는 colcon이 없다는 이유만으로 커밋을 막지 않는다.
- `.env`, 시크릿, `--force`, 미측정 하드웨어 PASS, 가짜 성능 숫자는 커밋하지 않는다.
- `README.md`, `AGENTS.md`, `.cursor/rules/`, `.cursorrules`, `evidence/`는 커밋 대상이다.
- `docs/`는 **커밋 대상이 아니다** (2026-10-27 사용자 지시). `.gitignore`에 들어 있다. 플랜·완료보고서·이슈보고서는 로컬 디스크에만 둔다. 게이트 증거는 `evidence/`가 담당한다.
