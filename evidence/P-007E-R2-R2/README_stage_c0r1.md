# P-007E-R2-R2 Stage C0-R1 — Acquisition Harness Repair & Re-Preregistration

> 선행: Stage A `CLOSED @3579ab5` · B `CLOSED @4d0c709` · C0 `CLOSED @121dc43` · C1 `X — CLOSED @64b32bc`
> 🔴 **네 게이트 모두 수정하지 않았다** (`stage_c0r1_immutability.json`, 전부 `git diff --exit-code` exit 0).
> C1 의 20개 anchor 프레임은 `FORENSIC_ONLY / NOT_FOR_THRESHOLD_TUNING / NOT_FOR_ROI_TUNING /
> NOT_FOR_SENSITIVITY_CLAIM` 로 재확인했고, 이번 설계에 그 값을 **전혀 쓰지 않았다**.

이 게이트는 **카메라를 한 번도 켜지 않았다.** `/dev/video0` open 0회 — 전부 정적 코드 + 모의(mock) 시험.

> 🔴 **2라운드 자체 검증(advisor)에서 8건 추가 지적 — 전부 반영했다.** 실제 opt-in 분기가 빈 stub 이었던 것,
> M5(cross-trial 판정)를 "데이터 없이 못 짠다"며 미루려 한 것 자체가 틀렸던 것, `roi_mean` 수정이 selftest
> 시연에만 머물고 `run_trial()`에 연결되지 않았던 것, 사전등록을 "골라 담기" 식으로 만들어 여러 키가
> 조용히 빠진 것(교훈5 재발), SHA 고정이 사전등록 파일 하나뿐이었던 것, `v4l2-ctl` 단축 플래그(`-v`/`-c`)가
> 가드를 우회할 수 있었던 것, crash 처리가 사전등록 문구("atexit/finally")와 실제 구현이 달랐던 것,
> 검수자의 1순위 결정사항(현재 YUYV 상태)이 decision 에 빠져 있던 것 — 아래 본문에 전부 반영했다.

## 판정

```
ACQUISITION REPAIR PASS / C1-R1 RE-PREREGISTERED / NO CAMERA OR ROBOT EXECUTION
```

## 🔴 검수자 1순위 결정사항 — 현재 장치는 `C1-induced YUYV` 다

사전등록은 `MJPG`를 기대한다. **이 상태 그대로 C1-R1 을 실행하면 모든 trial 이 즉시 INVALID 될 수 있다.**
재부팅·재연결로 MJPG 가 돌아오는지 확인하지 않았다(확인 자체가 장치 접근이다). 선택지:
① 승인된 1회 S_FMT 로 MJPG 복귀(별도 게이트) ② YUYV 를 새 기준선으로 수용 — 이 경우 **ROI·가드가
MJPG 프레임(Stage B3) 기준으로 정해졌다는 점**을 함께 고려해야 한다.

## 고친 것

| C1 원인 | 수정 |
|---|---|
| ① `HARNESS_OUTPUT_PATH_DEFECT` | `run_root/{manifest.json, trials/, frames/, summary/}` 로 경로 완전 분리. 🔴 **구현 중 이 결함이 실제로 재발**했다 — `--out`(파일)을 그대로 디렉터리 인자로 재사용. `--work-dir` 신설 + CLI 레벨 충돌 가드(exit 2) 로 막고 T0 로 검증 |
| ② `FORMAT_CHANGED_DURING_CAPTURE` | `cv2.VideoCapture` 제거 → `v4l2-ctl --stream-mmap` 기반(1차 소스로 S_FMT 미호출 확인) `NoNegotiationV4L2Reader` |
| M6 사전등록 내적 모순 | C1-R1 사전등록에서 capture_mechanism 교체로 해소. **C0(@121dc43)는 고치지 않았다** — 역사적 사실 보존 |
| M2 증분 저장 없음 | trial 단위 원자적 저장(tmp+fsync+os.replace), T3·T8 로 검증 |
| M3 포맷 변경 미처리 | `run_trial()` 이 매 trial 전/후 `format_invariant` 를 읽기 전용으로 확인, 불일치 시 그 trial INVALID |
| M4 `roi_mean` 전체 프레임 | `measurement.py roi_mean_brightness()` — ROI 마스킹 평균. selftest 로 버그값(138.6) vs 수정값(68.1) 차이 시연 |
| M5 분석 코드 부재 | 🔴 **부분 해결** — `run_trial()`/`run_full_sham()` 으로 캡처+추정+delta+저장은 구현했다. **cross-trial LOO/publication 임계·Q1~X 최종 판정은 아직 없다** — C1-R1 준비 단계로 남겼다(의도적 축소, 데이터 없이 짤 수 없는 부분) |

## 🔴 구현 중 실제로 재발·발견된 결함 2건 (1라운드)

1. **출력 경로 혼용 재발** — `negative_tests(tmp_base)` 가 `a.out`(JSON 파일 경로)을 그대로 디렉터리 인자로 받아 `NotADirectoryError`. `--work-dir` 분리 + CLI 충돌 가드로 수정(T0).
2. **`RunManifest` 디렉터리 생성 누락** — `run_root` 를 만들지 않고 바로 `manifest.json` 을 쓰려다 `FileNotFoundError`. T8(통합 모의 시험)이 잡았다. `os.makedirs(run_root)` 추가.

둘 다 "경로/디렉터리 처리"라는 **같은 클래스의 결함**이 같은 세션 안에서 두 번 나왔다 — 이 프로젝트에서 반복되는 취약 지점으로 기록한다.

## 🔴 2라운드 — advisor 지적 8건 전부 반영

| # | 지적 | 조치 |
|---|---|---|
| 1 | 실제 opt-in 분기가 빈 stub | `execute_c1r1()` 구현(env_guards 이식+배선). `--run-root` 인자 신설. T10 으로 FakeReader 검증 |
| 2 | M5 연기 사유가 틀림 | `scripts/p007e_r2r2_analysis.py` 결정적 순수함수 구현. T9 로 Q1/Q2/Q3/Q4/X 5분기 합성 단위시험 |
| 3 | `roi_mean`/페이싱이 trial 루프에 미연결 | `run_trial()` 이 매 프레임 `roi_mean_fn` 호출 + 0.25s 페이싱 + 2.0s trial 간격 + `frame_interval_stats` |
| 4 | 사전등록 "골라 담기"로 키 누락 | `dict(c0)` 전체를 베이스로, `superseded_keys` 만 명시 교체. python assert 로 누락 0·예상 밖 변경 0 확인 |
| 5 | SHA 고정이 사전등록 1개뿐 | acquisition·measurement·analysis 3개 모듈 sha256 을 사전등록에 고정, `--run-sham` 시 강제 대조(T12) |
| 6 | 단축 플래그(`-v`/`-c`) 우회 가능 | `v4l2-ctl --help-all` 1차 확인 → `_assert_safe_argv` 를 default-deny 로 교체(T11) |
| 7 | crash 처리가 사전등록 문구와 다름 | SIGTERM/SIGHUP→SystemExit 변환 + `except BaseException` + `atexit` 안전망 |
| 8 | 검수자 1순위 결정사항 누락 | 위 "검수자 1순위 결정사항" 섹션 + `next_decision_required_by_reviewer` 1번으로 승격 |

## 아키텍처

```
Acquisition Layer   scripts/p007e_r2r2_acquisition.py
                     CapturedFrame{sequence, capture_timestamp, encoded_bytes, format_hint, width, height, ok, error}
                     subprocess 호출 지점은 _run_v4l2ctl 단 하나, 전부 _assert_safe_argv(화이트리스트) 통과
Measurement Layer    scripts/p007e_r2r2_measurement.py — bytes 만 안다. 장치를 모른다
Orchestration        scripts/p007e_r2r2_c1r1_sham.py — 기본값 NO CAPTURE. 이중 opt-in
```

🔴 **자기참조 오탐 회피** — 화이트리스트/블랙리스트 상수는 금지 문자열을 "정의"로 담고 있어
naive substring 검사가 자기 자신을 오탐한다(i2c 선례와 동일 함정). `acquisition_module_check()`
는 대신 **구조**(subprocess 호출이 `_run_v4l2ctl` 하나뿐이고 가드를 거치는가)를 AST 로 검사한다.

## 동결 재확인 — C0 와 바이트 단위로 동일

ROI(3영역+M1~M10) · ORB(3000,1.2,8,12) · BFMatcher · estimateAffinePartial2D(RANSAC 1.5px) ·
quality guards · K=5/M=60/N_sham=10 · threshold 공식 · LOO · 판정어휘 Q1~X.
🔴 python `assert c0[...] == c1r1[...]` 로 기계적 대조했다(이번 응답 로그).

## 합성 추정기 회귀

| | C0 (@121dc43) | C1-R1 (리팩터 후) |
|---|---|---|
| 평행이동 오차 최대 | 0.703 px | 0.70305 px |
| 회전 오차 최대 | 0.060° | 0.06007° |

일치 — Measurement Layer 분리·리팩터가 동작을 바꾸지 않았다.

## 음성시험 T0~T12 — 전부 PASS

| | 내용 |
|---|---|
| T0 | 🔴 CLI 레벨 `--out`/`--work-dir` 경로 충돌 가드 (exit 2, 결과 파일 미생성) |
| T1 | 디렉터리/파일 경로 오용 → ABORT BEFORE RUN |
| T2 | mock trial 3개 원자적 저장 |
| T3 | 저장 직전 강제 예외 → 선행 trial 생존, 미완성 trial 부재로 명확 |
| T4 | malformed frame → `decode_gray` 가 예외 없이 `None` (dropout, 전체 소실 아님) |
| T5 | mock YUYV 메타데이터 → `format_invariant.match=False` |
| T6 | 사전등록 sha256 변조 → ABORT BEFORE CAPTURE |
| T7 | opt-in 없음 → NO CAPTURE |
| **T8** | 🔴 **통합 모의 end-to-end** — `FakeReader`(실제 장치 미참조)로 `run_trial`/`run_full_sham` 전체를 끝까지 돌린다. 정상 3-trial 완주(전부 VALID, roi_mean 기록 확인) + trial 도중 crash 시뮬레이션(trial0 생존, manifest `CRASHED`, `last_completed_trial=0`) — §14 핵심 acceptance 를 실제 함수로 확인 |
| T3b | 쓰기 "도중" 크래시 — `.tmp` 는 남지만 파싱 불가, 최종 파일은 없음(미완성 식별 가능) |
| T9 | 🔴 `analysis.py` Q1/Q2/Q3/Q4/X 5분기 전부 합성 trial 로 단위시험(순수 python, 로컬 Mac 재현 가능) |
| T10 | 🔴 `execute_c1r1()` 을 FakeReader 로 호출 — **실제 캡처 경로와 동일한 함수**, reader 클래스만 교체 |
| T11 | 🔴 `v4l2-ctl` 단축 플래그(`-v`/`-c`/`-p`) 우회 거부 확인 |
| T12 | 모듈 sha256 불일치 감지 로직 확인 |

이것이 C1 의 교훈("`--selftest` 는 capture 0 만 증명했지 올바른 캡처는 증명하지 못했다")을
이번엔 메운 지점이다 — T8 은 실제 캡처 경로가 쓸 바로 그 함수(`run_trial`/`run_full_sham`)를
모의 입력으로 끝까지 실행한다.

## 산출물

`stage_c1r1_preregistration.json`(C1-R1 규칙 동결) · `stage_c0r1_decision.json` · `stage_c0r1_acceptance.json`(R1-01~28) ·
`stage_c0r1_command_budget.json` · `stage_c0r1_immutability.json` · `logs/c0r1_negative_tests.json` · `logs/c0r1_synthetic_regression.json`

## 하지 않은 것

카메라 기동 0 · 새 캡처 0 · 실제 UVC 스트림 0 · 실제 장치 S_FMT 0 · 로봇/컨트롤러 접근 0 ·
152→150 0 · **C1-R1 실행 0**(이번 게이트가 증명하는 것은 "결과 데이터 없이 정적으로 결함을
고쳤고, 사후 튜닝 없이 다음 stationary sham 을 다시 할 수 있는 protocol 을 고정했다"는 것뿐이다.
**실제 카메라가 안정적이라는 뜻이 아니다.**)
