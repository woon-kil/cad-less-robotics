# P-007E-R2-R2 Stage C1 — Stationary Sham / Ego-Motion Noise Qualification

> 선행: Stage A `CLOSED @ 3579ab5` · Stage B `FOV_PASS / CLOSED @ 4d0c709` · Stage C0 `PREREGISTRATION PASS / CLOSED @ 121dc43`
> 🔴 **세 게이트 모두 수정하지 않았다** (`stage_c1_immutability.json`, 전부 `git diff --exit-code` exit 0).

## 판정

```
X — OBSERVATION INVALID
```

독립적인 근거 2건 (`stage_c1_run.json`):

1. **`HARNESS_OUTPUT_PATH_DEFECT`** — `run_sham()` 의 outdir 인자에 JSON 결과 경로가 그대로 전달돼
   `os.makedirs()` 가 그 경로를 디렉터리로 만들었고, 최종 저장이 `IsADirectoryError` 로 죽었다.
   10 trial 측정값 전부가 메모리에서만 계산되고 디스크·stdout 어디에도 남지 않았다.
2. **`FORMAT_CHANGED_DURING_CAPTURE`** — `cv2.VideoCapture(0, cv2.CAP_V4L2)` 가 FOURCC 미지정으로 열려
   장치 포맷이 **MJPG → YUYV** 로 바뀌었다. 🔴 원인을 OpenCV 4.x `cap_v4l.cpp`
   `autosetup_capture_mode_v4l2()` 원문(1차 소스, WebFetch 확인)으로 **승격 확정**했다 —
   FOURCC 미지정 시 `try_order[]`(BGR24→…→**YUYV**→…→**MJPEG**) 를 앞에서부터 시도하고,
   DaBai 드라이버가 지원하는(Stage B2 확인) 포맷 중 YUYV 가 먼저 성공한다.
   사전등록 INVALID ③(되돌릴 수 없는 포맷 변경)에 해당하고, **모든 trial 이전**(open 시점)에 발생했으므로
   10 trial 전부에 적용된다.

🔴 **두 번째 근거만으로도 X다.** harness 버그를 고치는 것은 재실행을 정당화하지 않는다.
🔴 게다가 **M6**(아래)에 따르면 `cv2.VideoCapture` 는 여는 것만으로 항상 S_FMT 를 호출한다 —
C0 의 `no_control_writes` 와 `capture_mechanism` 자체가 서로 모순이라, harness 를 아무리
잘 고쳐도 **사전등록을 개정(C0-R1)하지 않는 한** 같은 문제가 반복된다.

## UVC 제어 예산 — 숨기지 않고 BUDGET DEVIATION 으로 기록

```
harness 명시적 write   = 0
cv2 암묵적 S_FMT        >= 1  (MJPG → YUYV 관측)
```

다른 제어값(auto_exposure·auto WB·dynamic fps·50Hz·brightness·contrast)은 Stage B 와 동일 — 포맷만 바뀌었다.

## 사전등록 vs harness 불일치 (`stage_c1_preregistration_harness_mismatch.json`)

| ID | 심각도 | 내용 |
|---|---|---|
| M1 | BLOCKING | 출력 경로 버그 (실제 소실의 직접 원인) |
| M2 | BLOCKING | 증분 저장 없음 — 끝단 단발 저장(🔴 M1·M2 는 "둘 중 하나만 고쳐도 10개 전부 건지는" 관계) |
| M6 | BLOCKING(사전등록 결함) | `cv2.VideoCapture` 는 열 때마다 S_FMT 를 호출 — `no_control_writes` 와 내적 모순 |
| M3 | MAJOR | 포맷 변경을 INVALID 로 승격하는 로직 없음 |
| M4 | MAJOR | `roi_mean` 이 전체 프레임 평균(ROI 아님) |
| M5 | MAJOR | LOO·publication 임계·Q1~X 판정 계산 코드 harness 에 없음 |

🔴 **C0 의 "C1 HARNESS READY" 는 사실과 달랐다.** `--selftest` 는 "캡처하지 않는다"만 증명했고
"올바르게 캡처한다"는 증명하지 못했다. harness 는 `stage_c1_harness_defect_manifest.json` 에서
**`DEFECTIVE / DO NOT EXECUTE`** 로 표시했다 — 본문은 수정하지 않는다(provenance 보존).

## 회수한 것 / 회수하지 못한 것

- 회수: `logs/c1_failed_run_frames/*.jpg` 20장 (trial 당 window 1 anchor 프레임). sha256 전송 일치.
  🔴 **포렌식 보존용일 뿐, 어떤 수치도 계산하지 않았다** — window 당 1장으로는 사전등록된
  60장 median estimator를 적용할 수 없다.
- 회수 불가: 10 trial 의 tx/ty/rot/features/matches/inliers/residual/roi_mean/frame interval/delta —
  전부 메모리에서 소실.

## 안전 확인 (사후)

video0 holders 0 · i2c-7 holders는 preflight 기준선과 동일(2264/3091/3092, 새 보유자 없음) ·
모션 가능 프로세스 0 · 디스크 9.2G 동일. **이 단계가 로봇에 모션 명령을 보낸 적은 0건**(코드 사실)이다.
🔴 단, **물리 모션이 실제로 없었다는 것은 운영자 관측 채널이 없어 NOT OBSERVED** — 소프트웨어 가드
정상과 "움직이지 않았다"는 서로 다른 주장이다.

## 재시도하지 않았다

작업의뢰서 §3 `retry after failed result = PROHIBITED`, §22 "그 자리에서 문제를 고친 뒤
C1 을 다시 시작하지 않는다. 별도 검수 gate 로 넘긴다" — 그대로 따랐다.

## 다음 결정은 검수자 몫

- harness M1~M5 수정 (신규 파일명)
- 🔴 **M6 — 사전등록 자체의 재구성**: S_FMT 를 사전등록에 명시적으로 넣을지(예: MJPG 고정),
  아니면 `v4l2-ctl --stream-mmap` 처럼 **협상 없이 캡처하는 경로**로 바꿀지. ROI 는 MJPG 프레임에서
  정의됐다는 점 고려
- 새 게이트(C1-R1) 로 재시도 승인 여부
