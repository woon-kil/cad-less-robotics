# 표시용 분사 데모

기준 커밋은 `fdfa9c9`다. 입력은 `restore.json`의 관절 944개와 원본 구간 PROCESS 720, TRANSITION 20이다. 관절값은 바꾸지 않았다.

재생은 브라우저에서 `viewer/index.html`을 연다. 재생 간격 0.04 s는 화면용 시간이며 로봇 시간이 아니다. 분사는 `tool0` +Z 방향의 선이다. PROCESS 구간과 그 중간 rung에서만 켜지고, TRANSITION과 마지막 정지에서는 꺼진다.

화면의 coupon은 범위 상자다. 곡면 mesh가 아니다. 로봇은 막대 기구학 표시다. 노즐과 분사선은 시각 효과이며, 도포 품질을 예측하지 않고 새 공구 형상의 충돌도 검사하지 않았다. 이 화면의 확인은 기존 고정 표본 이산 검증과 별개다.

## 확인

| 항목 | 결과 |
| --- | --- |
| 관절 944개와 `restore.json` 일치 | 일치 |
| PROCESS 원본 720, TRANSITION 원본 20 | 일치 |
| 그래프 모서리 PROCESS 923, TRANSITION 20 | 일치 |
| 중간 rung의 다음 모서리 | PROCESS |
| 시작 `0` | PROCESS 0→1, 분사 ON, tool0 `[0.324, -0.050, 0.160]`, 축 `[0.000, -0.007, -1.000]` |
| 진행 `400` | PROCESS 307→308 중간, 분사 ON, 재생 시간 16.00 s |
| 전환 `34` | TRANSITION 34→35, 분사 OFF, 재생 시간 1.36 s |
| 종료 `943` | 분사 OFF, 재생 시간 37.72 s |
| 재생, 일시정지, 처음부터 | Chrome 마우스 이벤트로 버튼을 누름. `0 / 943` → `5 / 943`에서 정지 → `0 / 943`, `0.00 s` |

화면 기록은 `captures/start.png`, `process.png`, `transition.png`, `end.png`다. P-022, 채택 manifest, `restore.json`은 수정하지 않았다.
