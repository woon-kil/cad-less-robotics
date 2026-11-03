# 전체 layered graph 대응 검토

```text
REVIEW = CORRESPONDENCE_ONLY
NEW_SEARCH = NOT_RUN
INTEGRATED_MANIFEST = NOT_ADOPTED
P-022 = UNCHANGED
PASSING_RECORD_COUNT = 6
SOURCE_RESULT = evidence/UR-ANALYTIC-IK-PROCESS-FULL-LAYERED-GRAPH/
CONTINUOUS_SAFETY = NOT_CLAIMED
TIME_PARAMETERIZATION = NOT_CLAIMED
PHYSICAL_EXECUTION = NOT_CLAIMED
TOOL_FIXTURE_TABLE = NOT_CHECKED
```

원본 결과 JSON을 고치지 않았다. 선택 경로의 sequence 메타데이터와 검사 소스의 집계 규칙을 대조했다. 관절값을 다시 풀거나 충돌 검사를 다시 실행하지 않았다.

## 원본 740구간 대응

선택 rung 944개를 순서대로 이었다. 중간 rung은 앞뒤 원본 pose 하나의 PROCESS 구간을 반쪽 둘로 나누고, 그 외 인접 원본 pose는 도착 sequence가 TRANSITION 목록이면 TRANSITION, 아니면 분할되지 않은 PROCESS다.

| 구분 | 개수 |
| --- | ---: |
| 그래프 edge | 943 |
| 원본 구간 0→1 … 739→740 | 740, 누락 0, 여분 0 |
| PROCESS 원본 | 720 |
| 그중 분할 | 203 (그래프 edge 406) |
| 그중 미분할 | 517 |
| PROCESS 그래프 edge | 923 |
| TRANSITION 원본 | 20 |

PROCESS 표본 93,223은 923개 그래프 edge × 101표본(0…100)과 같다. 분할 구간은 같은 원본 `(from, to)`에 반쪽 2개가 있고, 반쪽마다 101표본이다. 원본 PROCESS 720개를 한 번씩 센 집계는 720이다.

TRANSITION 20개의 도착 sequence는 35, 70, 105, 140, 175, 210, 245, 281, 317, 353, 389, 425, 461, 496, 531, 566, 601, 636, 671, 706이다. 대응 구간은 그 직전 pose에서 해당 pose까지다. 중간 rung은 이 20구간에는 없다.

저장된 검사 결과는 구간별 합격이 아니라 PROCESS 합계와 TRANSITION 합계다. 구간별 오차·충돌은 이 파일에 없다. 따라서 합계 통과를 740개 구간 각각의 개별 기록으로 바꾸지 않는다.

## TRANSITION 40표본

직선 검사의 표본 수는 끝점 포함 `splits + 1`이다. `splits`는 raw joint 차이가 `0.01 * max_joint_span`보다 크면 그 비율의 올림이고, 아니면 1이다. 20구간의 표본 합이 40이면 구간마다 표본이 2개다. 각 TRANSITION은 양 끝 관절만 검사했다. 구간 내부 표본은 없다. 이 통과는 연속 충돌 부재가 아니다.

탐색 edge도 같은 간격 규칙을 쓴다. 선택된 TRANSITION의 raw 차이가 한 간격 이하이면 탐색 중 충돌 검사 역시 양 끝만 본다.

## 접합 기록

기록된 `junction_rad` 0은 각 edge의 선형 보간 끝 표본과 그 edge의 선택 끝 관절의 raw 차이다. 매개변수 0과 1에서는 보간식이 끝 관절과 같다. 서로 다른 두 저장 궤적의 접합 차이를 측정한 값은 아니다.

## 선택 후보 슬롯

`selected[].candidate_index`는 저장 후보 벡터의 인덱스다. 944개 rung 모두 224다. 같은 인덱스이므로 같은 관절값이라는 뜻은 아니다. 관절 벡터는 결과 JSON에 없다. 재현하려면 아래 입력의 후보 224를 읽어야 한다.

선택 배열의 SHA-256은 `eac7465345b5601422bcaa0225a607e885be177f6c5da30d75cd6925e8b066f4`다. 정규화는 `json.dumps(selected, separators=(",", ":"))`다.

## 해시와 재현

| 대상 | SHA-256 |
| --- | --- |
| `fullgraph.json` | `f46217366bcd37a80c4363ccee44e9ed36ff7070f5b3f5b9f1f391974601677d` |
| `summary.json` | `2d02c5b8b5acd03707d8a98a9ba70afccea7c3701f7e5ad2427fd92bdbc80901` |
| `parent_limit.json` | `20a6f269ff94b7a5966367cc3da5d6feb448c06e4c3fecae8195baf5a900e230` |
| `parent.log` | `ef0b2195376b25edaca543ba2280699ddb3d2b77ca78d6dad449946c1fcea313` |
| `inserts.bin` (`R6IN`, 2768164바이트) | `ff1f59d4275cdee2e41f5b8ef0da3182480204eb32d2372e1eb65ef4dd40f153` |
| 검사 소스 `full-graph-src.cpp` | `0824af0a2d775a7db093ff47ba9ab2d9eeab1c66be0cb5a307f9e395b3111347` |
| 부모 `full_graph_parent.py` | `b37e94974ffbf2544ec21570d266615cd30e1afd3f68676bf2f2af07a2dc12ea` |
| `runs.bin` | `4d51b28f0851114b8d36036d0fb96bb1681bf03f89ac31a47bf7d3a18eecfb26` |
| `transitions.bin` | `cc21e6083a9ecb0c740382ccf3f9b8a7e3e9bdaea64017389525cbc04cd821c4` |
| URDF | `a21bf5fb70b3a1745bf2e4816e0f43654539737ba7f2585939ec773d159778d8` |
| SRDF | `ee0d940b486bca52534d5b112c1020e0eaa542130c1301af8dc2fda28c757dbb` |
| coupon mesh | `ea78058ca33866ef3c69764be48f90da4a6f2491455c9077c514ccecd6acbc16` |
| mesh acceptance | `97fa8c989cfae92fc998003ceb010e4c585642fe13b157330a04cb162e610e44` |

R3 JSON 해시는 실행 전 부모 확인값이다. run 6 `cda4368e5f6eb8fe1b289ffd509eb37b7e2f4933b9503780097bdf42e179bcb8`, run 7 `973fa70e38412fe4e2bd952a898e2e35eda3186e207b08c4740f2641c024314a`, run 8 `1e47b542549f3e40b9ee2b7202999047850dfc39d581cb08575be0bc2fdcd709`, run 9 `ced6759a4b784877274b23a89b34442e485910347089d732555c8e2aa1cd9a86`, run 10 `721fec3b096d0ae0c53d1890e4288b97319619f73bd506831d1aa7b9041ea975`, run 11 `d513e81a72dd1acd7afae1bc73b12cfc4154c95cb61fa3f319badae85f8c6539`.

실행은 p620에서 1회였다. 모드 `fullgraph`, 스레드 24, 부모 벽시계 757초, 주소 공간 8,589,934,592바이트, 재시도 0. 부모 경과 343.7852756610373초, exit 0, wall kill false. 자식 결과의 `discrete_pass`는 true이고 결론 문장은 「저장 후보와 직선 TRANSITION을 사용한 전체 기하 후보 경로의 이산 검사 통과」다.

## manifest

새 통합 manifest는 채택하지 않는다. 이번 파일은 원본 740구간이 그래프에 빠지지 않았다는 구조 대응과, 합계 검사의 의미 한계를 남긴다. 구간별 검사 기록과 선택 관절은 결과 JSON 밖에 있다. 표본 사이 안전, 공구·치구·작업대, 시간 매개변수화, 실물 실행은 검사하지 않았다. P-022 `CLOSED_FAIL`과 통과 기록 6개는 그대로다.
