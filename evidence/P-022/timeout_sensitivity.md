# P-022 timeout sensitivity

```text
P022_TIMEOUT_SENSITIVITY = PASS_DIAGNOSTIC

EXPERIMENT_ROLE = DIAGNOSTIC_ONLY
REPAIR_SELECTED = NO
TIMEOUT_ONLY_REPAIR_SUPPORTED = NO
ALL_RUNS_FRACTION_1_STABLE_TIMEOUT = NONE
MONOTONIC_TIMEOUT_IMPROVEMENT = NO
P022_PASS = NO
BASELINE_PRESERVED = YES
BASELINE_FINAL_GATE = FAIL_CARTESIAN_API
P023 = NOT_OPENED
NEXT_ALLOWED_GATE = P-022
```

반복 횟수는 timeout마다 5다. `START_IK_FAILED`는 fraction 0으로 바꾸지 않았다. 21개 모두 fraction이 나와도 이 실험을 PASS로 올리지 않는다.

바꾼 값은 IK search budget 하나다. 시작 `setFromIK`와 group default IK timeout에 같은 값을 넣었다. `CartesianInterpolator`는 경로 IK에 0.0을 넘기고, `RobotState`가 그 값을 group default로 바꾼다. 실험 전에 읽은 library default는 0.5 s다. 기준선 binary는 `setDefaultIKTimeout`을 호출하지 않았으므로, 이 표의 5 ms 행은 그 기준선 실행의 재실행이 아니다.

그대로 둔 것: ProcessPath, UR5e model, datum, orientation, solver KDL, seed policy(`setToDefaultValues`), `eef_step` 0.005 m, threshold 0.95, `avoid_collisions` false, jump detection disabled, search discretization 0.005.

`ALL_21_RUNS_VALID`는 21개 run 모두 fraction 숫자가 있다는 뜻이다. fraction 1을 뜻하지 않는다.

```text
TIMEOUT_MS  REPEAT  START_IK_FAILED_RUNS   FULL  PARTIAL  MIN        MEDIAN  ALL_21  ELAPSED_S
5           1       5, 13                  19    0        1          1       NO      0.266
5           2       7, 17                  16    3        0.205882   1       NO      0.289
5           3       2, 12, 15, 17          16    1        0.575758   1       NO      0.279
5           4       5, 16, 20              16    2        0.205882   1       NO      0.277
5           5       5, 7, 8                18    0        1          1       NO      0.285
10          1       11, 17                 18    1        0.205882   1       NO      0.306
10          2       -                      18    3        0.181818   1       YES     0.362
10          3       -                      20    1        0.393939   1       YES     0.302
10          4       -                      19    2        0.264706   1       YES     0.310
10          5       5                      19    1        0.272727   1       NO      0.284
20          1       -                      21    0        1          1       YES     0.280
20          2       -                      18    3        0.0882353  1       YES     0.391
20          3       -                      18    3        0.0882353  1       YES     0.360
20          4       -                      19    2        0.0882353  1       YES     0.353
20          5       -                      20    1        0.0882353  1       YES     0.295
50          1       -                      20    1        0.637255   1       YES     0.303
50          2       -                      21    0        1          1       YES     0.292
50          3       -                      19    2        0.117647   1       YES     0.374
50          4       -                      21    0        1          1       YES     0.315
50          5       -                      21    0        1          1       YES     0.294
100         1       -                      18    3        0.264706   1       YES     0.377
100         2       -                      20    1        0.333333   1       YES     0.315
100         3       -                      18    3        0.333333   1       YES     0.359
100         4       -                      18    3        0.0882353  1       YES     0.379
100         5       -                      20    1        0.117647   1       YES     0.299
```

5 ms는 다섯 번 모두 `START_IK_FAILED`가 있고 실패 run이 움직인다. 20 ms부터 이 다섯 번 안에서는 시작 IK 실패가 없었다. fraction 1인 run 개수는 20 ms, 50 ms, 100 ms 모두 반복 사이에 달라진다. 100 ms가 50 ms보다 안정적이지 않다. 다섯 번 연속으로 21개 run이 모두 fraction 1인 timeout은 없다.

최소 search budget을 고르지 않았다. 새 frozen configuration으로 acceptance를 다시 실행하지 않았다.
