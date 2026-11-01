# P-011 생성

형상 계약 `geometry_contract.md`는 수정하지 않았다.

## 방법

네 개의 영역 solid를 만든 뒤 fuse한다. 각 solid는 B-spline 윗면, 그 면의 edge로 만든 ruled 옆면, 평면 바닥이다. 영역 경계는 형상 책임의 경계이며, face 수를 맞추려고 면을 나누지 않았다.

+X 끝의 비대칭 landmark는 원통이다. 축은 `(162, 84, 10)` mm, 반지름 5 mm, 길이 18 mm, 방향 +X이다. 공정 표면은 그 위에 있고, 원통의 가장 높은 점은 z=15 mm이다.

STEP을 다시 읽었을 때 pcurve가 없으면 B-spline 경계에서 `BOPAlgo_InvalidCurveOnSurface`가 난다. 내보내기 전에 `write.surfacecurve.mode = 1`을 설정한다. 메모리 안에서 tolerance를 올려 검사를 통과시키는 방식은 쓰지 않았다.

## 실행

```text
P011_STEP=assets/cad/complex_surface_coupon.step \
  /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd \
  scripts/p011_generate_coupon.py
```

GUI는 쓰지 않았다. 외부 CAD 파일은 쓰지 않았다.

## 버전

| 항목 | 값 |
| --- | --- |
| SOURCE | SELF_GENERATED |
| GENERATOR | FreeCAD Part / OpenCascade |
| FreeCAD | 1.1.4, build 2026-09-28, git `4fd3bf320d9566a27e60069fc8387448aaa3a094` |
| OpenCascade | 7.8.1 |
| 생성 시점 HEAD | `05d8ae4` |
| 생성 스크립트 SHA-256 | `6e1b9e5cdb58c93d2869591e1659d14e045ec6d33c91b75f3de2aaaa01fcf6a1` |

재현은 같은 스크립트를 새 프로세스로 두 번 실행했다. 두 STEP의 SHA-256은 같다.
