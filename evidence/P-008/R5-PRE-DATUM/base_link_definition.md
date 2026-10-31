# base_link

로봇 이름은 `DOFBOT_Pro-V24`다. 링크 이름 `base_link`가 있다. `world`, `base`, `base_footprint`는 없다.

`base_link`를 child로 가지는 joint가 없다. root다.

```text
world → base_link 사슬 없음
BASE_IS_ROOT = YES
```

visual origin `0 0 0`, rpy `0 0 0`. collision origin도 `0 0 0`, rpy `0 0 0`. mesh scale 요소는 없다.

```text
mesh = package://dofbot_pro_description/meshes/base_link.STL
로컬 파일 = .cache/jetson_robot_model/meshes/base_link.STL
```

inertial origin xyz는 `-8.03990775049002E-05 -0.047909102545025 0.0178929922479837`, rpy `0 0 0`이다. 질량은 `0.226715356738094`이다. 이 원점은 링크 원점과 다르다.

## mesh 범위

꼭짓점 전체다. 단위는 URDF 숫자 그대로 m이다.

```text
x  -0.092500 .. 0.092453
y  -0.221496 .. 0.078500
z  -0.015000 .. 0.097800
```

z가 0.08 이상인 부분은 x, y가 약 ±0.040 안에 있다. 둥근 몸체다.

y가 -0.08보다 작은 부분은 8057개 꼭짓점이다. x는 -0.092500 .. 0.092453, z는 -0.015000 .. 0.009000이다. 낮은 판이 -Y 쪽으로만 뻗는다. +Y 쪽으로 y>0.08인 꼭짓점은 0개다.

z=0은 mesh 바닥이 아니다. 바닥은 z=-0.015 m다. mesh 꼭대기는 z=0.097800 m다. 낮은 판은 z=-0.015 m와 z=0.009 m 양쪽에 걸치므로, z=0은 그 판의 윗면도 아랫면도 아니다.

mesh를 제작 datum으로 보증하는 기록은 없다.

```text
PHYSICAL_DATUM_CONFIDENCE = LOW
```

도식은 `base_link_axes.png`다. 점은 표시용으로 건너뛰어 그렸고, 범위 숫자는 전체 꼭짓점이다.
