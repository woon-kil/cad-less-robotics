# 제조사 모델

## STEP

`DOFBOT-Pro-AI.STEP`는 목록에 있고 크기는 51196917 bytes다. 파일 본문은 받지 못했다. 원점, 단위, 바닥면, 구멍, J1 중심, 비대칭 형상은 확인하지 않았다.

## URDF

공식 `DOFBOT_Pro-V24.urdf`는 SolidWorks to URDF Exporter 출력이다. `base_link`가 root다. visual origin과 collision origin은 `0 0 0`이다. mesh 경로는 `package://DOFBOT_Pro-V24/meshes/base_link.STL`이다. 그 STL은 이 폴더 목록에 없다.

`Arm1_Joint` origin은 `0 0 0.0895`, rpy `0 0 0`, axis `0 0 1`이다.

## CSV

같은 폴더의 `DOFBOT_Pro-V24.csv`는 exporter 표다. `Arm1_Link` 행의 joint origin은 `0, 0, 0.0895`, axis는 `0, 0, 1`, parent는 `base_link`다. 도면의 치수 공차가 아니다.

`Arm2_Link` 행의 axis는 `0.773585873491249 0.633691483558637 0`이다. 공식 URDF의 Arm2 axis는 `1 0 0`이다. 표의 축과 export된 관절 프레임은 다르다. 이 숫자를 실물 +X 표시로 쓰지 않는다.

## 지도

`Install the map`은 맵의 다각형 외곽을 섀시에 맞추라고 한다. 미터 치수, 로봇 중심, 방향 datum, `base_link`와의 관계는 없다. 설치용 외곽이다.

## 서보 보정

`Calibrate the Robotic Arm`은 서보 중위 오프셋을 저장하는 API다. 결과를 팔을 세우고 그리퍼를 닫는 상태로 설명한다. `base_link`의 z=0이나 +X를 정의하지 않는다. 이번 조사에서 그 명령을 보내지 않았다.
