# 출처

조회일 2026-10-31.

## 1. Yahboom 공식 공부 페이지

```text
https://www.yahboom.net/study/DOFBOT-Pro/
```

다운로드 항목이 페이지에 있다.

```text
Hardware_Info
https://drive.google.com/drive/folders/1tZ9P83NefgFXAJI-hjjUlVNw-03Yv7Y0

3D_Model_File
https://drive.google.com/drive/folders/1ItArfCEFyQPBZFL3xsVWgSauXUOD8DPq
```

Hardware_Info 폴더 제목은 `6.Hardware_Model_File`이다. 상위 목록의 수정 표시는 11/7/24다. 안의 항목은 Bus servo, Hardware Interface Manual, Communication Protocol, Model file, Functional distribution diagram이다. Model file은 3D_Model_File과 같은 폴더다.

Hardware Interface Manual 파일은 Google Drive가 `Quota exceeded`를 반환해서 열지 못했다.

## 2. 3D 모델 폴더

폴더 목록의 수정일:

```text
DOFBOT-Pro-AI.STEP     Feb 2     51196917 bytes
DOFBOT_Pro-V24.csv     7/29/24
DOFBOT_Pro-V24.urdf    9/10/24
```

연도는 폴더 목록에 연이 없는 `Feb 2`만 표시됐다. STEP 파일 id는 `1G40j6s4r1I6aSjUVUEhRvyEvYezH4FUC`다. 받기에서 `Quota exceeded`가 나왔고, 좌표계와 단위와 구멍은 읽지 못했다.

받은 파일:

```text
DOFBOT_Pro-V24.urdf
sha256 d55d542697546d90644158c474d98974802aec055c31ba2a20d43b312c4c57f1
19401 bytes

DOFBOT_Pro-V24.csv
sha256 e602c47a450f0c78865e92a83e95692ba9f3a0ffc2f565b2399c2af703397d4e
26039 bytes
```

## 3. 조립 자료

공식 GitHub `YahboomTechnology/DOFBOT-Pro` `main` `57cc4369af73d65188d33e49227650c789779a7e`, 커밋 시각 2026-08-26.

```text
https://github.com/YahboomTechnology/DOFBOT-Pro
```

트리 1056개 안에 STEP, STL, URDF는 없다. 조립 PDF는 있다.

```text
02. Assembly course/2. Calibrate the Robotic Arm.pdf
02. Assembly course/3. Install the map.pdf
21. ROS Basic Course/20.urdf model introduction.pdf
```

## 4. ROS2 / MoveIt

같은 GitHub의 MoveIt 과정은 코드 수업이다. 기계 치수 도면은 없다. URDF 소개 PDF의 예제 관절은 `xyz="0 0 0.06605"`이고 mesh 패키지는 `dofbot_moveit`이다. 계획 모델의 Arm1 `0.0895`와 다른 예제다.

## 5. ballbot 906da9f

계획 URDF의 기존 출처다. 이번 조사에서 그 커밋을 다시 받지 않았다. 공식 Drive의 URDF와 sha256이 다르다.
