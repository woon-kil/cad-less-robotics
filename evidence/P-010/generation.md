# P-010 생성 절차

공개 저장소에는 STEP/STL이 없다. 공식 절차는 FreeCAD 매크로 `uv_0.py`다. 이 파일을 수정하지 않았다.

1. `https://github.com/peterzask/freecad-shoe-last-wb` 의 `master` `59d1db3b48b460634f1504ddab7d1a1f33be05d7` 를 가져온다.
2. FreeCAD 1.1.4 (`freecadcmd`, git `4fd3bf320`)에서 문서를 연다.
3. `uv_0.py` 는 import 시점에 `FreeCADGui.ActiveDocument.ActiveView` 를 읽는다. 콘솔만으로는 이 속성이 없어, `FreeCADGui.showMainWindow()` 뒤에 문서를 만들고 같은 파일을 실행했다.
4. `uv_0.py` 가 `Part.BSplineSurface`, heel/toe cap, `Part.makeShell`, `Part.makeSolid`, `removeSplitter` 를 만든 뒤 `exportStep("shoe_last.step")` 과 `MeshPart.meshFromShape` 로 `shoe_last.stl` 을 쓴다.
5. STEP 기록 로그와 `END-ISO-10303-21` 까지 나온 뒤에 프로세스가 exit 139로 끝났다. 두 파일은 그 전에 닫혀 있었다.

생성 환경은 macOS의 FreeCAD 1.1.4다. 로봇, ROS 2, MoveIt 2, Gemini 335L은 사용하지 않았다.

산출 파일:

- `assets/cad/shoe_last.step` SHA-256 `66bb1aa019a410634fa317065ee550d4f988e434ccc4e98c43b529736b5f3196`
- `assets/cad/shoe_last.stl` SHA-256 `fc6be622849eec4dd450f7827fd9c6ddd125f313034943586fb4f723eb6ac7d6`
