"""freecadcmd build_model.py  -> CylindricalArmRobot.FCStd (home pose). Open it in FreeCAD and move the joints from the simulator."""
import os, sys
here = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
src = open(os.path.join(here, "fc_scene.py")).read()
exec(compile(src, "fc_scene.py", "exec"))
sys.path.insert(0, here)
import robot
q_home, _ = robot.ik([600, 0, 700], [0, 0, -1])
doc = apply({"q": [float(x) for x in q_home]})
doc.saveAs(os.path.join(here, "CylindricalArmRobot.FCStd"))
print("saved", len(doc.Objects), "objects")
