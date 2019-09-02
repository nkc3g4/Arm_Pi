import LeArm 
import kinematics as kin
import sys
LeArm.runActionGroup(str(sys.argv[1]),1)
print(str(sys.argv[1]))
