from __future__ import print_function
from __future__ import division
import subprocess
import os
import sys
# main = "D:/11projects/pal-face/camera/python_demo/ocamcalib_for_panoramic_unfold/Release/Project1 ./calib_results_1121.txt ./result/%06d.jpg"
#
# print("test")
# os.system(main)
# print("test2")
#
# # if os.path.exists(main):
# rc,out= subprocess.getstatusoutput(main)
# print (rc)
# print ('*'*10)
# print (out)

sys.path.append(r'..\..\insightface\deploy')
import face_model

try:
    while(True):
        try:
            print("loop")
        except Exception as e:
            print(e)
except KeyboardInterrupt:
    print("exit")
    pass
