import os
import numpy as np
import json
datapath = "/mnt/83296afe-1ebf-459c-999e-755847e2077a/calibration/APP_Calibration-master/data/set1/2024_01_15_17_43_55/"
files = os.listdir(datapath)
list_filename = []
for f in files:
    if "rgb" in f:
        list_filename.append(f[0:-7])
list_filename = np.sort(list_filename)

for filename in list_filename:
    path_json = datapath + filename + "UR10.json"
    st_json=open(path_json, "r")
    json_robot = json.load(st_json)
    robot=json.dumps(json_robot)
    id=robot.split("\\")[3][1:]
    matrix_raw=robot.split("\\")[6][3:-2]
    matrix_rows=matrix_raw.split("[")[2:5]
    matrix=np.eye(4)
    matrix_3x4=[]
    for row in matrix_rows:
        row_part=[float(i) for i in row.split(",")[0:3]]
        row_part=row_part+[float(row.split(",")[3][:-1])]
        matrix_3x4.append(row_part)
    matrix[0:3,:]=np.array(matrix_3x4)
    robot_remake={"id":id,"matrix":matrix.tolist()}
    with open(datapath + filename + "UR10_Re.json", 'w') as sf:
        json.dump(robot_remake, sf)
    print(robot)
