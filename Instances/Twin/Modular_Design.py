import numpy as np
from VehicleModelTwin import Vehicle
from Instances.Twin.base import Modular, CONTINENTAL12580R13
currveh = Modular()
currtire = CONTINENTAL12580R13()
# Coordinates used in Modular vehicle computations

# strut_x = 0
# strut_y = 0
# strut_z = 0
# obj_x = 0
# obj_y = -20.1
# obj_z = 0
# ibj_x = 25
# ibj_y = 12.2
# ibj_z = 0
# lbj_x = 0
# lbj_y = 0
# lbj_z = 0
# lbell_x = -20
# lbell_y = 28.02+16.6
# lbell_z = 0
# rbell_x = -20
# rbell_y = 0
# rbell_z = 0

strut_x = 0
strut_y = 0
strut_z = 0
obj_x = 0
obj_y = 0
obj_z = 0
ibj_x = 0
ibj_y = 0
ibj_z = 0
lbj_x = 0
lbj_y = 0
lbj_z = 0
lbell_x =0
lbell_y = 0
lbell_z = 0
rbell_x = 0
rbell_y = 0
rbell_z = 0

instance = Vehicle(
    np.array([1046.68 + strut_x, 70.42 + strut_y, 1491.13 + strut_z]),
    np.array([1118.06 + obj_x, 205.00 + obj_y, 1278.88 + obj_z]),
    np.array([1193.06 + ibj_x, 69.00 + ibj_y, 1282.88 + ibj_z]),
    np.array([963.30, 230.00, 1012.20]),
    np.array([957.67 + lbj_x, 161.50 + lbj_y, 978.43 + lbj_z]),
    np.array([1128.06 , 0.00, 1203.88]),
    np.array([1128.06 , 0.00, 1303.16]),
    np.array([1182.09 + lbell_x, -28.19 + lbell_y, 1224.88 + lbell_z]),
    np.array([1216.48 + rbell_x, 103.75 + rbell_y, 1224.18 + rbell_z]),
    r_La = np.array([839.67, 0.00, 1013.44]),
    r_Lb = np.array([1014.67, 0.00, 1013.44]),
    r_strut = np.array([974.14, 120.35, 1071.88]),
    GVW = 1050,
    b = 730,
    CG_height = 250,
    slr = currveh.slr,
    dlr = currveh.dlr,
    initial_camber = currveh.initial_camber,
    toe_in = currveh.toe_in,
    tw = currveh.tw,
    wb = currveh.wb,
    wheel_rate_f = currveh.wr_front,
    wheel_rate_r = currveh.wr_rear,
    tire_stiffness_f = currveh.tire_stiffness_front,
    tire_stiffness_r = currveh.tire_stiffness_rear,
    pinion = currveh.pinion,
    tirep = currveh.tirep,
    dila = currveh.dila,
    assumed_rack_stroke = currveh.assumed_rack_stroke,
    linkage_effort = currveh.linkage_effort,
    tiredata = currtire.tiredata,
    CF_Loads = currtire.CF_Loads,
    CF_Stiffnessrad = currtire.CF_Stiffnessrad,
    CF_pneumatictrail = currtire.pneumatictrail
)