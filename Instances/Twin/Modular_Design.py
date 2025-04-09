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
    np.array([1046.68 + strut_x, 68.41 + strut_y, 1440.54 + strut_z]),    #    
    np.array([1096.96 + obj_x, 200.77 + obj_y, 1228.09 + obj_z]),
    np.array([1193.06 + ibj_x, 69.00 + ibj_y, 1232.29 + ibj_z]),
    np.array([963.30, 230.00, 961.62]),   #
    np.array([957.67 + lbj_x, 159.50 + lbj_y, 927.85 + lbj_z]),
    np.array([1128.06 , 0.00, 1203.88]),
    np.array([1128.06 , 0.00, 1303.16]),
    np.array([1202.09 + lbell_x, -28.19 + lbell_y, 1174.3 + lbell_z]),
    np.array([1236.48 + rbell_x, 103.75 + rbell_y, 1173.59 + rbell_z]),
    r_La = np.array([839.67, 0.00, 962.85]),
    r_Lb = np.array([1014.67, 0.00, 962.85]),
    r_strut = np.array([981.77, 113.00, 1065.57]),
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