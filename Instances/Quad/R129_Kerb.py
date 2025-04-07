import numpy as np
from VehicleModel4Wh import Vehicle
from Instances.Quad.base import R129, CONTINENTAL12580R13
currveh = R129()
currtire = CONTINENTAL12580R13()
# Coordinates used in R129 vehicle computations
strut_x = 10
strut_y = -15.1
strut_z = 0
obj_x = 0
obj_y = -15
obj_z = 0
ibj_x = -5
ibj_y = -10
ibj_z = 0
lbj_x = -10
lbj_y = -15.1
lbj_z = 0

# strut_x = 0
# strut_y = 0
# strut_z = 0
# obj_x = 0
# obj_y = 0
# obj_z = 0
# ibj_x = 0
# ibj_y = 0
# ibj_z = 0
# lbj_x = 0
# lbj_y = 0
# lbj_z = 0
instance = Vehicle(
    np.array([1121.36 + strut_x, 373.90 + strut_y, 1464.24 + strut_z]),
    np.array([980.4 + obj_x, 496 + obj_y, 1105.4 + obj_z]),
    np.array([996.64 + ibj_x, 203.05 + ibj_y, 1113.44 + ibj_z]),
    np.array([1050.00, 550.00, 1000.00]),
    np.array([1050 + lbj_x, 504.93 + lbj_y, 937.63 + lbj_z]),
    r_La = np.array([950, 233.31, 961.24]),
    r_Lb = np.array([1150, 233.31, 961.24]),
    r_strut = np.array([1064.03, 451.65, 1074.02]),
    GVW = 530.53,
    b = 931.87,
    CG_height = 209.08,
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