import numpy as np
from VehicleModel4Wh import Vehicle
from Instances.base import QUTE, MRF13570R1269S
currveh = QUTE()
currtire = MRF13570R1269S()
instance = Vehicle(
    np.array([1047.51, 424.19, 1271.36]),
    np.array([970.68, 413.41, 1303.77]),
    np.array([1130.93, 20.67, 1316.84]),
    np.array([995.16, 570.00, 992.20]),
    np.array([987.72, 500.68, 955.80]),
    r_La = np.array([1322.30, 515.20, 965.00]),
    r_Lb = np.array([1322.30, 262.79, 965.00]),
    r_Ua = np.array([1240.14, 401.44, 1283.86]),
    r_Ub = np.array([1240.14, 497.44, 1283.86]),
    GVW = 904.76,
    b = 750.03,
    CG_height = 322.29,
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
    linkage_effort = currveh.linkage_effort,
    linkage_kpm = currveh.linkage_kpm,
    tiredata = currtire.tiredata,
    CF_Loads = currtire.CF_Loads,
    CF_Stiffnessrad = currtire.CF_Stiffnessrad,
    CF_pneumatictrail = currtire.pneumatictrail
)