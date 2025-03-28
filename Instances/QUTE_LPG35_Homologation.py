import numpy as np
from VehicleModel4Wh import Vehicle
from Instances.base import QUTE, MRF13570R1269S
currveh = QUTE()
currtire = MRF13570R1269S()
instance = Vehicle(
    np.array([1048.34, 424.19, 1262.10]),
    np.array([971.56, 413.37, 1294.62]),
    np.array([1130.44, 20.00, 1316.17]),
    np.array([995.55, 570.00, 983.02]),
    np.array([988.10, 500.68, 946.63]),
    r_La = np.array([1322.30, 515.20, 965.00]),
    r_Lb = np.array([1322.30, 262.79, 965.00]),
    r_Ua = np.array([1240.13, 401.44, 1283.81]),
    r_Ub = np.array([1240.13, 497.44, 1283.81]),
    GVW = 798,
    b = 682.03,
    CG_height = 348.14,
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
    linkage_kpm = currveh.linkage_kpm,
    tiredata = currtire.tiredata,
    CF_Loads = currtire.CF_Loads,
    CF_Stiffnessrad = currtire.CF_Stiffnessrad,
    CF_pneumatictrail = currtire.pneumatictrail
)