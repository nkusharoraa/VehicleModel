import numpy as np
from VehicleModel4Wh import Vehicle
from Instances.base import QUTE, MRF13570R1269S
currveh = QUTE()
currtire = MRF13570R1269S()
instance = Vehicle(
    np.array([1054.09, 424.19, 1232.43]),
    np.array([976.99, 413.12, 1267.05]),
    np.array([1130.93, 20.67, 1316.84]),
    np.array([997.96, 571.50, 953.90]),
    np.array([990.98, 500.68, 917.52]),
    r_La = np.array([1322.30, 515.20, 965.00]),
    r_Lb = np.array([1322.30, 262.79, 965.00]),
    r_Ua = np.array([1240.13, 401.44, 1283.81]),
    r_Ub = np.array([1240.13, 497.44, 1283.81]),
    GVW = 542.68,
    b = 763.95,
    CG_height = 330.72,
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