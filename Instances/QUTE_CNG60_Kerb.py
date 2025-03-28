import numpy as np
from VehicleModel4Wh import Vehicle
from Instances.base import QUTE, MRF13570R1269S
currveh = QUTE()
currtire = MRF13570R1269S()
instance = Vehicle(
    np.array([1056.44, 424.19, 1224.59]),
    np.array([980.09, 413.01, 1257.99]),
    np.array([1130.93, 20.67, 1316.84]),
    np.array([999.74, 570.00, 946.28]),
    np.array([992.16, 500.68, 909.91]),
    r_La = np.array([1322.30, 515.20, 965.00]),
    r_Lb = np.array([1322.30, 262.80, 965.00]),
    r_Ua = np.array([1240.08, 401.44, 1283.95]),
    r_Ub = np.array([1240.08, 497.44, 1283.95]),
    GVW = 606.16,
    b = 799.72,
    CG_height = 304.33,
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