import numpy as np
from VehicleModel4Wh import Vehicle
from Instances.base import QUTE, MRF13570R1269S
currveh = QUTE()
currtire = MRF13570R1269S()
instance = Vehicle(
    np.array([1047.43, 424.19, 1272.65]),  # Upper Ball Joint (UBJ), Top Strut Mount for MacPherson.
    np.array([970.60, 413.41, 1305.06]),   # OBJ of Tie Rod to make AB as Steering Arm.
    np.array([1130.44, 20.00, 1316.17]),   # IBJ of Tie Rod (Rack Vector Definition point) to make BC as Tie Rod.
    np.array([995.12, 570.00, 993.48]),    # Wheel Centre.
    np.array([987.68, 500.68, 957.08]),    # Lower Ball Joint (LBJ) to make KA as Kingpin Axis.
    r_La = np.array([1322.30, 515.20, 965.00]),   # LCA Bush 1
    r_Lb = np.array([1322.30, 262.79, 965.00]),   # LCA Bush 2
    r_Ua = np.array([1240.14, 401.44, 1283.86]),  # UCA Bush 1
    r_Ub = np.array([1240.14, 497.44, 1283.86]),  # UCA Bush 2
    GVW = 996,                                   # GVW
    b = 693.71,                                # b, CG from the rear axle
    CG_height = 419.75,                                # CG height from the wheel centre
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