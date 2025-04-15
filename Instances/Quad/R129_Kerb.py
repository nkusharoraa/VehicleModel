import numpy as np
from VehicleModel4Wh import Vehicle
from Instances.Quad.base import R129, CONTINENTAL12580R13
currveh = R129()
currtire = CONTINENTAL12580R13()
# Coordinates used in R129 vehicle computations

instance = Vehicle(
    np.array([1073.41, 373.90, 1404.88]), # Top Strut Mount
    np.array([930.40, 491.00, 1046.35]), #  Tierod Outer Ball Joint
    np.array([946.64, 203.05, 1054.39]), # Tierod Inner Ball Joint
    np.array([1000.00, 550.00, 940.95]), # Wheel Center
    np.array([990.00, 504.93, 878.58]), # Lower Ball Joint
    r_La = np.array([890.00, 233.31, 902.19]), # Lower A-Arm Bush Front
    r_Lb = np.array([1090.00, 233.31, 902.19]), # Lower A-Arm Bush Rear
    r_strut = np.array([1014.03, 451.65, 1014.97]), # Lower Strut Mount
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