import numpy as np
from VehicleModelModular import Vehicle
from Instances.Quad.base import Modular, MRF13570R1269S
currveh = Modular()
currtire = MRF13570R1269S()
instance = Vehicle(
    np.array([1311.64, 231.50, 1498.31]), #r_A -  Top of the Kingpin
    np.array([1210.46, 245, 1588.79]), # Tierod Outer Ball Joint
    # np.array([984.75, 20.67, 1103.56]), # Tierod Inner Ball Joint
    np.array([1000, 227.5, 859]), # Wheel Center
    np.array([1216.94, 231.50, 1238.19]), # Bottom of the Kingpin
    pivot1= np.array([914.05, 103.28, 901.67]), #r_O - Pivot Point 1
    pivot2= np.array([913.87, 134.28, 901.16]), #r_O - Pivot Point 2
    cibj = np.array([1313.43, -268.07, 1678.82]), # Column Inner Ball Joint
    cobj = np.array([1187.21, 249.58, 1649.93]), # Column Outer Ball Joint
    s1 = np.array([1643.87, -265.00, 1800.00]), # Steering Shaft Top
    s2 = np.array([1444.32, -265.00, 1644.09]), # Steering Shaft Bottom
    GVW =539.5,# Vehicle Weight in kg
    b = 2445-1454,
    CG_height = 322.73,# CG height from the wheel centre
    slr = currveh.slr,
    dlr = currveh.dlr,
    initial_camber = currveh.initial_camber,
    toe_in = currveh.toe_in,
    twf = currveh.twf,
    twr= currveh.twr,
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