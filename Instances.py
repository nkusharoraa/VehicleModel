import numpy as np
from VehicleModel4Wh import Vehicle

class QUTE:
    def __init__(self):
        self.slr = 226
        self.dlr = 241
        self.initial_camber = 0.4
        self.toe_in = 0.4
        self.tw = 1143
        self.wb = 1925
        self.wr_front = 1.2
        self.wr_rear = 3
        self.tire_stiffness_front = 220
        self.tire_stiffness_rear = 220
        self.pinion = 6
        self.tirep = 30
        self.dila = -45
        self.linkage_effort = 1.7
        self.linkage_kpm = 8.5


QUTE_CNG35_Kerb = Vehicle(
    np.array([1054.09, 424.19, 1232.43]),  # Upper Ball Joint (UBJ), Top Strut Mount for MacPherson.
    np.array([976.99, 413.12, 1267.05]),   # OBJ of Tie Rod to make AB as Steering Arm.
    np.array([1130.93, 20.67, 1316.84]),   # IBJ of Tie Rod (Rack Vector Definition point) to make BC as Tie Rod.
    np.array([997.96, 571.50, 953.90]),    # Wheel Centre.
    np.array([990.98, 500.68, 917.52]),    # Lower Ball Joint (LBJ) to make KA as Kingpin Axis.
    226,                                   # SLR
    241,                                   # DLR
    0.4,                                   # Initial_Camber
    0.4,                                   # Initial_Toe
    1143,                                  # Track width
    1925,                                  # Wheelbase
    542.68,                                # GVW
    763.95,                                # b, CG from the rear axle
    330.72,                                # CG height from the wheel centre
    1.2,                                   # Wheel Rate Front
    3,                                     # Wheel Rate Rear
    220,                                   # Tire Stiffness Front
    220,                                   # Tire Stiffness Rear
    6,                                     # Pinion Radius
    30,                                    # Tire Pressure
    -46,                                   # dila, designed inner lock angle
    np.array([1322.30, 515.20, 965.00]),   # LCA Bush 1
    np.array([1322.30, 262.79, 965.00]),   # LCA Bush 2
    r_Ua=np.array([1240.13, 401.44, 1283.81]),  # UCA Bush 1
    r_Ub=np.array([1240.13, 497.44, 1283.81]),   # UCA Bush 2
    linkage_effort = 1.7,
    linkage_kpm = 8.5,
    tiredata = np.array([0.6973898421665277, 0.7700506688653974, 1.9320119069166874, 0.7266206756744256, 1.6523916932662972, 1.841498219515277, 0.3383790728230551, 2.0001252930291913]),
    CF_Loads = np.array([0, 125, 175, 200, 225, 250, 290, 325, 500]),
    CF_Stiffnessrad = np.array([15853.74219, 15853.74219,	21845.73481, 23885.46456, 25281.18975, 26546.28056, 28205.56634, 29273.55967, 29273.55967 + 175*(29273.55967-28205.56634)/35]),
    CF_pneumatictrail = np.array([0, 0.006427979,	0.013799826, 0.018545748 , 0.020164369, 0.020624733, 0.024673851, 0.028284815, 0.028284815 + 175*(0.028284815-0.024673851)/35]) #MRF 135/70 R12 
)
QUTE_CNG60_Kerb = Vehicle(
    np.array([1056.44, 424.19, 1224.59]),  # Upper Ball Joint (UBJ), Top Strut Mount for MacPherson.
    np.array([980.09, 413.01, 1257.99]),   # OBJ of Tie Rod to make AB as Steering Arm.
    np.array([1130.93, 20.67, 1316.84]),   # IBJ of Tie Rod (Rack Vector Definition point) to make BC as Tie Rod.
    np.array([999.74, 570.00, 946.28]),    # Wheel Centre.
    np.array([992.16, 500.68, 909.91]),    # Lower Ball Joint (LBJ) to make KA as Kingpin Axis.
    226,                                   # SLR
    241,                                   # DLR
    0.4,                                   # Initial_Camber
    0.4,                                   # Initial_Toe
    1143,                                  # Track width
    1925,                                  # Wheelbase
    606.16,                                # GVW
    799.72,                                # b, CG from the rear axle
    304.33,                                # CG height from the wheel centre
    1.2,                                   # Wheel Rate Front
    3,                                     # Wheel Rate Rear
    220,                                   # Tire Stiffness Front
    220,                                   # Tire Stiffness Rear
    6,                                     # Pinion Radius
    30,                                    # Tire Pressure
    -46,                                   # dila, designed inner lock angle
    np.array([1322.30, 515.20, 965.00]),   # LCA Bush 1
    np.array([1322.30, 262.80, 965.00]),   # LCA Bush 2
    r_Ua=np.array([1240.08, 401.44, 1283.95]),  # UCA Bush 1
    r_Ub=np.array([1240.08, 497.44, 1283.95]),   # UCA Bush 2
    linkage_effort = 1.7,
    linkage_kpm = 8.5,
    tiredata = np.array([0.6973898421665277, 0.7700506688653974, 1.9320119069166874, 0.7266206756744256, 1.6523916932662972, 1.841498219515277, 0.3383790728230551, 2.0001252930291913]),
    CF_Loads = np.array([0, 125, 175, 200, 225, 250, 290, 325, 500]),
    CF_Stiffnessrad = np.array([15853.74219, 15853.74219,	21845.73481, 23885.46456, 25281.18975, 26546.28056, 28205.56634, 29273.55967, 29273.55967 + 175*(29273.55967-28205.56634)/35]),
    CF_pneumatictrail = np.array([0, 0.006427979,	0.013799826, 0.018545748 , 0.020164369, 0.020624733, 0.024673851, 0.028284815, 0.028284815 + 175*(0.028284815-0.024673851)/35]) #MRF 135/70 R12 
)