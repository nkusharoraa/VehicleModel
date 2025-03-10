import numpy as np
from VehicleModel4Wh import Vehicle

# --- All the Inputs for the Vehicle Class ---
# Upper Ball Joint (UBJ), Top Strut Mount for MacPherson.
# OBJ of Tie Rod to make AB as Steering Arm.
# --- List to be completed ---

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
class MRF13570R1269S:
    def __init__(self):
        self.tiredata = np.array([0.697389842166, 0.1120749440478134, 17.8337673155644, 0.4054933824758519, 0.25184969239087557, 5.904032519832173, 0.5968391994177625, 0.309857379732586 ])
        self.CF_Loads = np.array([0, 150, 200, 250, 500])
        self.CF_Stiffnessrad = np.array([0, 20234.57749,	23031.75745, 24629.16378, 24629.16378 + 250*(24629.16378-23031.75745)/50])
        self.pneumatictrail = np.array([0, 0.011909253, 0.018484467, 0.023331694, 0.023331694 + 250*(0.023331694-0.018484467)/50])

objQUTE = QUTE()
objMRF13570R1269S = MRF13570R1269S()

QUTE_CNG35_Kerb = Vehicle(
    np.array([1054.09, 424.19, 1232.43]),  # Upper Ball Joint (UBJ), Top Strut Mount for MacPherson.
    np.array([976.99, 413.12, 1267.05]),   # OBJ of Tie Rod to make AB as Steering Arm.
    np.array([1130.93, 20.67, 1316.84]),   # IBJ of Tie Rod (Rack Vector Definition point) to make BC as Tie Rod.
    np.array([997.96, 571.50, 953.90]),    # Wheel Centre.
    np.array([990.98, 500.68, 917.52]),    # Lower Ball Joint (LBJ) to make KA as Kingpin Axis.
    r_La = np.array([1322.30, 515.20, 965.00]),   # LCA Bush 1
    r_Lb = np.array([1322.30, 262.79, 965.00]),   # LCA Bush 2
    r_Ua=np.array([1240.13, 401.44, 1283.81]),  # UCA Bush 1
    r_Ub=np.array([1240.13, 497.44, 1283.81]),   # UCA Bush 2
    GVW = 542.68,                                # GVW
    b = 763.95,                                # b, CG from the rear axle
    CG_height = 330.72,                                # CG height from the wheel centre
    slr = objQUTE.slr,                                    # SLR
    dlr = objQUTE.dlr,                                    # DLR
    initial_camber = objQUTE.initial_camber,              # Initial_Camber
    toe_in = objQUTE.toe_in,                              # Initial_Toe
    tw = objQUTE.tw,                                      # Track width
    wb = objQUTE.wb,                                      # Wheelbase
    wheel_rate_f = objQUTE.wr_front,                      # Wheel Rate Front
    wheel_rate_r = objQUTE.wr_rear,                       # Wheel Rate Rear
    tire_stiffness_f = objQUTE.tire_stiffness_front,      # Tire Stiffness Front
    tire_stiffness_r = objQUTE.tire_stiffness_rear,       # Tire Stiffness Rear
    pinion = objQUTE.pinion,                              # Pinion Radius
    tirep = objQUTE.tirep,                                # Tire Pressure
    dila = objQUTE.dila,                                  # dila, designed inner lock angle
    linkage_effort = objQUTE.linkage_effort,              # Steering Effort Linkage Friction Estimate from Cascading Activity
    linkage_kpm = objQUTE.linkage_kpm,                    # Multiplied by the Mechanical Advantage
    tiredata = objMRF13570R1269S.tiredata,                # Tire Data - Coefficients of the Magic Formula
    CF_Loads = objMRF13570R1269S.CF_Loads,                # Tire Data - Loads
    CF_Stiffnessrad = objMRF13570R1269S.CF_Stiffnessrad,  # Tire Data - Cornering Stiffness for different Loads
    CF_pneumatictrail = objMRF13570R1269S.pneumatictrail  # Tire Data - Pneumatic Trail for different Loads 
)
QUTE_CNG60_Kerb = Vehicle(
    np.array([1056.44, 424.19, 1224.59]),  # Upper Ball Joint (UBJ), Top Strut Mount for MacPherson.
    np.array([980.09, 413.01, 1257.99]),   # OBJ of Tie Rod to make AB as Steering Arm.
    np.array([1130.93, 20.67, 1316.84]),   # IBJ of Tie Rod (Rack Vector Definition point) to make BC as Tie Rod.
    np.array([999.74, 570.00, 946.28]),    # Wheel Centre.
    np.array([992.16, 500.68, 909.91]),    # Lower Ball Joint (LBJ) to make KA as Kingpin Axis.
    r_La = np.array([1322.30, 515.20, 965.00]),   # LCA Bush 1
    r_Lb = np.array([1322.30, 262.80, 965.00]),   # LCA Bush 2
    r_Ua=np.array([1240.08, 401.44, 1283.95]),  # UCA Bush 1
    r_Ub=np.array([1240.08, 497.44, 1283.95]),   # UCA Bush 2
    GVW = 606.16,                                # GVW
    b = 799.72,                                # b, CG from the rear axle
    CG_height = 304.33,                                # CG height from the wheel centre
    slr = objQUTE.slr,                                    # SLR
    dlr = objQUTE.dlr,                                    # DLR
    initial_camber = objQUTE.initial_camber,              # Initial_Camber
    toe_in = objQUTE.toe_in,                              # Initial_Toe
    tw = objQUTE.tw,                                      # Track width
    wb = objQUTE.wb,                                      # Wheelbase
    wheel_rate_f = objQUTE.wr_front,                      # Wheel Rate Front
    wheel_rate_r = objQUTE.wr_rear,                       # Wheel Rate Rear
    tire_stiffness_f = objQUTE.tire_stiffness_front,      # Tire Stiffness Front
    tire_stiffness_r = objQUTE.tire_stiffness_rear,       # Tire Stiffness Rear
    pinion = objQUTE.pinion,                              # Pinion Radius
    tirep = objQUTE.tirep,                                # Tire Pressure
    dila = objQUTE.dila,                                  # dila, designed inner lock angle
    linkage_effort = objQUTE.linkage_effort,              # Steering Effort Linkage Friction Estimate from Cascading Activity
    linkage_kpm = objQUTE.linkage_kpm,                    # Multiplied by the Mechanical Advantage
    tiredata = objMRF13570R1269S.tiredata,                # Tire Data - Coefficients of the Magic Formula
    CF_Loads = objMRF13570R1269S.CF_Loads,                # Tire Data - Loads
    CF_Stiffnessrad = objMRF13570R1269S.CF_Stiffnessrad,  # Tire Data - Cornering Stiffness for different Loads
    CF_pneumatictrail = objMRF13570R1269S.pneumatictrail  # Tire Data - Pneumatic Trail for different Loads 
)
QUTE_35L_EV_Design = Vehicle(
    np.array([1047.18, 424.19, 1278.59]),                 # Upper Ball Joint (UBJ), Top Strut Mount for MacPherson.
    np.array([970.32, 413.43, 1310.96]),                  # OBJ of Tie Rod to make AB as Steering Arm.
    np.array([1130.93, 20.67, 1316.84]),                  # IBJ of Tie Rod (Rack Vector Definition point) to make BC as Tie Rod.
    np.array([995.03, 570.00, 999.39]),                   # Wheel Centre.
    np.array([987.59, 500.68, 962.99]),                   # Lower Ball Joint (LBJ) to make KA as Kingpin Axis.
    r_La = np.array([1322.29, 515.20, 965.00]),           # LCA Bush 1
    r_Lb = np.array([1322.30, 262.79, 965.00]),           # LCA Bush 2
    r_Ua = np.array([1240.14, 401.44, 1283.86]),          # UCA Bush 1
    r_Ub = np.array([1240.14, 497.44, 1283.86]),          # UCA Bush 2
    GVW = 1073.67,                                        # GVW
    b = 721.05,                                           # b, CG from the rear axle
    CG_height = 395.12,                                   # CG height from the wheel centre
    slr = objQUTE.slr,                                    # SLR
    dlr = objQUTE.dlr,                                    # DLR
    initial_camber = objQUTE.initial_camber,              # Initial_Camber
    toe_in = objQUTE.toe_in,                              # Initial_Toe
    tw = objQUTE.tw,                                      # Track width
    wb = objQUTE.wb,                                      # Wheelbase
    wheel_rate_f = objQUTE.wr_front,                      # Wheel Rate Front
    wheel_rate_r = objQUTE.wr_rear,                       # Wheel Rate Rear
    tire_stiffness_f = objQUTE.tire_stiffness_front,      # Tire Stiffness Front
    tire_stiffness_r = objQUTE.tire_stiffness_rear,       # Tire Stiffness Rear
    pinion = objQUTE.pinion,                              # Pinion Radius
    tirep = objQUTE.tirep,                                # Tire Pressure
    dila = objQUTE.dila,                                  # dila, designed inner lock angle
    linkage_effort = objQUTE.linkage_effort,              # Steering Effort Linkage Friction Estimate from Cascading Activity
    linkage_kpm = objQUTE.linkage_kpm,                    # Multiplied by the Mechanical Advantage
    tiredata = objMRF13570R1269S.tiredata,                # Tire Data - Coefficients of the Magic Formula
    CF_Loads = objMRF13570R1269S.CF_Loads,                # Tire Data - Loads
    CF_Stiffnessrad = objMRF13570R1269S.CF_Stiffnessrad,  # Tire Data - Cornering Stiffness for different Loads
    CF_pneumatictrail = objMRF13570R1269S.pneumatictrail  # Tire Data - Pneumatic Trail for different Loads 
)