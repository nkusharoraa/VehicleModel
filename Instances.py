import numpy as np
from VehicleModel4Wh import Vehicle

# --- Helper Classes for Vehicle Inputs ---
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
        self.dila = -46
        self.linkage_effort = 1.7
        self.linkage_kpm = 8.5

class R129:
    def __init__(self):
        self.slr = 243
        self.dlr = 257
        self.initial_camber = 0
        self.toe_in = 0
        self.tw = 1100
        self.wb = 1930
        self.wr_front = 1.2
        self.wr_rear = 2
        self.tire_stiffness_front = 220
        self.tire_stiffness_rear = 220
        self.pinion = 6
        self.tirep = 30
        self.dila = -45
        self.linkage_effort = 1.36
        self.linkage_kpm = 8.16

class MRF13570R1269S:
    def __init__(self):
        self.tiredata = np.array([
            0.697389842166, 0.1120749440478134, 17.8337673155644,
            0.4054933824758519, 0.25184969239087557, 5.904032519832173,
            0.5968391994177625, 0.309857379732586
        ])
        self.CF_Loads = np.array([0, 150, 200, 250, 500])
        self.CF_Stiffnessrad = np.array([
            0, 20234.57749, 23031.75745, 24629.16378,
            24629.16378 + 250*(24629.16378-23031.75745)/50
        ])
        self.pneumatictrail = np.array([
            0, 0.011909253, 0.018484467, 0.023331694,
            0.023331694 + 250*(0.023331694-0.018484467)/50
        ])

class CONTINENTAL12580R13:
    def __init__(self):
        self.tiredata = np.array([
            0.5094636099593582, 0.1120749440478134, 17.8337673155644,
            0.4054933824758519, 0.25184969239087557, 5.904032519832173,
            0.5968391994177625, 0.309857379732586
        ])
        self.CF_Loads = np.array([0, 150, 200, 250, 500])
        self.CF_Stiffnessrad = np.array([
            0, 20234.57749, 23031.75745, 24629.16378,
            24629.16378 + 250*(24629.16378-23031.75745)/50
        ])
        self.pneumatictrail = np.array([
            0, 0.011909253, 0.018484467, 0.023331694,
            0.023331694 + 250*(0.023331694-0.018484467)/50
        ])

# --- Cache common objects so they are created only once ---
_common_objs = None
def get_common_objs():
    global _common_objs
    if _common_objs is None:
        _common_objs = {
            'objQUTE': QUTE(),
            'objR129': R129(),
            'objMRF13570R1269S': MRF13570R1269S(),
            'objCONTINENTAL12580R13': CONTINENTAL12580R13()
        }
    return _common_objs

# --- Lazy Vehicle Instance Creation Functions ---
def get_QUTE_CNG35_Kerb():
    objs = get_common_objs()
    return Vehicle(
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
        slr = objs['objQUTE'].slr,
        dlr = objs['objQUTE'].dlr,
        initial_camber = objs['objQUTE'].initial_camber,
        toe_in = objs['objQUTE'].toe_in,
        tw = objs['objQUTE'].tw,
        wb = objs['objQUTE'].wb,
        wheel_rate_f = objs['objQUTE'].wr_front,
        wheel_rate_r = objs['objQUTE'].wr_rear,
        tire_stiffness_f = objs['objQUTE'].tire_stiffness_front,
        tire_stiffness_r = objs['objQUTE'].tire_stiffness_rear,
        pinion = objs['objQUTE'].pinion,
        tirep = objs['objQUTE'].tirep,
        dila = objs['objQUTE'].dila,
        linkage_effort = objs['objQUTE'].linkage_effort,
        linkage_kpm = objs['objQUTE'].linkage_kpm,
        tiredata = objs['objMRF13570R1269S'].tiredata,
        CF_Loads = objs['objMRF13570R1269S'].CF_Loads,
        CF_Stiffnessrad = objs['objMRF13570R1269S'].CF_Stiffnessrad,
        CF_pneumatictrail = objs['objMRF13570R1269S'].pneumatictrail
    )
def get_QUTE_CNG60_Kerb():
    objs = get_common_objs()
    return Vehicle(
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
        slr = objs['objQUTE'].slr,
        dlr = objs['objQUTE'].dlr,
        initial_camber = objs['objQUTE'].initial_camber,
        toe_in = objs['objQUTE'].toe_in,
        tw = objs['objQUTE'].tw,
        wb = objs['objQUTE'].wb,
        wheel_rate_f = objs['objQUTE'].wr_front,
        wheel_rate_r = objs['objQUTE'].wr_rear,
        tire_stiffness_f = objs['objQUTE'].tire_stiffness_front,
        tire_stiffness_r = objs['objQUTE'].tire_stiffness_rear,
        pinion = objs['objQUTE'].pinion,
        tirep = objs['objQUTE'].tirep,
        dila = objs['objQUTE'].dila,
        linkage_effort = objs['objQUTE'].linkage_effort,
        linkage_kpm = objs['objQUTE'].linkage_kpm,
        tiredata = objs['objMRF13570R1269S'].tiredata,
        CF_Loads = objs['objMRF13570R1269S'].CF_Loads,
        CF_Stiffnessrad = objs['objMRF13570R1269S'].CF_Stiffnessrad,
        CF_pneumatictrail = objs['objMRF13570R1269S'].pneumatictrail
    )
def get_QUTE_35L_EV_Design():
    objs = get_common_objs()
    return Vehicle(
        np.array([1047.18, 424.19, 1278.59]),
        np.array([970.32, 413.43, 1310.96]),
        np.array([1130.93, 20.67, 1316.84]),
        np.array([995.03, 570.00, 999.39]),
        np.array([987.59, 500.68, 962.99]),
        r_La = np.array([1322.29, 515.20, 965.00]),
        r_Lb = np.array([1322.30, 262.79, 965.00]),
        r_Ua = np.array([1240.14, 401.44, 1283.86]),
        r_Ub = np.array([1240.14, 497.44, 1283.86]),
        GVW = 1073.67,
        b = 721.05,
        CG_height = 395.12,
        slr = objs['objQUTE'].slr,
        dlr = objs['objQUTE'].dlr,
        initial_camber = objs['objQUTE'].initial_camber,
        toe_in = objs['objQUTE'].toe_in,
        tw = objs['objQUTE'].tw,
        wb = objs['objQUTE'].wb,
        wheel_rate_f = objs['objQUTE'].wr_front,
        wheel_rate_r = objs['objQUTE'].wr_rear,
        tire_stiffness_f = objs['objQUTE'].tire_stiffness_front,
        tire_stiffness_r = objs['objQUTE'].tire_stiffness_rear,
        pinion = objs['objQUTE'].pinion,
        tirep = objs['objQUTE'].tirep,
        dila = objs['objQUTE'].dila,
        linkage_effort = objs['objQUTE'].linkage_effort,
        linkage_kpm = objs['objQUTE'].linkage_kpm,
        tiredata = objs['objMRF13570R1269S'].tiredata,
        CF_Loads = objs['objMRF13570R1269S'].CF_Loads,
        CF_Stiffnessrad = objs['objMRF13570R1269S'].CF_Stiffnessrad,
        CF_pneumatictrail = objs['objMRF13570R1269S'].pneumatictrail
    )
def get_QUTE_LPG35_Homologation():
    objs = get_common_objs()
    return Vehicle(
        np.array([1048.34, 424.19, 1262.10]),
        np.array([971.56, 413.37, 1294.62]),
        np.array([1130.44, 20.00, 1316.17]),
        np.array([995.55, 570.00, 983.02]),
        np.array([988.10, 500.68, 946.63]),
        r_La = np.array([1322.30, 515.20, 965.00]),
        r_Lb = np.array([1322.30, 262.79, 965.00]),
        r_Ua = np.array([1240.13, 401.44, 1283.81]),
        r_Ub = np.array([1240.13, 497.44, 1283.81]),
        GVW = 814,
        b = 678.74,
        CG_height = 348.14,
        slr = objs['objQUTE'].slr,
        dlr = objs['objQUTE'].dlr,
        initial_camber = objs['objQUTE'].initial_camber,
        toe_in = objs['objQUTE'].toe_in,
        tw = objs['objQUTE'].tw,
        wb = objs['objQUTE'].wb,
        wheel_rate_f = objs['objQUTE'].wr_front,
        wheel_rate_r = objs['objQUTE'].wr_rear,
        tire_stiffness_f = objs['objQUTE'].tire_stiffness_front,
        tire_stiffness_r = objs['objQUTE'].tire_stiffness_rear,
        pinion = objs['objQUTE'].pinion,
        tirep = objs['objQUTE'].tirep,
        dila = objs['objQUTE'].dila,
        linkage_effort = objs['objQUTE'].linkage_effort,
        linkage_kpm = objs['objQUTE'].linkage_kpm,
        tiredata = objs['objMRF13570R1269S'].tiredata,
        CF_Loads = objs['objMRF13570R1269S'].CF_Loads,
        CF_Stiffnessrad = objs['objMRF13570R1269S'].CF_Stiffnessrad,
        CF_pneumatictrail = objs['objMRF13570R1269S'].pneumatictrail
    )
def get_QUTE_LPG35_Design():
    objs = get_common_objs()
    return Vehicle(
        np.array([1047.43, 424.19, 1272.65]),  # Upper Ball Joint (UBJ), Top Strut Mount for MacPherson.
        np.array([970.60, 413.41, 1305.06]),   # OBJ of Tie Rod to make AB as Steering Arm.
        np.array([1130.44, 20.00, 1316.17]),   # IBJ of Tie Rod (Rack Vector Definition point) to make BC as Tie Rod.
        np.array([995.12, 570.00, 993.48]),    # Wheel Centre.
        np.array([987.68, 500.68, 957.08]),    # Lower Ball Joint (LBJ) to make KA as Kingpin Axis.
        r_La = np.array([1322.30, 515.20, 965.00]),   # LCA Bush 1
        r_Lb = np.array([1322.30, 262.79, 965.00]),   # LCA Bush 2
        r_Ua = np.array([1240.14, 401.44, 1283.86]),  # UCA Bush 1
        r_Ub = np.array([1240.14, 497.44, 1283.86]),  # UCA Bush 2
        GVW = 995,                                   # GVW
        b = 692.33,                                # b, CG from the rear axle
        CG_height = 419.75,                                # CG height from the wheel centre
        slr = objs['objQUTE'].slr,
        dlr = objs['objQUTE'].dlr,
        initial_camber = objs['objQUTE'].initial_camber,
        toe_in = objs['objQUTE'].toe_in,
        tw = objs['objQUTE'].tw,
        wb = objs['objQUTE'].wb,
        wheel_rate_f = objs['objQUTE'].wr_front,
        wheel_rate_r = objs['objQUTE'].wr_rear,
        tire_stiffness_f = objs['objQUTE'].tire_stiffness_front,
        tire_stiffness_r = objs['objQUTE'].tire_stiffness_rear,
        pinion = objs['objQUTE'].pinion,
        tirep = objs['objQUTE'].tirep,
        dila = objs['objQUTE'].dila,
        linkage_effort = objs['objQUTE'].linkage_effort,
        linkage_kpm = objs['objQUTE'].linkage_kpm,
        tiredata = objs['objMRF13570R1269S'].tiredata,
        CF_Loads = objs['objMRF13570R1269S'].CF_Loads,
        CF_Stiffnessrad = objs['objMRF13570R1269S'].CF_Stiffnessrad,
        CF_pneumatictrail = objs['objMRF13570R1269S'].pneumatictrail
    )
def get_QUTE_35L_EV_Homologation():
    objs = get_common_objs()
    return Vehicle(
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
        slr = objs['objQUTE'].slr,
        dlr = objs['objQUTE'].dlr,
        initial_camber = objs['objQUTE'].initial_camber,
        toe_in = objs['objQUTE'].toe_in,
        tw = objs['objQUTE'].tw,
        wb = objs['objQUTE'].wb,
        wheel_rate_f = objs['objQUTE'].wr_front,
        wheel_rate_r = objs['objQUTE'].wr_rear,
        tire_stiffness_f = objs['objQUTE'].tire_stiffness_front,
        tire_stiffness_r = objs['objQUTE'].tire_stiffness_rear,
        pinion = objs['objQUTE'].pinion,
        tirep = objs['objQUTE'].tirep,
        dila = objs['objQUTE'].dila,
        linkage_effort = objs['objQUTE'].linkage_effort,
        linkage_kpm = objs['objQUTE'].linkage_kpm,
        tiredata = objs['objMRF13570R1269S'].tiredata,
        CF_Loads = objs['objMRF13570R1269S'].CF_Loads,
        CF_Stiffnessrad = objs['objMRF13570R1269S'].CF_Stiffnessrad,
        CF_pneumatictrail = objs['objMRF13570R1269S'].pneumatictrail
    )

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

def get_R129_Design():
    objs = get_common_objs()
    return Vehicle(
        np.array([1121.36 + strut_x, 373.90 + strut_y, 1464.24 + strut_z]),
        np.array([981.89 + obj_x, 495.46 + obj_y, 1134.17 + obj_z]),
        np.array([996.64 + ibj_x, 203.05 + ibj_y, 1113.44 + ibj_z]),
        np.array([1050.85, 550.26, 1028.75]),
        np.array([1050 + lbj_x, 505.92 + lbj_y, 965.88 + lbj_z]),
        r_La = np.array([950, 233.31, 961.24]),
        r_Lb = np.array([1150, 233.31, 961.24]),
        r_strut = np.array([1064.96, 450.97, 1101.50]),
        GVW = 750.53,
        b = 848.94,
        CG_height = 254.63,
        slr = objs['objR129'].slr,
        dlr = objs['objR129'].dlr,
        initial_camber = objs['objR129'].initial_camber,
        toe_in = objs['objR129'].toe_in,
        tw = objs['objR129'].tw,
        wb = objs['objR129'].wb,
        wheel_rate_f = objs['objR129'].wr_front,
        wheel_rate_r = objs['objR129'].wr_rear,
        tire_stiffness_f = objs['objR129'].tire_stiffness_front,
        tire_stiffness_r = objs['objR129'].tire_stiffness_rear,
        pinion = objs['objR129'].pinion,
        tirep = objs['objR129'].tirep,
        dila = objs['objR129'].dila,
        linkage_effort = objs['objR129'].linkage_effort,
        linkage_kpm = objs['objR129'].linkage_kpm,
        tiredata = objs['objCONTINENTAL12580R13'].tiredata,
        CF_Loads = objs['objCONTINENTAL12580R13'].CF_Loads,
        CF_Stiffnessrad = objs['objCONTINENTAL12580R13'].CF_Stiffnessrad,
        CF_pneumatictrail = objs['objCONTINENTAL12580R13'].pneumatictrail
    )
def get_R129_Homologation():
    objs = get_common_objs()
    return Vehicle(
        np.array([1121.36 + strut_x, 373.90 + strut_y, 1464.24 + strut_z]),
        np.array([981.15 + obj_x, 496.08 + obj_y, 1119.71 + obj_z]),
        np.array([996.64 + ibj_x, 203.05 + ibj_y, 1113.44 + ibj_z]),
        np.array([1050.46,	550.47,	1014.32]),
        np.array([1050 + lbj_x, 505.79 + lbj_y, 951.67 + lbj_z]),
        r_La = np.array([950, 233.31, 961.24]),
        r_Lb = np.array([1150, 233.31, 961.24]),
        r_strut = np.array([1064.96, 454.61, 1087.66]),
        GVW = 677.83,
        b = 893.28,
        CG_height = 243.96,
        slr = objs['objR129'].slr,
        dlr = objs['objR129'].dlr,
        initial_camber = objs['objR129'].initial_camber,
        toe_in = objs['objR129'].toe_in,
        tw = objs['objR129'].tw,
        wb = objs['objR129'].wb,
        wheel_rate_f = objs['objR129'].wr_front,
        wheel_rate_r = objs['objR129'].wr_rear,
        tire_stiffness_f = objs['objR129'].tire_stiffness_front,
        tire_stiffness_r = objs['objR129'].tire_stiffness_rear,
        pinion = objs['objR129'].pinion,
        tirep = objs['objR129'].tirep,
        dila = objs['objR129'].dila,
        linkage_effort = objs['objR129'].linkage_effort,
        linkage_kpm = objs['objR129'].linkage_kpm,
        tiredata = objs['objCONTINENTAL12580R13'].tiredata,
        CF_Loads = objs['objCONTINENTAL12580R13'].CF_Loads,
        CF_Stiffnessrad = objs['objCONTINENTAL12580R13'].CF_Stiffnessrad,
        CF_pneumatictrail = objs['objCONTINENTAL12580R13'].pneumatictrail
    )

def get_R129_Kerb():
    objs = get_common_objs()
    return Vehicle(
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
        slr = objs['objR129'].slr,
        dlr = objs['objR129'].dlr,
        initial_camber = objs['objR129'].initial_camber,
        toe_in = objs['objR129'].toe_in,
        tw = objs['objR129'].tw,
        wb = objs['objR129'].wb,
        wheel_rate_f = objs['objR129'].wr_front,
        wheel_rate_r = objs['objR129'].wr_rear,
        tire_stiffness_f = objs['objR129'].tire_stiffness_front,
        tire_stiffness_r = objs['objR129'].tire_stiffness_rear,
        pinion = objs['objR129'].pinion,
        tirep = objs['objR129'].tirep,
        dila = objs['objR129'].dila,
        linkage_effort = objs['objR129'].linkage_effort,
        linkage_kpm = objs['objR129'].linkage_kpm,
        tiredata = objs['objCONTINENTAL12580R13'].tiredata,
        CF_Loads = objs['objCONTINENTAL12580R13'].CF_Loads,
        CF_Stiffnessrad = objs['objCONTINENTAL12580R13'].CF_Stiffnessrad,
        CF_pneumatictrail = objs['objCONTINENTAL12580R13'].pneumatictrail
    )
# --- Lazy Loading via Module __getattr__ ---
_lazy_instances = {
    "QUTE_CNG35_Kerb": get_QUTE_CNG35_Kerb,
    "QUTE_CNG60_Kerb": get_QUTE_CNG60_Kerb,
    "QUTE_35L_EV_Design": get_QUTE_35L_EV_Design,
    "QUTE_LPG35_Homologation": get_QUTE_LPG35_Homologation,
    "QUTE_35L_EV_Homologation": get_QUTE_35L_EV_Homologation,
    "QUTE_LPG35_Design": get_QUTE_LPG35_Design,
    "R129_Design": get_R129_Design,
    "R129_Homologation": get_R129_Homologation,
    "R129_Kerb": get_R129_Kerb,
}
def __getattr__(name):
    if name in _lazy_instances:
        value = _lazy_instances[name]()
        globals()[name] = value  # cache the computed value
        return value
    raise AttributeError(f"module {__name__} has no attribute {name}")
