import numpy as np

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
        self.I_w = 0.32 # Moment of Inertia of the Parts that rotate about the Kingpin Axis
        self.I_ss = 0.03 # Moment of Inertia of the Steering Parts that rotate about the Steering Wheel Axis

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
        self.I_w = 0.34 # Moment of Inertia of the Parts that rotate about the Kingpin Axis
        self.I_ss = 0.03 # Moment of Inertia of the Steering Parts that rotate about the Steering Wheel Axis

class MRF13570R1269S:
    def __init__(self):
        self.tiredata = np.array([
            0.42655091013115615, 2.2944796384877866, 1.0391976647792596, 0.27382438112470636, 1.837242576093849, 1.7772036056703764, 0.3292803666428915, 1.8407332594187324 
        ])
        self.CF_Loads = np.array([0, 125, 175, 200, 225, 250, 290, 325, 500])
        self.CF_Stiffnessrad = np.array([
            0, 15853.74219, 21845.73481, 23885.46456, 25281.18975, 26546.28056, 28205.56634, 29273.55967,
            29273.55967
        ])
        self.pneumatictrail = np.array([
            0, 0.006427979,0.013799826,	0.018545748, 0.020164369, 0.020624733, 0.024673851, 0.028284815,
            0.028284815
        ])

class CONTINENTAL12580R13:
    def __init__(self):
        self.tiredata = np.array([
            0.5094636099593582, 0.1120749440478134, 17.8337673155644,
            0.4054933824758519, 0.25184969239087557, 5.904032519832173, 0.5968391994177625, 0.309857379732586
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

