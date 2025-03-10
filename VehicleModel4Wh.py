# Important Packages
import numpy as np
import pandas as pd
from scipy.optimize import fsolve
from scipy.optimize import curve_fit
from scipy.integrate import solve_ivp
from sklearn.linear_model import LinearRegression
from sklearn.neural_network import MLPRegressor
from sklearn.ensemble import RandomForestRegressor
from sklearn.svm import SVR
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.ensemble import HistGradientBoostingRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.metrics import mean_squared_error
from scipy import integrate
from scipy.optimize import minimize, NonlinearConstraint
from scipy.optimize import root_scalar
from scipy.interpolate import interp1d
from openpyxl import load_workbook

# Class Vehicle with all the functions
class Vehicle:
    
    """The Vehicle class encapsulates a variety of methods and properties related to vehicle dynamics, steering, wheel loads, and geometric calculations.
     It leverages numerical methods, regressions, and physical equations to model and analyze vehicle behavior under different conditions.
     The class is designed to provide comprehensive insights into the performance and characteristics of a vehicle's steering and suspension systems.
    """
    # --- Constuctor for Inputs ---

    def __init__(self,    
             r_A: np.array,
             r_B: np.array,
             r_C: np.array,
             r_O: np.array,
             r_K: np.array,
             slr: float,
             dlr: float,
             initial_camber: float,
             toe_in: float,
             tw: float,
             wb: float,
             GVW: float,
             b: float,
             CG_height: float,
             wheel_rate_f: float,
             wheel_rate_r: float,
             tire_stiffness_f: float,
             tire_stiffness_r: float,
             pinion: float,
             tirep: float,
             dila: float,
             r_La: np.array,
             r_Lb: np.array,
             r_strut: np.array = np.array([0, 0, 0]),
             r_Ua: np.array = np.array([0, 0, 0]),
             r_Ub: np.array = np.array([0, 0, 0]),
             mu : float = 0.4,
             g : float = 9.8,
             speed: float = 10.0,
             linkage_effort: float = 1.36, # Nm
             linkage_kpm: float = 8.16, # Nm
             tiredata: np.array =  np.array([0.5094636099593582, 0.1120749440478134, 17.8337673155644, 0.4054933824758519, 0.25184969239087557, 5.904032519832173, 0.5968391994177625, 0.309857379732586 ]),
             CF_Loads: np.array = np.array([0, 150, 200, 250, 500]),
             CF_Stiffnessrad: np.array = np.array([0, 20234.57749,	23031.75745, 24629.16378, 24629.16378 + 250*(24629.16378-23031.75745)/50]),
             CF_pneumatictrail: np.array = np.array([0, 0.011909253,	0.018484467, 0.023331694, 0.023331694 + 250*(0.023331694-0.018484467)/50])): # Continental R13

        # Create static object
        # ulr - (ulr-slr)/load_rating

        self.static = Vehicle.create_object(r_A, r_B, r_C, r_O, r_K, slr, initial_camber, toe_in, tw, wb, GVW, b, 
                                          CG_height, wheel_rate_f, wheel_rate_r, tire_stiffness_f, tire_stiffness_r,
                                          pinion, tirep, dila, r_La, r_Lb, r_strut, r_Ua, r_Ub, mu, g, tiredata, speed)

        # Create dynamic object 
        self.dynamic = Vehicle.create_object(r_A, r_B, r_C, r_O, r_K, dlr, initial_camber, toe_in, tw, wb, GVW, b,
                                           CG_height, wheel_rate_f, wheel_rate_r, tire_stiffness_f, tire_stiffness_r,
                                           pinion, tirep, dila, r_La, r_Lb, r_strut, r_Ua, r_Ub, mu, g, tiredata, speed)
        # Initialize common parameters
        self.CF_Loads = CF_Loads
        self.CF_Stiffnessrad = CF_Stiffnessrad
        self.CF_pneumatictrail = CF_pneumatictrail
        self.dynamic_analysis = 1
        self.model = self.regression_model()
        reference = self.reference()
        self.dynamic_analysis = 0
        self.model = self.regression_model()
        self.rack_stroke = self.rack_vs_road_steer(reference.dila - toe_in)
        self.trainslipangles()
        self.linkage_friction_contribution_on_kpm = linkage_kpm
        self.linkage_friction_contribution_on_steering = linkage_effort   
    @classmethod
    def create_object(cls, r_A, r_B, r_C, r_O, r_K, tire_radius, initial_camber, toe_in, tw, wb, GVW, b, CG_height, 
                    wheel_rate_f, wheel_rate_r, tire_stiffness_f, tire_stiffness_r, pinion, tirep, dila,
                    r_La, r_Lb, r_strut, r_Ua, r_Ub, mu, g, tiredata,speed):
        
        obj = type('VehicleState', (), {})()
        
        # Assign instance variables
        obj.mu = mu
        obj.g = g
        obj.r_A = r_A
        obj.r_B = r_B 
        obj.r_C = r_C
        obj.r_O = r_O
        obj.r_K = r_K
        obj.tire_radius = tire_radius
        obj.initial_camber = initial_camber
        obj.tw = tw
        obj.wb = wb
        obj.GVW = GVW
        obj.b = b
        obj.a = wb - b
        obj.Kf = wheel_rate_f * tire_stiffness_f / (wheel_rate_f + tire_stiffness_f)
        obj.Kr = wheel_rate_r * tire_stiffness_r / (wheel_rate_r + tire_stiffness_r)
        obj.tiredata = tiredata
        obj.pinion = pinion
        obj.tirep = tirep
        obj.dila = dila
        obj.r_La = r_La
        obj.r_Lb = r_Lb
        obj.r_strut = r_strut
        obj.r_Ua = r_Ua
        obj.r_Ub = r_Ub

        # Calculate additional points
        obj.r_D = np.array([obj.r_C[0], 0.00, obj.r_C[2]])
        obj.r_T = np.array([obj.r_O[0], obj.r_O[1] - obj.tire_radius * np.sin(np.radians(obj.initial_camber)),
                            obj.r_O[2] - obj.tire_radius])
        obj.r_O[2] = obj.r_O[2] - obj.tire_radius + obj.tire_radius * np.cos(np.radians(obj.initial_camber))
        obj.r_W = obj.r_T - np.array([np.cos(np.radians(toe_in)), np.sin(np.radians(toe_in)), 0])

        # Calculate KPA
        obj.KPA = (r_A - r_K) / cls.magnitude(r_A - r_K)
        obj.currKPA = (r_A - r_K) / cls.magnitude(r_A - r_K)

        # Initialize arrays
        obj.mindp = 50
        obj.maxdp = 50
        obj.step = 0.1
        obj.dpK = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.dpT = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.dpO = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.dpW = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.dpA = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.dpB = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.dpnewB = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.dpC = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.dpdz = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1)))
        obj.dpfvsa = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.dpsvsa = np.zeros((int(obj.mindp / obj.step + obj.maxdp / obj.step + 1), 3))
        obj.zeropos = int(obj.mindp / obj.step)

        # Set initial positions
        obj.dpK[obj.zeropos] = obj.r_K
        obj.dpO[obj.zeropos] = obj.r_O  
        obj.dpT[obj.zeropos] = obj.r_T
        obj.dpW[obj.zeropos] = obj.r_W
        obj.dpA[obj.zeropos] = obj.r_A
        obj.dpB[obj.zeropos] = obj.r_B
        obj.dpnewB[obj.zeropos] = obj.r_B
        obj.dpC[obj.zeropos] = obj.r_C
        obj.dpdz[obj.zeropos] = 0

        # Calculate angles
        h1 = (obj.KPA - np.dot(obj.KPA, np.array([0, 1, 0])) * np.array([0, 1, 0])) / cls.magnitude(
            obj.KPA - np.dot(obj.KPA, np.array([0, 1, 0])) * np.array([0, 1, 0]))
        h2 = (obj.KPA - np.dot(obj.KPA, np.array([1, 0, 0])) * np.array([1, 0, 0])) / cls.magnitude(
            obj.KPA - np.dot(obj.KPA, np.array([1, 0, 0])) * np.array([1, 0, 0]))
        obj.caster = np.degrees(np.arccos(np.dot(h1, np.array([0, 0, 1]))))
        obj.kpi = np.degrees(np.arccos(np.dot(h2, np.array([0, 0, 1]))))

        # Calculate projection points
        t = (obj.r_T[2] - obj.r_K[2]) / obj.KPA[2]
        obj.r_I = obj.r_K + t * obj.KPA
        obj.r_Aprime = cls.projection(obj.r_A, obj.KPA, obj.r_B)
        obj.r_Iprime = cls.projection(obj.r_A, obj.KPA, obj.r_T)
        obj.r_Iwprime = cls.projection(obj.r_A, obj.KPA, obj.r_W)
        obj.r_Ioprime = cls.projection(obj.r_A, obj.KPA, obj.r_O)

        obj.maxdecimal = int(-np.log10(obj.step))
        obj.conversionstep = int(10**obj.maxdecimal)
        # Initializing additional helper variables and methods
        
        obj.speed = speed*5/18 #m/s
        obj.CG_height = CG_height
        obj.slipangles = np.zeros((50, 2))
        obj.slipangles[0] = np.array([0.0,0.0])
        obj.Flguess = np.zeros((50))
        obj.Frguess = np.zeros((50))
        obj.Rlguess = np.zeros((50))
        obj.Rrguess = np.zeros((50))
        obj.Flguess[0] = obj.GVW*obj.b/(obj.a+obj.b)*0.5
        obj.Frguess[0] = obj.GVW*obj.b/(obj.a+obj.b)*0.5
        obj.Rlguess[0] = obj.GVW*obj.a/(obj.a+obj.b)*0.5
        obj.Rrguess[0] = obj.GVW*obj.a/(obj.a+obj.b)*0.5
        obj.patch_radius_left = 0
        obj.patch_radius_right = 0
        obj.tempdynamicsolution = np.zeros(12)
        obj.tempdynamictheta = 0


        return obj        
    def reference(self):
        if(self.dynamic_analysis == 0):
            reference = self.static
        else:
            reference = self.dynamic
        return reference
    # --- Calculation of instantaneous axis for suspension travel ---
    def fvsa_equations(self, values):
        """
        Calculates the equations for Front View Swing Arm (FVSA) optimization.

        Computes the difference between two vectors based on vehicle geometry and steering parameters
        to find optimal values of `la` and `mu`. Depending on whether `r_strut` is defined, calculates
        equations for suspension parameters affecting FVSA optimization.

        Args:
        values (list or tuple): Contains two float values representing:
            - `la`: Parameter affecting the vector calculation based on current_A and current_K.
            - `mu`: Parameter affecting the vector calculation based on current_K and average of r_La and r_Lb.

        Returns:
        list: A list containing two equations (`eq1` and `eq2`) representing the difference between `l2` and `l1`.
            - `eq1`: Difference in the y-component between `l2` and `l1`.
            - `eq2`: Difference in the z-component between `l2` and `l1`.

        Notes:
        - If `r_strut` is not defined (equal to [0, 0, 0]), calculates `a2` based on average of r_Ua and r_Ub.
        - If `r_strut` is defined, calculates `a2` based on current_A and cross product of r_strut-a1 and [1, 0, 0].
        - `current_A`, `current_K`, and `current_O` are calculated using `self.curr_A`, `self.curr_K`, and `self.curr_O`
        methods respectively, with `self.curr_KPA_angle_for_fvsa` as input.
        """
        reference = self.reference()
        if(reference.r_strut[0] == 0):
            la = values[0]
            mu = values[1]
            current_A = self.curr_A(self.curr_KPA_angle_for_fvsa)
            current_K = self.curr_K(self.curr_KPA_angle_for_fvsa)
            if reference.r_strut[0] == 0:
                # No strut present
                if(np.abs(reference.r_Ua[0] - reference.r_Ub[0])<1 and np.abs(reference.r_Ua[2] - reference.r_Ub[2])<1) :
                    a1 = reference.r_Ua
                    a2 = reference.r_Ub
                    b1 = reference.r_La
                    b2 = reference.r_Lb
                    l1 = a1 + la * (a1 - a2)
                    l2 = b1 + mu * (b1 - b2)
                else:
                    a1 = current_A
                    a2 = (reference.r_Ua + reference.r_Ub) / 2
                    b1 = current_K
                    b2 = (reference.r_La + reference.r_Lb) / 2
                    l1 = a1 + la * (a1 - a2)
                    l2 = b1 + mu * (b1 - b2)  
            l1 = a1 + la*(a1-a2)
            l2 = b1 + mu*(b1-b2)
            eq1 = (l2-l1)[1]
            eq2 = (l2-l1)[2]
        else:
            la = values[0]
            mu = values[1]
            current_A = self.curr_A(self.curr_KPA_angle_for_fvsa)
            current_K = self.curr_K(self.curr_KPA_angle_for_fvsa)
            current_O = self.curr_O(self.curr_KPA_angle_for_fvsa)
            a1 = current_A
            a2 = a1+np.cross(reference.r_strut-a1, np.array([1,0,0]))
            b1 = current_K
            b2 = (reference.r_La+reference.r_Lb)/2
            a2 += 1e-9
            b2 += 1e-9
            l1 = a1 + la*(a1-a2)
            l2 = b1 + mu*(b1-b2)
            eq1 = (l2-l1)[1]
            eq2 = (l2-l1)[2]
        return [eq1,eq2]
    def fvsa_ic(self, curr_KPA_angle):
        """
        Computes the Instantaneous Centers (IC) for the Front View Swing Arm (FVSA) suspension.

        This method calculates the IC based on the current KPA angle and the geometry of the suspension.

        Args:
        curr_KPA_angle (float): Current Kingpin Axis (KPA) angle in degrees.

        Returns:
        ndarray: Coordinates of the Instantaneous Center (IC) in 3D space.

        Notes:
        - Uses numerical root-finding (fsolve) to staticsolve the FVSA equations for la and mu.
        - Handles different configurations based on the presence of a strut.
        """
        reference = self.reference()
        self.curr_KPA_angle_for_fvsa = curr_KPA_angle
        position_to_add = reference.zeropos + int(np.round(curr_KPA_angle, reference.maxdecimal) * reference.conversionstep)
        if(np.abs(reference.dpfvsa[position_to_add][0])>np.abs(reference.dpfvsa[reference.zeropos][0]/10000)):
            return reference.dpfvsa[position_to_add]
        if reference.r_strut[0] == 0:
            # No strut present
            
            current_A = self.curr_A(curr_KPA_angle)
            current_K = self.curr_K(curr_KPA_angle)
            if(np.abs(reference.r_Ua[0] - reference.r_Ub[0])<1 and np.abs(reference.r_Ua[2] - reference.r_Ub[2])<1) :
                reference.dpfvsa[position_to_add] = self.svsa_ic(curr_KPA_angle) + np.array([0,1,0])
                return reference.dpfvsa[position_to_add]
            else:
                la, mu = fsolve(self.fvsa_equations, [0.01, 0.01])
                a1 = current_A
                a2 = (reference.r_Ua + reference.r_Ub) / 2
                b1 = current_K
                b2 = (reference.r_La + reference.r_Lb) / 2
                l1 = a1 + la * (a1 - a2)
                l2 = b1 + mu * (b1 - b2)   
        else:
            # Strut present
            la, mu = fsolve(self.fvsa_equations, [0.01, 0.01])
            current_A = self.curr_A(curr_KPA_angle)
            current_K = self.curr_K(curr_KPA_angle)
            current_O = self.curr_O(curr_KPA_angle)
            a1 = current_A
            a2 = a1 + np.cross(reference.r_strut - a1, np.array([1, 0, 0]))
            b1 = current_K
            b2 = (reference.r_La + reference.r_Lb) / 2
            l1 = a1 + la * (a1 - a2)
            l2 = b1 + mu * (b1 - b2)
        reference.dpfvsa[position_to_add] = (l1 + l2) / 2

        return reference.dpfvsa[position_to_add]
    def svsa_equations(self, values):
        """
        Calculates the Side View Swing Arm (SVSA) suspension equations for finding la and mu.

        This method computes the equations based on the current configuration of the suspension.
        For configurations without a strut, it uses the upper (Ua, Ub) and lower (La, Lb) control arm pivot points.
        For configurations with a strut, it adjusts the calculation based on the strut position relative to the upper pivot.

        Args:
        values (list): List containing la and mu values to staticsolve the equations.

        Returns:
        list: Equations [eq1, eq2] representing the difference between computed lengths l2 and l1 along x and z axes.

        Notes:
        - Uses current KPA angle for calculating current_A.
        - Handles different suspension configurations based on the presence of a strut (r_strut).
        """
        reference = self.reference()
        if reference.r_strut[0] == 0:
            la = values[0]
            mu = values[1]
            # No strut present
            current_A = self.curr_A(self.curr_KPA_angle_for_svsa)
            current_K = self.curr_K(self.curr_KPA_angle_for_svsa)
            if(np.abs(reference.r_Ua[0] - reference.r_Ub[0])<1 and np.abs(reference.r_Ua[2] - reference.r_Ub[2])<1) :
                a1 = current_A
                a2 = (reference.r_Ua + reference.r_Ub) / 2
                b1 = current_K
                b2 = (reference.r_La + reference.r_Lb) / 2
                l1 = a1 + la * (a1 - a2)
                l2 = b1 + mu * (b1 - b2)
            else:
                a1 = reference.r_Ua
                a2 = reference.r_Ub
                b1 = reference.r_La
                b2 = reference.r_Lb
                l1 = a1 + la * (a1 - a2)
                l2 = b1 + mu * (b1 - b2)
            eq1 = (l2 - l1)[0]
            eq2 = (l2 - l1)[2]
        else:
            # Strut present
            la = values[0]
            mu = values[1]
            current_A = self.curr_A(self.curr_KPA_angle_for_svsa)
            a1 = current_A
            a2 = a1 + np.cross(reference.r_strut - a1, np.array([0, 1, 0]))
            b1 = reference.r_La
            b2 = reference.r_Lb
            a2 += 1e-9
            b2 += 1e-9
            l1 = a1 + la * (a1 - a2)
            l2 = b1 + mu * (b1 - b2)
            eq1 = (l2 - l1)[0]
            eq2 = (l2 - l1)[2]
        
        return [eq1, eq2]
    def svsa_ic(self, curr_KPA_angle):
        """
        Computes the Instantaneous Centers (IC) for the Side View Swing Arm (SVSA) suspension.

        This method calculates the IC based on the current configuration of the SVSA suspension.
        If no strut is present, it uses the upper (Ua, Ub) and lower (La, Lb) control arm pivot points.
        If a strut is present, it adjusts the calculation based on the strut position relative to the upper pivot.

        Args:
        curr_KPA_angle (float): Current Kingpin Axis (KPA) angle in radians.

        Returns:
        ndarray: Coordinates of the IC (Instantaneous Center) calculated as the midpoint of lengths l1 and l2.

        Notes:
        - Uses fsolve to staticsolve the svsa_equations for la and mu.
        - Handles different suspension configurations based on the presence of a strut (r_strut).
        """
        reference = self.reference()
        self.curr_KPA_angle_for_svsa = curr_KPA_angle
        position_to_add = reference.zeropos + int(np.round(curr_KPA_angle, reference.maxdecimal) * reference.conversionstep)
        if(reference.dpsvsa[position_to_add][0]>reference.dpsvsa[reference.zeropos][0]/10):
            return reference.dpsvsa[position_to_add]
        if reference.r_strut[0] == 0:
            # No strut present            
            [la, mu] = fsolve(self.svsa_equations, [0.01, 0.01])
            current_A = self.curr_A(self.curr_KPA_angle_for_svsa)
            current_K = self.curr_K(curr_KPA_angle)
            if(np.abs(reference.r_Ua[0] - reference.r_Ub[0])<1 and np.abs(reference.r_Ua[2] - reference.r_Ub[2])<1) :
                a1 = current_A
                a2 = (reference.r_Ua + reference.r_Ub) / 2
                b1 = current_K
                b2 = (reference.r_La + reference.r_Lb) / 2
                l1 = a1 + la * (a1 - a2)
                l2 = b1 + mu * (b1 - b2)
            else:
                a1 = reference.r_Ua
                a2 = reference.r_Ub
                b1 = reference.r_La
                b2 = reference.r_Lb
                l1 = a1 + la * (a1 - a2)
                l2 = b1 + mu * (b1 - b2)
        else:
            # Strut present
            [la, mu] = fsolve(self.svsa_equations, [0.01, 0.01])
            current_A = self.curr_A(self.curr_KPA_angle_for_svsa)
            a1 = current_A
            a2 = a1 + np.cross(reference.r_strut - a1, np.array([0, 1, 0]))
            b1 = reference.r_La
            b2 = reference.r_Lb
            l1 = a1 + la * (a1 - a2)
            l2 = b1 + mu * (b1 - b2)
        reference.dpsvsa[position_to_add] = (l1 + l2) / 2
        
        return reference.dpsvsa[position_to_add]
    def solveK(self, inputval):
        """
        Solves for the position of point K along the suspension axis.

        This method calculates the position of point K based on the input value `t` and the current configuration
        of the suspension. It uses the current orientation and positions of points O, K, and the instantaneous
        centers (ICs) for the FVSA and SVSA suspensions.

        Args:
        inputval (list): Input value `t` as a list where `t = [t_val]`.

        Returns:
        list: A list containing the staticequation `eq1`, representing the error between the current and previous delta_z values
            and the difference between `tempK` and the stored position of K.

        Notes:
        - Uses the current KPA angle (`curr_KPA_angle_for_K`) to calculate positions and orientations.
        - Adjusts positions based on the sign of `curr_KPA_angle_for_K`.
        - Utilizes rotational transformations and delta_z calculations for accuracy.
        """
        reference = self.reference()
        t = inputval[0]
        tempO = self.curr_O(self.curr_KPA_angle_for_K)
        position_to_add = reference.zeropos + int(np.round(self.curr_KPA_angle_for_K, reference.maxdecimal) * reference.conversionstep)
        tempK = Vehicle.rotation(reference.dpK[position_to_add - int(np.sign(self.curr_KPA_angle_for_K))].tolist(),
                                self.fvsa_ic(self.curr_KPA_angle_for_K - np.sign(self.curr_KPA_angle_for_K) * reference.step).tolist(),
                                self.svsa_ic(self.curr_KPA_angle_for_K - np.sign(self.curr_KPA_angle_for_K) * reference.step).tolist(), t)
        eq1 = self.delta_z(self.curr_KPA_angle_for_K) - self.delta_z(self.curr_KPA_angle_for_K - reference.step * np.sign(self.curr_KPA_angle_for_K)) + (tempK - reference.dpK[position_to_add - int(np.sign(self.curr_KPA_angle_for_K))])[2]
        return [eq1]
    def curr_K(self, curr_KPA_angle):
        """
        Computes the position of point K based on the current KPA angle.

        This method calculates the position of point K along the suspension axis based on the provided current
        Kingpin Axis (KPA) angle. If the angle is zero, it returns the initial position of point K. Otherwise,
        it adjusts the position using the stored positions and angles of various suspension components and ICs.

        Args:
        curr_KPA_angle (float): Current Kingpin Axis (KPA) angle in degrees.

        Returns:
        ndarray: The current position of point K in 3D space.

        Notes:
        - If `curr_KPA_angle` is zero, returns `reference.r_K`.
        - Uses the stored positions (`dpK`) and ICs (`fvsa_ic` and `svsa_ic`) to compute the current position of K.
        - Adjusts positions based on the sign of `curr_KPA_angle` for accuracy.
        """
        reference = self.reference()
        if curr_KPA_angle == 0:
            return reference.r_K

        position_to_add = reference.zeropos + int(np.round(curr_KPA_angle, reference.maxdecimal) * reference.conversionstep)

        # Adjust position based on stored data and IC calculations
        if reference.dpK[position_to_add][0] < reference.r_K[0] / 10:
            self.curr_KPA_angle_for_K = curr_KPA_angle
            [t] = fsolve(self.solveK, [0.01])
            reference.dpK[position_to_add] = Vehicle.rotation(reference.dpK[position_to_add - int(np.sign(curr_KPA_angle))].tolist(),
                                                        self.fvsa_ic(curr_KPA_angle - np.sign(curr_KPA_angle) * reference.step).tolist(),
                                                        self.svsa_ic(curr_KPA_angle - np.sign(curr_KPA_angle) * reference.step).tolist(), t)

        return reference.dpK[position_to_add]
    def curr_KPA(self, curr_KPA_angle):
        """
        Computes the current Kingpin Axis (KPA) based on the given KPA angle.

        This method calculates the current Kingpin Axis (KPA) by determining the vector difference between
        the current position of point A (upper ball joint) and point K (lower ball joint) at the given
        Kingpin Axis angle. It normalizes this vector to obtain the direction of the KPA.

        Args:
        curr_KPA_angle (float): The current Kingpin Axis (KPA) angle in degrees.

        Returns:
        ndarray: The normalized direction vector of the current Kingpin Axis (KPA).

        Notes:
        - The method updates the instance variable `reference.currKPA` to the computed KPA direction.
        - Uses the methods `curr_A` and `curr_K` to obtain the current positions of points A and K.
        """
        reference = self.reference()
        t = self.curr_A(curr_KPA_angle) - self.curr_K(curr_KPA_angle)
        m = t / Vehicle.magnitude(t)
        reference.currKPA = m
        return m
    def solveA(self, inputval):
        """
        Solves for the rotation parameter `t` to determine the current position of point A (upper ball joint) at the specified Kingpin Axis (KPA) angle.

        This method computes the rotation parameter `t` needed to align the position of point A (upper ball joint)
        using the Front View Swing Arm (FVSA) and Side View Swing Arm (SVSA) intersection points.
        The method solves for `t` by ensuring the vertical displacement consistency between successive KPA angles.

        Args:
        inputval (list): A list containing the initial guess for the rotation parameter `t`.

        Returns:
        list: A list containing the staticequation residual that needs to be solved to find `t`.

        Notes:
        - The method uses `curr_O` to obtain the current position of point O (wheel center).
        - The position to add is determined based on the current KPA angle.
        - The temporary position of point A (`tempA`) is computed using the `rotation` method.
        - The vertical displacement (`delta_z`) consistency is checked between successive KPA angles.
        """
        reference = self.reference()
        t = inputval[0]
        tempO = self.curr_O(self.curr_KPA_angle_for_A)
        position_to_add = reference.zeropos + int(np.round(self.curr_KPA_angle_for_A, reference.maxdecimal) * reference.conversionstep)
        tempA = Vehicle.rotation(
            reference.dpA[position_to_add - int(np.sign(self.curr_KPA_angle_for_A))].tolist(),
            self.fvsa_ic(self.curr_KPA_angle_for_A - np.sign(self.curr_KPA_angle_for_A) * reference.step).tolist(),
            self.svsa_ic(self.curr_KPA_angle_for_A - np.sign(self.curr_KPA_angle_for_A) * reference.step).tolist(),
            t
        )
        eq1 = self.delta_z(self.curr_KPA_angle_for_A) - self.delta_z(self.curr_KPA_angle_for_A - reference.step * np.sign(self.curr_KPA_angle_for_A)) + (tempA - reference.dpA[position_to_add - int(np.sign(self.curr_KPA_angle_for_A))])[2]
        return [eq1]
    def curr_A(self, curr_KPA_angle):
        """
        Determines the current position of point A (upper ball joint) at the specified Kingpin Axis (KPA) angle.

        This method calculates the position of point A by solving for the rotation parameter `t` if the position
        has not been previously computed for the given KPA angle. It updates the position in the dpA array.

        Args:
        curr_KPA_angle (float): The current Kingpin Axis (KPA) angle in degrees.

        Returns:
        ndarray: The position of point A at the specified KPA angle.

        Notes:
        - If the `curr_KPA_angle` is zero, it returns the initial position `r_A`.
        - The method checks if the position for the given angle has already been computed.
        - If not, it solves for the rotation parameter `t` using the `solveA` method.
        - The position is then updated in the `dpA` array using the `rotation` method.
        """
        reference = self.reference()
        # print(curr_KPA_angle)
        if np.abs(curr_KPA_angle) < 10e-4:    
            return reference.r_A
        position_to_add = reference.zeropos + int(np.round(curr_KPA_angle, reference.maxdecimal) * reference.conversionstep)
        # print(position_to_add)
        # print(reference.dpA[position_to_add][0])
        if reference.dpA[position_to_add][0] < reference.r_A[0] / 100000:
            self.curr_KPA_angle_for_A = curr_KPA_angle
            [t] = fsolve(self.solveA, [0.01])
            reference.dpA[position_to_add] = Vehicle.rotation(
                reference.dpA[position_to_add - int(np.sign(curr_KPA_angle))].tolist(),
                self.fvsa_ic(curr_KPA_angle - np.sign(curr_KPA_angle) * reference.step).tolist(),
                self.svsa_ic(curr_KPA_angle - np.sign(curr_KPA_angle) * reference.step).tolist(),
                t
            )
        return reference.dpA[position_to_add]

# --- Projection of point (x,y,z) on the plane a*x + b*y + c*z = 1 --- 
    def project_points(x, y, z, a, b, c):
        """
        Projects the points with coordinates x, y, z onto the plane
        defined by a*x + b*y + c*z = 1
        """
        vector_norm = a*a + b*b + c*c
        normal_vector = np.array([a, b, c]) / np.sqrt(vector_norm)
        point_in_plane = np.array([a, b, c]) / vector_norm

        points = np.column_stack((x, y, z))
        points_from_point_in_plane = points - point_in_plane
        proj_onto_normal_vector = np.dot(points_from_point_in_plane,
                                        normal_vector)
        proj_onto_plane = (points_from_point_in_plane -
                        proj_onto_normal_vector[:, None]*normal_vector)

        return point_in_plane + proj_onto_plane
    # --- Magnitude of a vector ---
    def magnitude(vector):
        return np.sqrt(sum(pow(element, 2) for element in vector))
    # --- Matrix Multiplication ---
    @staticmethod
    def safe_normalize(U):
        norm = np.linalg.norm(U)
        if norm == 0:
            return np.zeros_like(U)
        return U / norm

    @staticmethod
    def matrix_multiply(*matrices):
        result = matrices[0]
        for matrix in matrices[1:]:
            result = np.dot(result, matrix)
        return result
    # --- Rotaion of point p at an angle t about the axis defined by points x1,x2 ---
    def rotation(p, x1, x2, t):
        theta = np.radians(t)
        p = np.array([[pp] for pp in p] + [[1]])
        x1, y1, z1 = x1
        x2, y2, z2 = x2

        # Define the unit vector U along the axis of rotation
        U = np.array([x2 - x1, y2 - y1, z2 - z1])
        U = Vehicle.safe_normalize(U)
        a, b, c = U

        d = np.sqrt(b**2 + c**2)
        if d == 0:
            d = 1e-9  # Handle case where b and c are both zero to avoid division by zero

        # Translation matrices
        T = np.array([
            [1, 0, 0, -x1],
            [0, 1, 0, -y1],
            [0, 0, 1, -z1],
            [0, 0, 0, 1]
        ])
        T_inv = np.array([
            [1, 0, 0, x1],
            [0, 1, 0, y1],
            [0, 0, 1, z1],
            [0, 0, 0, 1]
        ])

        # Rotation matrices around x, y, and z axes
        R_x = np.array([
            [1, 0, 0, 0],
            [0, c / d, -b / d, 0],
            [0, b / d, c / d, 0],
            [0, 0, 0, 1]
        ])
        R_x_inv = np.array([
            [1, 0, 0, 0],
            [0, c / d, b / d, 0],
            [0, -b / d, c / d, 0],
            [0, 0, 0, 1]
        ])

        R_y = np.array([
            [d, 0, -a, 0],
            [0, 1, 0, 0],
            [a, 0, d, 0],
            [0, 0, 0, 1]
        ])
        R_y_inv = np.array([
            [d, 0, a, 0],
            [0, 1, 0, 0],
            [-a, 0, d, 0],
            [0, 0, 0, 1]
        ])

        # Rotation matrix around z-axis
        ct = np.cos(theta)
        st = np.sin(theta)
        R_z = np.array([
            [ct, st, 0, 0],
            [-st, ct, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Composite transformation
        p_transformed = Vehicle.matrix_multiply(T_inv, R_x_inv, R_y_inv, R_z, R_y, R_x, T, p)

        return p_transformed[:3, 0]
    # --- Projection of a point given normal and point on plane ---
    def projection(point, normal, point_on_plane):
        """
        Projects the vector point on the plane with normal vector and point_on_plane vector
        """
        x=point[0]
        y=point[1]
        z=point[2]
        a=normal[0]/np.dot(normal,point_on_plane)
        b=normal[1]/np.dot(normal,point_on_plane)
        c=normal[2]/np.dot(normal,point_on_plane)
        return Vehicle.project_points(x,y,z,a,b,c)[0]
    # --- Local X and Y axes for the given centre, point and normal ---
    # --- Current Coordinates of points B,C,W,T and wheel travel in Z ---
    def old_B(self, curr_KPA_angle):
        reference = self.reference()
        if np.abs(curr_KPA_angle) < 1e-3:
            return reference.r_B
        shift = curr_KPA_angle*reference.conversionstep
        rounded_curr_KPA_angle = np.round(curr_KPA_angle,reference.maxdecimal)
        rounded_shift = int(rounded_curr_KPA_angle*reference.conversionstep)
        difference = curr_KPA_angle - rounded_curr_KPA_angle
        position_to_add = reference.zeropos + rounded_shift
        
        if(np.abs(shift-rounded_shift) < 1e-3):
            if(reference.dpB[position_to_add][0]<reference.r_B[0]/10):
                reference.dpB[position_to_add] = Vehicle.rotation(reference.dpnewB[position_to_add-int(np.sign(curr_KPA_angle))].tolist(), self.curr_A(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(),self.curr_K(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(), np.sign(curr_KPA_angle)*reference.step)          
            return reference.dpB[position_to_add]
        
        return Vehicle.rotation(reference.dpnewB[position_to_add].tolist(), self.curr_A(rounded_curr_KPA_angle).tolist(),self.curr_K(rounded_curr_KPA_angle).tolist(), difference)
    def curr_B(self, curr_KPA_angle):
        reference = self.reference()
        if np.abs(curr_KPA_angle) < 1e-3:
            return reference.r_B
        shift = curr_KPA_angle*reference.conversionstep
        rounded_curr_KPA_angle = np.round(curr_KPA_angle,reference.maxdecimal)
        difference = curr_KPA_angle - rounded_curr_KPA_angle
        rounded_shift = int(rounded_curr_KPA_angle*reference.conversionstep)
        if(np.abs(shift-rounded_shift) < 1e-3):
            position_to_add = reference.zeropos + rounded_shift
            if(reference.dpnewB[position_to_add][0]<reference.r_B[0]/10):
                reference.dpnewB[position_to_add] = self.old_B(curr_KPA_angle) + self.curr_K(curr_KPA_angle) - self.curr_K(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle))
            return reference.dpnewB[position_to_add]
        return self.old_B(curr_KPA_angle) + self.curr_K(curr_KPA_angle) - self.curr_K(rounded_curr_KPA_angle)
    def curr_C(self, curr_KPA_angle):
        reference = self.reference()
        if np.abs(curr_KPA_angle) < 1e-3:
            return reference.r_C
        length = Vehicle.magnitude(reference.r_C-reference.r_B)
        temp = self.curr_B(curr_KPA_angle)
        shift = curr_KPA_angle*reference.conversionstep
        rounded_curr_KPA_angle = np.round(curr_KPA_angle,reference.maxdecimal)
        rounded_shift = int(rounded_curr_KPA_angle*reference.conversionstep)
        if(np.abs(shift-rounded_shift) < 1e-3):
            position_to_add = reference.zeropos + rounded_shift
            if(reference.dpC[position_to_add][0]<reference.r_C[0]/10):
                reference.dpC[position_to_add] = np.array([reference.r_C[0],temp[1]-np.sqrt(length**2-(reference.r_C[0]-temp[0])**2-(reference.r_C[2]-temp[2])**2), reference.r_C[2]])
            return reference.dpC[position_to_add]
        
        return np.array([reference.r_C[0],temp[1]-np.sqrt(length**2-(reference.r_C[0]-temp[0])**2-(reference.r_C[2]-temp[2])**2), reference.r_C[2]])

    def curr_W(self, curr_KPA_angle):
        reference = self.reference()
        if curr_KPA_angle==0:
            return reference.r_W
        position_to_add = reference.zeropos+int(np.round(curr_KPA_angle,reference.maxdecimal)*reference.conversionstep)
        if(reference.dpW[position_to_add][0]<reference.r_W[0]/1000000000):
            temp = Vehicle.rotation(reference.dpW[position_to_add-int(np.sign(curr_KPA_angle))].tolist(), self.curr_A(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(),self.curr_K(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(), np.sign(curr_KPA_angle)*reference.step)
            temp[2] = reference.r_W[2]
            reference.dpW[position_to_add] = temp
            return reference.dpW[position_to_add]
        temp = Vehicle.rotation(reference.dpW[position_to_add-int(np.sign(curr_KPA_angle))].tolist(), self.curr_A(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(),self.curr_K(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(), curr_KPA_angle - np.round(curr_KPA_angle,reference.maxdecimal))
        temp[2] = reference.r_W[2]
        return temp
    def curr_T(self, curr_KPA_angle):
        reference = self.reference()
        if curr_KPA_angle==0:
            return reference.r_T
        position_to_add = reference.zeropos+int(np.round(curr_KPA_angle,reference.maxdecimal)*reference.conversionstep)
        if(reference.dpT[position_to_add][0]<reference.r_T[0]/1000000000):
            
            temp = Vehicle.rotation(reference.dpT[position_to_add-int(np.sign(curr_KPA_angle))].tolist(), self.curr_A(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(),self.curr_K(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(), np.sign(curr_KPA_angle)*reference.step)
            temp[2] = reference.r_T[2]
            reference.dpT[position_to_add] = temp
            return reference.dpT[position_to_add]
        temp = Vehicle.rotation(reference.dpT[position_to_add-int(np.sign(curr_KPA_angle))].tolist(), self.curr_A(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(),self.curr_K(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(), curr_KPA_angle - np.round(curr_KPA_angle,reference.maxdecimal))
        temp[2] = reference.r_T[2]
        return temp
    def delta_z(self, curr_KPA_angle):
        reference = self.reference()
        if curr_KPA_angle==0:
            return 0
        position_to_add = reference.zeropos+int(np.round(curr_KPA_angle,reference.maxdecimal)*reference.conversionstep)
        if(reference.dpdz[position_to_add]==0):
            reference.dpdz[position_to_add] = reference.dpdz[position_to_add-int(np.sign(curr_KPA_angle))]+Vehicle.rotation(reference.dpT[position_to_add-int(np.sign(curr_KPA_angle))].tolist(), self.curr_A(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(),self.curr_K(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(), np.sign(curr_KPA_angle)*reference.step)[2] - reference.r_T[2]
        return reference.dpdz[position_to_add]
    def curr_O(self, curr_KPA_angle):
        reference = self.reference()
        if np.abs(curr_KPA_angle)<=10e-4:
            return reference.r_O
        position_to_add = reference.zeropos+int(np.round(curr_KPA_angle,reference.maxdecimal)*reference.conversionstep)
        if(reference.dpO[position_to_add][0]<reference.r_O[0]/1000000000):
            # print(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle))
            temp = Vehicle.rotation(reference.dpO[position_to_add-int(np.sign(curr_KPA_angle))].tolist(), self.curr_A(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(),self.curr_K(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle)).tolist(), np.sign(curr_KPA_angle)*reference.step)
            temp[2] = temp[2] - self.delta_z(curr_KPA_angle) + self.delta_z(curr_KPA_angle-reference.step*np.sign(curr_KPA_angle))
            reference.dpO[position_to_add] = temp
        return reference.dpO[position_to_add]
    # --- Current Tangent Motion of the Tire Contact Patch, returns the direction ---
    def curr_tangent(self, point):
        reference = self.reference()
        temp = Vehicle.projection(reference.r_A,reference.currKPA,point) - point
        product = np.cross(temp, reference.currKPA)
        ans = np.array([product[0], product[1], 0])
        ans = ans/Vehicle.magnitude(ans)
        return ans
    # --- Rack Displacement, Steering Arm and Tie Rod ---
    def rack_displacement(self, curr_KPA_angle):
        reference = self.reference()
        return self.curr_C(curr_KPA_angle)[1]-reference.r_C[1]
    def steering_arm(self, curr_KPA_angle):
        reference = self.reference()
        temp = self.curr_B(curr_KPA_angle)
        reference.currKPA = (self.curr_A(curr_KPA_angle)-self.curr_K(curr_KPA_angle))/Vehicle.magnitude(reference.r_A-reference.r_K)
        return temp-Vehicle.projection(reference.r_A, reference.currKPA, temp)
    def tierod(self, curr_KPA_angle):
        return self.curr_C(curr_KPA_angle)-self.curr_B(curr_KPA_angle)
    # --- Inclination and Heading ---
    def wheel_inclination(self, curr_KPA_angle):
        return self.curr_O(curr_KPA_angle)-self.curr_T(curr_KPA_angle)
    def wheel_heading(self, curr_KPA_angle):
        
        return self.curr_W(curr_KPA_angle)-self.curr_T(curr_KPA_angle)
    # --- Caster Trail, Scrub Radius and Spindle Length ---
    def trails(self, curr_KPA_angle):
        reference = self.reference()
        return self.curr_T(curr_KPA_angle)-reference.r_I
    def caster_trail(self, curr_KPA_angle):
        head = self.wheel_heading(curr_KPA_angle)
        mag = Vehicle.magnitude(head)
        return -np.dot(self.trails(curr_KPA_angle),head/mag)
    def scrub_radius(self, curr_KPA_angle):
        head = self.wheel_heading(curr_KPA_angle)
        mag  = Vehicle.magnitude(head)
        return np.sign(np.dot(self.trails(curr_KPA_angle),
                                np.cross(head,np.array([0,0,1]))))*Vehicle.magnitude(self.trails(curr_KPA_angle) + 
                                                                                     self.caster_trail(curr_KPA_angle)*head/mag)
    # --- Camber and Road Steer ---
    def camber(self, curr_KPA_angle):
        inclination = self.wheel_inclination(curr_KPA_angle)
        heading = self.wheel_heading(curr_KPA_angle)
        triple_product = np.cross(np.cross(heading, inclination), heading)
        mag = Vehicle.magnitude(triple_product)
        return np.sign(np.cross(inclination,heading)[2])*np.degrees(np.arccos(np.dot(triple_product, np.array([0,0,1]))/mag))
    def road_steer(self, curr_KPA_angle):
        return np.degrees(np.sign(curr_KPA_angle)*np.arccos(np.dot(self.wheel_heading(curr_KPA_angle),
                                                                   self.wheel_heading(0))/Vehicle.magnitude(self.wheel_heading(curr_KPA_angle))))
    def wheel_angle(self, curr_KPA_angle):
        return -np.sign(self.wheel_heading(curr_KPA_angle)[1])*np.degrees(np.arccos(np.dot(self.wheel_heading(curr_KPA_angle),
                                                                   np.array([-1,0,0]))/Vehicle.magnitude(self.wheel_heading(curr_KPA_angle))))
    # --- Steering Ratio ---
    def steering_ratio(self, curr_KPA_angle):
        """Steering Ratio is the Ratio of Steering Wheel Angle to the Road Steer Angle

        Args:
            curr_KPA_angle (float): Angle rotated by the Kingpin Axis 

        Returns:
            float: Steering Ratio
        """
        if np.abs(curr_KPA_angle)<0.2:
            return self.steering_ratio(0.2)
        reference = self.reference()
        return -1/(self.road_steer(curr_KPA_angle)/self.rack_displacement(curr_KPA_angle)*2*np.pi*reference.pinion/360)
    def caster(self, curr_KPA_angle):
        CurrentKPA = self.curr_KPA(curr_KPA_angle)
        currx = np.array([1,0,0])
        curry = np.cross(np.array([0,0,1]), currx)
        h1 = (CurrentKPA - np.dot(CurrentKPA,curry)*curry)/Vehicle.magnitude(CurrentKPA - np.dot(CurrentKPA,curry)*curry)
        h2 = (CurrentKPA - np.dot(CurrentKPA,currx)*currx)/Vehicle.magnitude(CurrentKPA - np.dot(CurrentKPA,currx)*currx)
        return np.degrees(np.arccos(np.dot(h1,np.array([0,0,1]))))
    def kpi(self, curr_KPA_angle):
        CurrentKPA = self.curr_KPA(curr_KPA_angle)
        currx = np.array([1,0,0])
        curry = np.cross(np.array([0,0,1]), currx)
        h1 = (CurrentKPA - np.dot(CurrentKPA,curry)*curry)/Vehicle.magnitude(CurrentKPA - np.dot(CurrentKPA,curry)*curry)
        h2 = (CurrentKPA - np.dot(CurrentKPA,currx)*currx)/Vehicle.magnitude(CurrentKPA - np.dot(CurrentKPA,currx)*currx)
        return np.degrees(np.arccos(np.dot(h2,np.array([0,0,1])))) 
    #  --- Regression Modeling for Inverse Functions, Kingpin Rotation Angle, Road Steering and Rack Displacement Correlated ---
    def helperroadsteer(self,x):
        # Ensure x is a scalar
        x = x[0] if isinstance(x, (list, np.ndarray)) else x
        return self.road_steer(x)
    def helperrack(self,x):
        # Ensure x is a scalar
        x = x[0] if isinstance(x, (list, np.ndarray)) else x
        return self.rack_displacement(x)
    def inverse_f(self, y):
        # Find x such that f(x) = y
        return fsolve(lambda x: self.helperroadsteer(x) - y, x0=[y])[0] 
    def regression_model(self):
        reference = self.reference()
        X = np.array([])
        y = np.array([])
        t = 0#-5self.step
        for i in range(0,50*reference.conversionstep):
            t = np.round(t + reference.step,2)
            X = np.append(X,self.road_steer(t))
            # print(self.curr_T(t))
            y = np.append(y,t)
        t = 0
        for i in range(0,50*reference.conversionstep):
            t = np.round(t - reference.step,2)
            X = np.append(X,self.road_steer(t))
            y = np.append(y,t)
        X = X.reshape(-1,1)
        poly_features = PolynomialFeatures(degree=10, include_bias=False)
        X_poly = poly_features.fit_transform(X)
        model = LinearRegression()
        model.fit(X_poly, y)
        X1 = np.array([])
        y1 = np.array([])
        t1 = 0 # -5self.step
        for i in range(0,50*reference.conversionstep):
            t1 = t1 + reference.step
            X1 = np.append(X1,self.rack_displacement(t1))
            y1 = np.append(y1,t1)
        t1 = 0
        for i in range(0,50*reference.conversionstep):
            t1 = t1 - reference.step
            X1 = np.append(X1,self.rack_displacement(t1))
            y1 = np.append(y1,t1)
        X1 = X1.reshape(-1,1)
        poly_features1 = PolynomialFeatures(degree=10, include_bias=False)
        X_poly1 = poly_features1.fit_transform(X1)
        model1 = LinearRegression()
        model1.fit(X_poly1, y1)
        return model,poly_features,model1,poly_features1
    def KPA_rotation_angle(self, input_road_steer):
        # input_road_steer = np.array([input_road_steer]).reshape(-1, 1)
        # input_road_steer = reference.model[1].fit_transform(input_road_steer)
        # return (reference.model[0].predict(input_road_steer))[0]
        return fsolve(lambda x: self.helperroadsteer(x) - input_road_steer, x0=[input_road_steer])[0] 
    def rack_vs_road_steer(self, input_road_steer):
        return self.rack_displacement(self.KPA_rotation_angle(input_road_steer))

    def KPA_rotation_angle_vs_rack(self, input_rack_stroke):
        try:
            if np.abs(input_rack_stroke)<1e-3:
                return 0
            val = input_rack_stroke
            input_rack_stroke = np.array([input_rack_stroke]).reshape(-1, 1)
            input_rack_stroke = self.model[3].fit_transform(input_rack_stroke)
            guess = (self.model[2].predict(input_rack_stroke))[0]
            return fsolve(lambda x: self.helperrack(x) - val, x0=[guess], xtol=0.01, factor = 5)[0]
        except Exception as error:
            # Log the error and adjust theta by subtracting 0.01
            print(f"Error encountered at rack displacement = {input_rack_stroke}: {error}. Retrying with rack displacement = {input_rack_stroke + 0.05}")
            return self.KPA_rotation_angle_vs_rack(input_rack_stroke + 0.05)
        return (reference.model[2].predict(input_rack_stroke))[0]
       
    def road_steer_vs_rack(self, input_rack_stroke):
        return self.road_steer(self.KPA_rotation_angle_vs_rack(input_rack_stroke))
    
    # --- Ackerman Calculations ---
    def ackerman(self, inner_angle):
        reference = self.reference()
        return np.degrees(np.arctan(reference.wb/(reference.wb/np.tan(np.radians(inner_angle))+reference.tw)))
    def ackerman_percentage(self, inner, outer):
        return (inner - outer)/(inner - self.ackerman(inner))*100
    def ackerman_vs_KPA(self, curr_KPA_angle):
        return self.ackerman_percentage(np.maximum(np.abs(self.wheel_angle(curr_KPA_angle)),
                                                   np.abs(self.wheel_angle(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle))))),
                                                   np.minimum(np.abs(self.wheel_angle(curr_KPA_angle)),
                                                              np.abs(self.wheel_angle(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle))))))
    def tcr(self, outer_angle, inner_angle):
        reference = self.reference()
        a = reference.a
        t = reference.tw
        theta1 = np.radians(outer_angle)
        theta2 = np.radians(inner_angle)
        OP1 = np.sin(theta2)*t/np.sin(theta2 - theta1)
        OP2 = np.sin(theta1)*t/np.sin(theta2 - theta1)
        OG = np.sqrt(t**2/4 + OP2**2 + 2*t/2*OP2*np.cos(theta2))
        sin_tau = np.sin(theta2)/OG*OP2
        R = np.sqrt(a**2 + OG**2 - 2*a*OG*sin_tau)
        # OG = t/2*np.sin(theta2)/np.sin((theta2-theta1)/2)
        return R/1000 #np.sqrt(a**2+OG**2-2*a*OG*np.sin(theta1/2+theta2/2)**2)
    # --- Tire Contact Patch positions: x_L, x_R, y_L, y_R ---
    def delta_T(self, curr_KPA_angle):
        reference = self.reference()
        return self.curr_T(curr_KPA_angle)-reference.r_T
    def x_R(self, curr_KPA_angle):
        return self.delta_T(curr_KPA_angle)[0]
    def y_R(self, curr_KPA_angle):
        return self.delta_T(curr_KPA_angle)[1]
    def x_L(self, curr_KPA_angle):
        return self.delta_T(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle)))[0]
    def y_L(self, curr_KPA_angle):
        return -self.delta_T(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle)))[1]
    def z_R(self, curr_KPA_angle):
        return self.delta_z(curr_KPA_angle)
    def z_L(self, curr_KPA_angle):
        return self.delta_z(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle)))
    
    # --- Wheel Loads ----
    def F_Lz(self, curr_KPA_angle):
        """
        Curent KPA angle
        """
         # --- Wheel Loads Object ----
        return self.staticsolve(curr_KPA_angle)[0]
    def F_Rz(self, curr_KPA_angle):
        """
        Curent KPA angle
        """
        return self.staticsolve(curr_KPA_angle)[1]
    def R_Lz(self, curr_KPA_angle):
        """
        Curent KPA angle
        """
        return self.staticsolve(curr_KPA_angle)[2]
    def R_Rz(self, curr_KPA_angle):
        """
        Curent KPA angle
        """
        return self.staticsolve(curr_KPA_angle)[3]
    def FrontLoad(self, curr_KPA_angle):
        return self.F_Lz(curr_KPA_angle)+self.F_Rz(curr_KPA_angle)
    def RearLoad(self, curr_KPA_angle):
        reference = self.reference()
        return reference.GVW - self.FrontLoad(curr_KPA_angle)
    def FLRR(self,curr_KPA_angle):
        return self.F_Lz(curr_KPA_angle)+self.R_Rz(curr_KPA_angle)
    def FRRL(self,curr_KPA_angle):
        reference = self.reference()
        return reference.GVW - self.FLRR(curr_KPA_angle)
    def CF_L(self, curr_KPA_angle):
        return self.dynamicsolve(curr_KPA_angle)[8]
    def CF_R(self, curr_KPA_angle):
        return self.dynamicsolve(curr_KPA_angle)[9]
    def CR_L(self, curr_KPA_angle):
        return self.dynamicsolve(curr_KPA_angle)[10]
    def CR_R(self, curr_KPA_angle):
        return self.dynamicsolve(curr_KPA_angle)[11]
    def NF_L(self, curr_KPA_angle):
        return self.dynamicsolve(curr_KPA_angle)[0]
    def NF_R(self, curr_KPA_angle):
        return self.dynamicsolve(curr_KPA_angle)[1]
    def NR_L(self, curr_KPA_angle):
        return self.dynamicsolve(curr_KPA_angle)[2]
    def NR_R(self, curr_KPA_angle):
        return self.dynamicsolve(curr_KPA_angle)[3]
    # --- Set of Equations for Wheel Load Calculations ---
    def staticequation(self, x):
        self.dynamic_analysis = 0
        reference = self.reference()
        zfl = x[0]
        zfr = x[1]
        zrl = x[2]
        zrr = x[3]
        Fl = reference.Kf*zfl
        Fr = reference.Kf*zfr
        Rl = reference.Kr*zrl
        Rr = reference.Kr*zrr
        
        t = reference.tw
        a = reference.a
        b = reference.b
        W = reference.GVW
        FL = np.array([self.x_L(self.curr_KPA_angle), self.y_L(self.curr_KPA_angle), -self.z_L(self.curr_KPA_angle)-zfl])
        FR = np.array([self.x_R(self.curr_KPA_angle), reference.tw+self.y_R(self.curr_KPA_angle), -self.z_R(self.curr_KPA_angle)-zfr])
        RL = np.array([reference.a+reference.b, 0, -zrl])
        RR = np.array([reference.a+reference.b, reference.tw, -zrr])

        eq1 = Fl*(self.y_L(self.curr_KPA_angle)) + Rr*t +Fr*(t+self.y_R(self.curr_KPA_angle)) - W*t/2
        eq2 = Fl*(a+b-self.x_L(self.curr_KPA_angle)) + Fr*(a+b-self.x_R(self.curr_KPA_angle)) - W*b
        eq3 = Fl + Fr + Rl + Rr - W
        eq4 = np.dot(np.cross(FR - FL,RL - FL), FR - RR)
        return [eq1,eq2,eq3,eq4]
    def staticsolve(self, theta):
        self.dynamic_analysis = 0
        reference = self.reference()
        a = reference.a
        b = reference.b
        W = reference.GVW
        self.curr_KPA_angle = theta
        F = W*b/(a+b)*0.5
        R = W*a/(a+b)*0.5
        [zfl,zfr,zrl,zrr] = (fsolve(self.staticequation,[F/reference.Kf,F/reference.Kf,R/reference.Kr,R/reference.Kr]))
        Fl = reference.Kf*zfl
        Fr = reference.Kf*zfr
        Rl = reference.Kr*zrl
        Rr = reference.Kr*zrr
        return Fl,Fr,Rl,Rr

    def dynamicequation(self, x):
        self.dynamic_analysis = 1
        reference = self.reference()
        mu = reference.mu
        g = reference.g
        Fl = x[0]
        Fr = x[1]
        Rl = x[2]
        Rr = x[3]
        alphafL = x[4]
        alphafR = x[5]
        # R = x[6]
        
        # Fl = reference.Kfce.Kfcerence.Kf*zfl
        # Fr = reference.Kfce.Kf*zfr
        # Rl = reference.Krce.Krce.Krce.Kr*zrl
        # Rr = reference.Krce.Kr*zrr
        theta = self.curr_KPA_angle
        thetaL = np.abs(self.road_steer(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(theta))))
        
        thetaR = np.abs(self.road_steer(theta))
       
        zfl = Fl/reference.Kf
        zfr = Fr/reference.Kf
        zrl = Rl/reference.Kr
        zrr = Rr/reference.Kr
        a = reference.a
        b = reference.b
        t = reference.tw
        FL = np.array([self.x_L(theta), self.y_L(theta), -self.z_L(theta)-zfl])
        FR = np.array([self.x_R(theta), t+self.y_R(theta), -self.z_R(theta)-zfr])
        RL = np.array([a+b, 0, -zrl])
        RR = np.array([a+b, t, -zrr])

        h = reference.CG_height + self.curr_O(theta)[2]-self.curr_T(theta)[2]
        M = reference.GVW
        W = M*g
        V = reference.speed
        yL = self.y_L(theta)
        yR = self.y_R(theta)
        xL = self.x_L(theta)
        xR = self.x_R(theta)

        theta2 = np.radians(thetaR - alphafR)
        theta1 = np.radians(thetaL - alphafL)

        OP1 = np.sin(theta2)*t/np.sin(theta2 - theta1)
        OP2 = np.sin(theta1)*t/np.sin(theta2 - theta1)
        OG = np.sqrt(t**2/4 + OP2**2 + 2*t/2*OP2*np.cos(theta2))
        sin_tau = np.sin(theta2)/OG*OP2
        cos_tau = np.sqrt(1-sin_tau**2)
        R = np.sqrt(a**2 + OG**2 - 2*a*OG*sin_tau)
        # print(R)
        tan_alpharL = ((a+b)/OP1 - np.sin(theta1))/np.cos(theta1)
        tan_alpharR = ((a+b)/OP2 - np.sin(theta2))/np.cos(theta2)
        alpharL = np.rad2deg(np.atan(tan_alpharL))
        alpharR = np.rad2deg(np.atan(tan_alpharR))
        theta3 = np.radians(alpharL)
        theta4 = np.radians(alpharR)
        # print(alpharR)

        gamma = np.acos(OG*cos_tau/R)
        B = reference.tiredata[0]
        C = reference.tiredata[1]
        D = reference.tiredata[2]
        E = reference.tiredata[3]
        CF_Loads = self.CF_Loads # np.array([0, 150, 200, 250, 500])
        CF_Stiffnessrad= self.CF_Stiffnessrad # np.array([0, 20234.57749,	23031.75745, 24629.16378, 24629.16378 + 250*(24629.16378-23031.75745)/50])
        interpolator = interp1d(CF_Loads, CF_Stiffnessrad, kind='linear')
        Cfl = interpolator(Fl)
        Cfr = interpolator(Fr)
        Crl = interpolator(Rl)
        Crr = interpolator(Rr)
        alphafLprime = Cfl/g*np.tan(np.radians(alphafL))/mu/Fl
        alphafRprime = Cfr/g*np.tan(np.radians(alphafR))/mu/Fr
        alpharLprime = Crl/g*np.tan(np.radians(alpharL))/mu/Rl
        alpharRprime = Crr/g*np.tan(np.radians(alpharR))/mu/Rr

        CFL = mu*D*Fl*np.sin(C*np.atan(B*((alphafLprime) - E*(alphafLprime) +E/B*np.atan(B*(alphafLprime)))))
        CFR = mu*D*Fr*np.sin(C*np.atan(B*((alphafRprime) - E*(alphafRprime) +E/B*np.atan(B*(alphafRprime)))))
        CRL = mu*D*Rl*np.sin(C*np.atan(B*((alpharLprime) - E*(alpharLprime) +E/B*np.atan(B*(alpharLprime)))))
        CRR = mu*D*Rr*np.sin(C*np.atan(B*((alpharRprime) - E*(alpharRprime) +E/B*np.atan(B*(alpharRprime)))))

        dlr = reference.tire_radius

        # CFL = 2000/g*np.sin(1.2*np.radians(alphafL) + 0.8*(np.radians(alphafL) - np.atan(1.2*np.radians(alphafL))))
        # CFR = 2000/g*np.sin(1.2*np.radians(alphafR) + 0.8*(np.radians(alphafR) - np.atan(1.2*np.radians(alphafR))))
        # CRL = 2000/g*np.sin(1.2*np.radians(alpharL) + 0.8*(np.radians(alpharL) - np.atan(1.2*np.radians(alpharL))))
        # CRR = 2000/g*np.sin(1.2*np.radians(alpharR) + 0.8*(np.radians(alpharR) - np.atan(1.2*np.radians(alpharR))))

        # n=1/2
        # CFL = (Fl)**n/2*alphafL
        # CFR = (Fr)**n/2*alphafR
        # CRL = (Rl)**n/2*alpharL
        # CRR = (Rr)**n/2*alpharR
        

        # eq1 = (CFL+CFR+CRL+CRR)*h - (Fl-Fr+Rl-Rr)*t/2# due to radial force
        eq1 = Rr*t + Fr*(t+yR)+Fl*yL + (CFL*np.cos(theta1) + CFR*np.cos(theta2)+CRL*np.cos(theta3) + CRR*np.cos(theta4))*(h+dlr) - M*t/2  # Fl*yL + Rr*t + Fr*(t+yR) + (CFL*np.cos(theta1) + CFR*np.cos(theta2)+CRL*np.cos(theta3) + CRR*np.cos(theta4))*h - M*t/2 
        eq2 = Fl*(a + b -xL) + Fr*(a +b -xR) + (CRL*np.sin(theta3) + CRR*np.sin(theta4))*(h+dlr) - (M*b + (CFL*np.sin(theta1) + CFR*np.sin(theta2))*(h+dlr)) # Fl*(a + b - xL) + Fr*(a +b - xR) + (CRL*np.sin(theta3) + CRR*np.sin(theta4))*h - (M*b + (CFL*np.sin(theta1) + CFR*np.sin(theta2))*h)
        eq3 = (Fl + Fr + Rl + Rr - M)
        eq4 = np.dot(np.cross(FR - FL,RL - FL), FR - RR)
        eq5 = CFL*np.cos(theta1)*yL+CFR*np.cos(theta2)*(t+yR) + CFL*np.sin(theta1)*(a+b-xL) + CFR*np.sin(theta2)*(a+b-xR) - (CRR*np.cos(theta4)*t + M/g*V**2/R*1000*np.sin(gamma)*t/2 + M/g*V**2/R*1000*np.cos(gamma)*b) # CFL*np.cos(theta1)*(yL) + CFR*np.cos(theta2)*(t+yR) + CFL*np.sin(theta1)*(a+b-xL) + CFR*np.sin(theta2)*(a+b-xR) - (CRR*np.cos(theta4)*t + M/g*V**2/R*1000*np.sin(gamma)*t/2 + M/g*V**2/R*1000*np.cos(gamma)*b)
        # eq6 = CFL*np.sin(theta1) + CFR*np.sin(theta2) - CRL*np.sin(theta3) - CRR*np.sin(theta4) - M/g*V**2/R*1000*np.sin(gamma)
        # eq4 = CFL*np.cos(theta1) + CFR*np.cos(theta2) + CRL*np.cos(theta3) + CRR*np.cos(theta4) - M/g*V**2/R*1000*np.cos(gamma)
        eq6 = CFL + CFR + CRL + CRR - M/g*V**2/R*1000
        return [eq1,eq2,eq3,eq4,eq5,eq6]     
    def dynamicsolve(self, theta):
        try:
            self.dynamic_analysis = 1
            reference = self.reference()
            mu = reference.mu
            g = reference.g
            a = reference.a
            b = reference.b
            W = reference.GVW
            t = reference.tw
            if(theta<=0):
                loc = -int(int(theta))
                limits = reference.slipangles[loc-1]
                Flguess = reference.Flguess[loc-1]
                Frguess = reference.Frguess[loc-1]
                Rlguess = reference.Rlguess[loc-1]
                Rrguess = reference.Rrguess[loc-1]
                self.curr_KPA_angle = theta
                Fhalf = W*b/(a+b)*0.5
                Rhalf = W*a/(a+b)*0.5
                # Fl,Fr,Rl,Rr = self.staticsolve(theta)
                # inner_angle = np.abs(self.road_steer(theta))
                # outer_angle = np.abs(self.road_steer_vs_rack(-self.rack_displacement(theta)))
                # max_angle = np.maximum(inner_angle, outer_angle)
                # Rad = self.tcr(outer_angle + inner_angle - max_angle, max_angle)
                # initial_CF = Fhalf/g*reference.speed**2/Rad*1000*np.sin(np.radians((inner_angle + outer_angle)/2))
                # initial_CR = Rhalf/g*reference.speed**2/Rad*1000*np.sin(np.radians((inner_angle + outer_angle)/2))
                # initial_alphaf = np.rad2deg(initial_CF/2000*g) #np.sin(1.2*np.radians(alphafL) + 0.8*(np.radians(alphafL) - np.atan(1.2*np.radians(alphafL))))
                # initial_alphar = np.rad2deg(initial_CR/2000*g)

                # Flguess = W*b/(a+b)*0.5
                # Frguess = W*b/(a+b)*0.5
                # Rlguess = W*a/(a+b)*0.5
                # Rrguess = W*a/(a+b)*0.5
                # if (reference.Flguess[int(-theta-1)]!=0):
                #     Flguess = reference.Flguess[int(-theta-1)]
                #     Frguess = reference.Frguess[int(-theta-1)]
                #     Rlguess = reference.Rlguess[int(-theta-1)]
                #     Rrguess = reference.Rrguess[int(-theta-1)]
                [Fl,Fr,Rl,Rr, alphafL, alphafR] = (fsolve(self.dynamicequation, [Flguess, Frguess, Rlguess, Rrguess, limits[0], limits[1]], xtol=1e-3))
                # [Fl,Fr,Rl,Rr, alphafL, alphafR] = (fsolve(self.dynamicequation,[Fhalf,Fhalf,Rhalf,Rhalf,limits[0],limits[1]], xtol=0.001))
                
                # Fl = reference.Kf*zfl
                # Fr = reference.Kf*zfr
                # Rl = reference.Kr*zrl
                # Rr = reference.Kr*zrr
                
                thetaL = np.abs(self.road_steer(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(theta))))
                thetaR = np.abs(self.road_steer(theta))
                theta2 = np.radians(thetaR - alphafR)
                theta1 = np.radians(thetaL - alphafL)
                
                OP1 = np.sin(theta2)*t/np.sin(theta2 - theta1)
                OP2 = np.sin(theta1)*t/np.sin(theta2 - theta1)
                OG = np.sqrt(t**2/4 + OP2**2 + 2*t/2*OP2*np.cos(theta2))
                sin_tau = np.sin(theta2)/OG*OP2
                cos_tau = np.sqrt(1-sin_tau**2)
                R = np.sqrt(a**2 + OG**2 - 2*a*OG*sin_tau)
                
                tan_alpharL = ((a+b)/OP1 - np.sin(theta1))/np.cos(theta1)
                tan_alpharR = ((a+b)/OP2 - np.sin(theta2))/np.cos(theta2)
                alpharL = np.rad2deg(np.atan(tan_alpharL))
                alpharR = np.rad2deg(np.atan(tan_alpharR))
            else:
                opposite_rack_travel = -self.rack_displacement(theta)
                theta = self.KPA_rotation_angle_vs_rack(opposite_rack_travel)
                loc = -int(int(theta))
                limits = reference.slipangles[loc]
                self.curr_KPA_angle = theta
                # Fhalf = reference.GVW*reference.b/(reference.a+reference.b)*0.5
                # Rhalf = reference.GVW*reference.a/(reference.a+reference.b)*0.5
                Flguess = reference.GVW*reference.b/(reference.a+reference.b)*0.5
                Frguess = reference.GVW*reference.b/(reference.a+reference.b)*0.5
                Rlguess = reference.GVW*reference.a/(reference.a+reference.b)*0.5
                Rrguess = reference.GVW*reference.a/(reference.a+reference.b)*0.5
                if (reference.Flguess[int(theta-1)]!=0):
                    Flguess = reference.Flguess[int(theta-1)]
                    Frguess = reference.Frguess[int(theta-1)]
                    Rlguess = reference.Rlguess[int(theta-1)]
                    Rrguess = reference.Rrguess[int(theta-1)]
                [Fl,Fr,Rl,Rr, alphafL, alphafR] = (fsolve(self.dynamicequation, [Flguess, Frguess, Rlguess, Rrguess, limits[0], limits[1]], xtol=1e-4, maxfev=5000))
                # Fl,Fr,Rl,Rr = self.staticsolve(theta)
                # inner_angle = np.abs(self.road_steer(theta))
                # outer_angle = np.abs(self.road_steer_vs_rack(-self.rack_displacement(theta)))
                # max_angle = np.maximum(inner_angle, outer_angle)
                # Rad = self.tcr(outer_angle + inner_angle - max_angle, max_angle)
                # initial_CF = Fhalf/g*reference.speed**2/Rad*1000*np.sin(np.radians((inner_angle + outer_angle)/2))
                # initial_CR = Rhalf/g*reference.speed**2/Rad*1000*np.sin(np.radians((inner_angle + outer_angle)/2))
                # initial_alphaf = np.rad2deg(initial_CF/2000*g) #np.sin(1.2*np.radians(alphafL) + 0.8*(np.radians(alphafL) - np.atan(1.2*np.radians(alphafL))))
                # initial_alphar = np.rad2deg(initial_CR/2000*g)
                # [Fl,Fr,Rl,Rr, alphafL, alphafR] = (fsolve(self.dynamicequation,[Fhalf,Fhalf,Rhalf,Rhalf,limits[0],limits[1]], xtol=0.001))
                # Fl = reference.Kf*zfl
                # Fr = reference.Kf*zfr
                # Rl = reference.Kr*zrl
                # Rr = reference.Kr*zrr
                theta = self.curr_KPA_angle
                thetaL = np.abs(self.road_steer(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(theta))))
                thetaR = np.abs(self.road_steer(theta))
                theta2 = np.radians(thetaR - alphafR)
                theta1 = np.radians(thetaL - alphafL)
                OP1 = np.sin(theta2)*t/np.sin(theta2 - theta1)
                OP2 = np.sin(theta1)*t/np.sin(theta2 - theta1)
                OG = np.sqrt(t**2/4 + OP2**2 + 2*t/2*OP2*np.cos(theta2))
                sin_tau = np.sin(theta2)/OG*OP2
                cos_tau = np.sqrt(1-sin_tau**2)
                R = np.sqrt(a**2 + OG**2 - 2*a*OG*sin_tau)
                
                tan_alpharL = ((a+b)/OP1 - np.sin(theta1))/np.cos(theta1)
                tan_alpharR = ((a+b)/OP2 - np.sin(theta2))/np.cos(theta2)
                alpharL = np.rad2deg(np.atan(tan_alpharL))
                alpharR = np.rad2deg(np.atan(tan_alpharR))
            B = reference.tiredata[0]
            C = reference.tiredata[1]
            D = reference.tiredata[2]
            E = reference.tiredata[3]
            CF_Loads = self.CF_Loads # np.array([0, 150, 200, 250, 500])
            CF_Stiffnessrad = self.CF_Stiffnessrad # np.array([0, 20234.57749,	23031.75745, 24629.16378, 24629.16378 + 250*(24629.16378-23031.75745)/50])
            interpolator = interp1d(CF_Loads, CF_Stiffnessrad, kind='linear')
            Cfl = interpolator(Fl)
            Cfr = interpolator(Fr)
            Crl = interpolator(Rl)
            Crr = interpolator(Rr)
            alphafLprime = Cfl/g*np.tan(np.radians(alphafL))/mu/Fl
            alphafRprime = Cfr/g*np.tan(np.radians(alphafR))/mu/Fr
            alpharLprime = Crl/g*np.tan(np.radians(alpharL))/mu/Rl
            alpharRprime = Crr/g*np.tan(np.radians(alpharR))/mu/Rr
            CFL = mu*D*Fl*np.sin(C*np.atan(B*((alphafLprime) - E*(alphafLprime) +E/B*np.atan(B*(alphafLprime)))))
            CFR = mu*D*Fr*np.sin(C*np.atan(B*((alphafRprime) - E*(alphafRprime) +E/B*np.atan(B*(alphafRprime)))))
            CRL = mu*D*Rl*np.sin(C*np.atan(B*((alpharLprime) - E*(alpharLprime) +E/B*np.atan(B*(alpharLprime)))))
            CRR = mu*D*Rr*np.sin(C*np.atan(B*((alpharRprime) - E*(alpharRprime) +E/B*np.atan(B*(alpharRprime)))))
            SAT = self.sat(alphafL, alphafR, Fl, Fr)    
            return Fl,Fr,Rl,Rr, alphafL, alphafR, alpharL, alpharR, CFL, CFR, CRL, CRR, SAT
        except Exception as error:
            # Log the error and adjust theta by subtracting 0.01
            print(f"Error encountered at theta = {theta}: {error}. Retrying with theta = {theta - 0.01}")
            return self.dynamicsolve(theta - 0.01)

    def trainslipangles(self):
        self.dynamic_analysis = 1
        reference = self.reference()
        angle = 0
        for i in range(49):
            angle = angle - 1
            temp = self.dynamicsolve(angle)
            reference.Flguess[i+1] = temp[0]
            reference.Frguess[i+1] = temp[1]
            reference.Rlguess[i+1] = temp[2]
            reference.Rrguess[i+1] = temp[3]
            reference.slipangles[i+1][0] = temp[4]
            reference.slipangles[i+1][1] =temp[5]
    # --- Kingpin Moment Calulations ---
    def kpm_circular(self, theta):
        reference = self.reference()
        self.curr_KPA_angle = theta
        reference.currKPA = (self.curr_A(theta)-self.curr_K(theta))/Vehicle.magnitude(reference.r_A-reference.r_K)
        t = theta
        normal = self.F_Rz(t)*np.array([0,0,1])/1000*reference.g
        fric_dir = np.sign(t)*np.cross(normal,self.wheel_heading(t))
        # friction = 0 # mu*F_Rz(t)*fric_dir/magnitude(fric_dir)/1000*g
        # moment_arm = self.curr_T(t)-reference.r_I
        # total_force = friction+normal
        patch_radius = np.sqrt(self.F_Rz(self.curr_KPA_angle)*reference.g/np.pi/reference.tirep/6894.75729)
        temp = integrate.dblquad(self.tire_twisting_moment_circular_static, 0, 2*np.pi, 0, 1000*patch_radius)[0]/10**9
        if 0==self.curr_KPA_angle:
            return temp
        return temp*np.sign(self.curr_KPA_angle) #np.dot(np.cross(moment_arm, total_force), KPA) + temp #+kpm_tp(curr_KPA_angle)
    def circular_contactpatch_element(self, r, phi):
        curr_point = self.curr_T(self.curr_KPA_angle) + np.array([r*np.cos(phi),r*np.sin(phi),0])
        return self.curr_tangent(curr_point)
    def linear_interpolation(input_value):
        """
        Linearly interpolates with increasing gradient.
        Maps 0 to 0.2 and 50 to 0.7.

        Parameters:
            input_value (float): The input value to scale.

        Returns:
            float: A scaled value between 0.2 and 0.7.
        """
        # Ensure input_value is within the valid range
        input_value = np.maximum(-11, np.minimum(input_value, 11))
        x = 0.75
        y = 0.05
        # Calculate the scaled value
        scaled_value = (x+y)/2 - (x-y) * (input_value / 22)
        return scaled_value
    def tire_twisting_moment_circular_return(self, r,phi):
        reference = self.reference()
        distance = self.curr_T(self.curr_KPA_angle)+np.array([r*np.cos(phi),r*np.sin(phi),0])-reference.r_I
        #Vehicle.linear_interpolation(self.delta_z(self.curr_KPA_angle))
        # 
        force = reference.mu*reference.tirep*6894.75729*r*self.circular_contactpatch_element(r,phi) #+ reference.tirep*6894.75729*r*np.array([0,0,1]) #np.array([-np.sin(phi),np.cos(phi),0])
        return np.dot(np.cross(distance,force),reference.currKPA)
    def tire_twisting_moment_circular_static(self, r,phi):
        reference = self.reference()
        distance = self.curr_T(self.curr_KPA_angle)+np.array([r*np.cos(phi),r*np.sin(phi),0])-reference.r_I
        #Vehicle.linear_interpolation(self.delta_z(self.curr_KPA_angle))
        # 
        force = reference.mu*reference.tirep*6894.75729*r*self.circular_contactpatch_element(r,phi) + reference.tirep*6894.75729*r*np.array([0,0,1]) #np.array([-np.sin(phi),np.cos(phi),0])
        return np.dot(np.cross(distance,force),reference.currKPA)
   
    def dynamic_element_moment_circular_right(self, r,phi):
        self.dynamic_analysis = 1
        reference = self.reference()
        theta = self.curr_KPA_angle
        distance = self.curr_T(theta)+np.array([r*np.cos(phi),r*np.sin(phi),0])-reference.r_I
        #Vehicle.linear_interpolation(self.delta_z(self.curr_KPA_angle))
        temp = reference.tempdynamicsolution
        thetaL = np.abs(self.road_steer(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(theta))))
        thetaR = np.abs(self.road_steer(theta))
        alphafL = temp[4]
        alphafR = temp[5]
        theta1 = np.radians(thetaL - alphafL)
        theta2 = np.radians(thetaR - alphafR)
        left_dir = np.array([np.sin(theta1),np.cos(theta1),0])
        right_dir = np.array([np.sin(theta2),np.cos(theta2),0])
        CFL = temp[8]
        CFR = temp[9]
        patch_radius = self.patch_radius_right
        force = CFR/np.pi/patch_radius**2*reference.g*r*right_dir + reference.tirep*6894.75729*r*np.array([0,0,1]) #- reference.mu*reference.tirep*6894.75729*r*self.circular_contactpatch_element(r,phi)
        return np.dot(np.cross(distance,force),reference.currKPA)    
    def dynamic_element_moment_circular_left(self, r,phi):
        self.dynamic_analysis = 1
        reference = self.reference()
        theta = self.curr_KPA_angle
        distance = self.curr_T(theta)+np.array([r*np.cos(phi),r*np.sin(phi),0])-reference.r_I
        #Vehicle.linear_interpolation(self.delta_z(self.curr_KPA_angle))
        temp = reference.tempdynamicsolution
        thetaL = np.abs(self.road_steer(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(theta))))
        thetaR = np.abs(self.road_steer(theta))
        alphafL = temp[4]
        alphafR = temp[5]
        theta1 = np.radians(thetaL - alphafL)
        theta2 = np.radians(thetaR - alphafR)
        left_dir = np.array([np.sin(theta1),np.cos(theta1),0])
        right_dir = np.array([np.sin(theta2),np.cos(theta2),0])
        CFL = temp[8]
        CFR = temp[9]
        patch_radius = self.patch_radius_left
        normal_contribution  = reference.tirep*6894.75729*r*np.array([0,0,1])
        
        cornering_contribution = CFL/np.pi/patch_radius**2*reference.g*r*left_dir
        # print(cornering_contribution)
        force = cornering_contribution + normal_contribution #- reference.mu*reference.tirep*6894.75729*r*self.circular_contactpatch_element(r,phi)
        return np.dot(np.cross(distance,force),reference.currKPA)
    def helper_return(self, theta, normal_force):
        reference = self.reference()
        self.curr_KPA_angle = theta
        reference.currKPA = (self.curr_A(theta)-self.curr_K(theta))/Vehicle.magnitude(reference.r_A-reference.r_K)
        t = theta
        normal = normal_force*np.array([0,0,1])/1000*reference.g
        fric_dir = np.sign(t)*np.cross(normal,self.wheel_heading(t))
        # friction = 0 # mu*F_Rz(t)*fric_dir/magnitude(fric_dir)/1000*g
        # moment_arm = self.curr_T(t)-reference.r_I
        # total_force = friction+normal
        patch_radius = np.sqrt(normal_force*reference.g/np.pi/reference.tirep/6894.75729)
        temp = integrate.dblquad(self.tire_twisting_moment_circular_return, 0, 2*np.pi, 0, 1000*patch_radius)[0]/10**9
        if 0==self.curr_KPA_angle:
            return temp
        return temp*np.sign(self.curr_KPA_angle)
    def kpm_circular_dynamic_left(self, theta):
        self.dynamic_analysis = 1
        reference = self.reference()
        if (theta==0):
            return 0
        if (theta>0):
            theta = self.KPA_rotation_angle_vs_rack(-self.rack_displacement(theta))
            return self.kpm_circular_dynamic_right(theta)
        if (reference.tempdynamictheta != theta):
            reference.tempdynamictheta = theta    
            reference.tempdynamicsolution = self.dynamicsolve(theta)
        self.curr_KPA_angle = theta
        reference.currKPA = (self.curr_A(theta)-self.curr_K(theta))/Vehicle.magnitude(reference.r_A-reference.r_K)
        t = theta
        patch_radius = np.sqrt(reference.tempdynamicsolution[0]*reference.g/np.pi/reference.tirep/6894.75729)
        self.patch_radius_left = patch_radius
        temp = integrate.dblquad(self.dynamic_element_moment_circular_left, 0, 2*np.pi, 0, 1000*patch_radius)[0]/10**9
        # temp2 = integrate.dblquad(self.tire_twisting_moment_circular, 0, 2*np.pi, 0, 1000*patch_radius)[0]/10**9
        if 0==self.curr_KPA_angle:
            return temp
        # friction_contribution = self.helper_return(theta, reference.tempdynamicsolution[0])
        sat_contribution =  - np.sign(theta)*np.abs(np.dot(np.array([0,0,reference.tempdynamicsolution[12][0]]),reference.currKPA))
        # print(sat_contribution)
        # print(friction_contribution)
        return temp + sat_contribution # + friction_contribution # + temp2*np.sign(theta) #np.dot(np.cross(moment_arm, total_force), KPA) + temp #+kpm_tp(curr_KPA_angle)
    def kpm_circular_dynamic_right(self, theta):
        self.dynamic_analysis = 1
        reference = self.reference()
        if (theta==0):
            return 0
        if (theta>0):
            theta = self.KPA_rotation_angle_vs_rack(-self.rack_displacement(theta))
            return self.kpm_circular_dynamic_left(theta)
        if (reference.tempdynamictheta != theta):  
            reference.tempdynamictheta = theta     
            reference.tempdynamicsolution = self.dynamicsolve(theta)
        self.curr_KPA_angle = theta
        reference.currKPA = (self.curr_A(theta)-self.curr_K(theta))/Vehicle.magnitude(reference.r_A-reference.r_K)
        t = theta
        patch_radius = np.sqrt(reference.tempdynamicsolution[1]*reference.g/np.pi/reference.tirep/6894.75729)
        self.patch_radius_right = patch_radius
        temp = integrate.dblquad(self.dynamic_element_moment_circular_right, 0, 2*np.pi, 0, 1000*patch_radius)[0]/10**9
        # temp2 = integrate.dblquad(self.tire_twisting_moment_circular, 0, 2*np.pi, 0, 1000*patch_radius)[0]/10**9
        if 0==self.curr_KPA_angle:
            return temp
        return temp - np.sign(theta)*np.abs(np.dot(np.array([0,0,reference.tempdynamicsolution[12][1]]),reference.currKPA)) #+ self.helper_return(theta, reference.tempdynamicsolution[1]) #+ # temp2*np.sign(theta) #np.dot(np.cross(moment_arm, total_force), KPA) + temp #+kpm_tp(curr_KPA_angle)
       
    def static_kingpin_moment(self, curr_KPA_angle):
        return self.kpm_circular(curr_KPA_angle)
    def sat(self, alphafL, alphafR, Fl, Fr):
        self.dynamic_analysis = 1
        reference = self.reference()
        mu = reference.mu
        g = reference.g
        B = reference.tiredata[4]
        C = reference.tiredata[5]
        D = reference.tiredata[6]
        E = reference.tiredata[7]
        CF_Loads = self.CF_Loads # np.array([0, 150, 200, 250, 500])
        CF_Stiffnessrad = self.CF_Stiffnessrad # np.array([0, 20234.57749,	23031.75745, 24629.16378, 24629.16378 + 250*(24629.16378-23031.75745)/50])
        CF_pneumatictrail = self.CF_pneumatictrail # np.array([0, 0.011909253,	0.018484467, 0.023331694, 0.023331694 + 250*(0.023331694-0.018484467)/50])
        interpolator = interp1d(CF_Loads, CF_Stiffnessrad, kind='linear')
        pneumaticinterpolator = interp1d(CF_Loads, CF_pneumatictrail, kind='linear')
        Cfl = interpolator(Fl)
        Cfr = interpolator(Fr)
        alphafLprime = Cfl/g*np.tan(np.radians(alphafL))/mu/Fl
        alphafRprime = Cfr/g*np.tan(np.radians(alphafR))/mu/Fr
        satFLprime = D*np.sin(C*np.atan(B*((alphafLprime) - E*(alphafLprime) +E/B*np.atan(B*(alphafLprime)))))
        satFRprime = D*np.sin(C*np.atan(B*((alphafRprime) - E*(alphafRprime) +E/B*np.atan(B*(alphafRprime)))))
        TzL = pneumaticinterpolator(Fl)
        TzR = pneumaticinterpolator(Fr)
        satFL = TzL*satFLprime*mu*Fl*g
        satFR = TzR*satFRprime*mu*Fr*g
        return satFL, satFR
    def left_plus_right_returning_moment(self, curr_KPA_angle):
        self.dynamic_analysis = 1
        reference = self.reference()
        reference.currKPA = (self.curr_A(curr_KPA_angle)-self.curr_K(curr_KPA_angle))/Vehicle.magnitude(reference.r_A-reference.r_K)
        raw_left = self.kpm_circular_dynamic_right(curr_KPA_angle)
        raw_right = self.kpm_circular_dynamic_left(curr_KPA_angle)
        friction_contribution_left = self.helper_return(curr_KPA_angle, reference.tempdynamicsolution[0])
        friction_contribution_right = self.helper_return(curr_KPA_angle, reference.tempdynamicsolution[1])
        return raw_left + raw_right + friction_contribution_left + friction_contribution_right
    # --- Steering Effort ---
    def tierod_force(self, curr_KPA_angle):
        self.dynamic_analysis = 0
        reference = self.reference()
        reference.currKPA = (self.curr_A(curr_KPA_angle)-self.curr_K(curr_KPA_angle))/Vehicle.magnitude(reference.r_A-reference.r_K)
        return self.static_kingpin_moment(curr_KPA_angle)*1000/np.dot(np.cross(self.steering_arm(curr_KPA_angle),
                                                                        self.tierod(curr_KPA_angle)/Vehicle.magnitude(self.tierod(curr_KPA_angle))),
                                                                        reference.currKPA)*self.tierod(curr_KPA_angle)/Vehicle.magnitude(self.tierod(curr_KPA_angle))
    def tierod_force_dynamic_right(self, curr_KPA_angle):
        self.dynamic_analysis = 1
        reference = self.reference()
        # if(curr_KPA_angle==0):
        #         return 0
        # if(curr_KPA_angle>0):
        #     curr_KPA_angle = self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle))
        #     return self.tierod_force_dynamic_left(curr_KPA_angle)
        tierod = self.tierod(curr_KPA_angle)
        mag= Vehicle.magnitude(self.tierod(curr_KPA_angle))
        reference.currKPArrKPArrKPA = (self.curr_A(curr_KPA_angle)-self.curr_K(curr_KPA_angle))/Vehicle.magnitude(reference.r_A-reference.r_K)
        force = self.kpm_circular_dynamic_right(curr_KPA_angle)*1000/np.dot(np.cross(self.steering_arm(curr_KPA_angle),
                                                                        tierod/mag),
                                                                        reference.currKPA)*tierod/mag
        return force
    def tierod_force_dynamic_left(self, curr_KPA_angle):
        self.dynamic_analysis = 1
        reference = self.reference()
        # if(curr_KPA_angle==0):
        #         return 0
        # if(curr_KPA_angle>0):
        #     curr_KPA_angle = self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle))
        #     return self.tierod_force_dynamic_right(curr_KPA_angle)
        tierod = self.tierod(curr_KPA_angle)
        mag= Vehicle.magnitude(self.tierod(curr_KPA_angle))
        reference.currKPA = (self.curr_A(curr_KPA_angle)-self.curr_K(curr_KPA_angle))/Vehicle.magnitude(reference.r_A-reference.r_K)
        force = self.kpm_circular_dynamic_left(curr_KPA_angle)*1000/np.dot(np.cross(self.steering_arm(curr_KPA_angle),
                                                                        tierod/mag),
                                                                        reference.currKPA)*tierod/mag
        force[1] = - force[1]
        return force
    def rack_force_dynamic(self, curr_KPA_angle):
        self.dynamic_analysis = 1
        reference = self.reference()
        # if(curr_KPA_angle==0):
        #         return 0
        # if(curr_KPA_angle>0):
        #     curr_KPA_angle = self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle))
        right_tierod_force = self.tierod_force_dynamic_right(curr_KPA_angle)
        left_tierod_force = self.tierod_force_dynamic_left(curr_KPA_angle)
        return Vehicle.magnitude(right_tierod_force)**2/(np.dot(right_tierod_force,
                          np.array([0,1,0]))) - Vehicle.magnitude(left_tierod_force)**2/(np.dot(left_tierod_force,
                                                    np.array([0,1,0])))
    def rack_force(self, curr_KPA_angle):
        current_tierod_force = self.tierod_force(curr_KPA_angle)
        opposite_tierod_force = self.tierod_force(self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle)))
        return Vehicle.magnitude(current_tierod_force)**2/(np.dot(current_tierod_force,
                          np.array([0,1,0]))) - Vehicle.magnitude(opposite_tierod_force)**2/(np.dot(opposite_tierod_force,
                                                    np.array([0,1,0])))
    def static_steering_effort(self, curr_KPA_angle):
        self.dynamic_analysis = 0
        reference = self.reference()
        return np.abs(self.rack_force(curr_KPA_angle)*reference.pinion/1000) + self.linkage_friction_contribution_on_steering
    def dynamic_steering_effort(self, curr_KPA_angle):
        self.dynamic_analysis = 1
        reference = self.reference()
        if(curr_KPA_angle==0):
            return 0
        if(curr_KPA_angle>0):
            curr_KPA_angle = self.KPA_rotation_angle_vs_rack(-self.rack_displacement(curr_KPA_angle))
        return np.abs(self.rack_force_dynamic(curr_KPA_angle)*reference.pinion/1000)
    def returnability(self, lim_time):
        self.dynamic_analysis = 1
        reference = self.reference()
        # Parameters
        I_w = 0.34  # Moment of Inertia of the wheel wrt the Kingpin Axis
        I_ss = 0.030 # Moment of Interia of the steering system wrt the steering column
        y0 = self.rack_stroke/37.71*360  # Initial condition for y
        v0 = 0.0  # Initial condition for y'
        # t_span = (0, 0.1)  # Time range
        # t_eval = np.linspace(t_span[0], t_span[1], 150)  # Time points to evaluate
        # Solve the ODE
        # wheel_solution = solve_ivp(self.wheel_system, t_span, [y0, v0], t_eval=t_eval, args=(I_w,))
        # steering_solution = solve_ivp(self.steering_system, t_span, [y0, v0], t_eval=t_eval, args=(I_ss,))
        # Extract solution
        system = self.wheel_system
        k = I_w
        # system = self.steering_system
        # print(wheel_solution.y[0][-1])
        # print(steering_solution.y[0][-1])
        # if wheel_solution.y[0][-1] > steering_solution.y[0][-1]:
        #     system = self.wheel_system
        #     k = I_w
        #     print('It is a wheel solution')

        t_span = (0, lim_time)  # Time range
        t_eval = np.linspace(t_span[0], t_span[1], 50)  # Time points to evaluate
        # Solve the ODE
        solution = solve_ivp(system, t_span, [y0, v0], t_eval=t_eval, args=(k,), rtol = 1e-3, atol=1e-6)
        # Extract solution
        t = solution.t
        y = solution.y[0]  # y corresponds to y1
        # Plot the solution
        plt.figure(figsize=(8, 5))
        plt.plot(t, y, label="y(t)")
        plt.title("Solution to Differential Equation $ky'' = f(y)$")
        plt.xlabel("Time (t)")
        plt.ylabel("y(t)")
        plt.grid()
        plt.legend()
        plt.show()
        return (y0-y[-1])/y0*100
    def wheel_system(self, t, Y, k):
        self.dynamic_analysis = 1
        reference = self.reference()
        y1, y2 = Y  # Unpack Y = [y1, y2]z
        dy1_dt = y2
        c_factor = 2*np.pi*reference.pinion
        angle = self.KPA_rotation_angle_vs_rack(y1/360*c_factor)
        friction = self.linkage_friction_contribution_on_kpm
        factor = 1
        if(np.sign(y2)>=0 and t>0):
            factor = 0
            
            print(f"Optimal parameters: y1 = {y1}, t = {t}")
        dy2_dt = -factor*(self.left_plus_right_returning_moment(angle) - 2*friction)/ 2/ k * self.steering_ratio(angle)
        return [dy1_dt, dy2_dt]
    def steering_system(self,t,Y,k):
        self.dynamic_analysis = 1
        reference = self.reference()
        y1, y2 = Y  # Unpack Y = [y1, y2]
        dy1_dt = y2
        c_factor = 2*np.pi*reference.pinion
        angle = self.KPA_rotation_angle_vs_rack(y1/360*c_factor)
        friction = self.linkage_friction_contribution_on_steering
        dy2_dt = -(self.dynamic_steering_effort(angle) - friction)/ k
        return [dy1_dt, dy2_dt]

    # --- Plotting Functions ---
    def plotmycoordinates(self, func, title, legend, ylabel, xlabel):
        reference = self.reference()
        num = len(func)
        for j in range(num):
            X2 = np.array([])
            y2 = np.array([])
            t2 = self.KPA_rotation_angle_vs_rack(self.rack_stroke)
            max = self.KPA_rotation_angle_vs_rack(-self.rack_stroke)
            opop = int(max - t2)
            for i in range(0,opop*reference.conversionstep):
                t2 = t2 + reference.step
                X2 = np.append(X2,func[j](t2)[1])
                y2 = np.append(y2,func[j](t2)[0])
            X2 = X2.reshape(-1,1)
            plt.plot(X2,-y2)
            plt.scatter(func[j](0)[1], -func[j](0)[0])
            plt.scatter(func[j](self.KPA_rotation_angle_vs_rack(self.rack_stroke))[1], -func[j](self.KPA_rotation_angle_vs_rack(self.rack_stroke))[0])
            print('Initial ' + legend[j*3] + ' is ' + '('+str(round(func[j](0)[1],2)) + ', ' + str(round(func[j](0)[0],2)) +')')
            print('Extreme Values are (' + str(round(func[j](max)[1],2)) + 
                  ', '+str(round(func[j](max)[0],2)) + 
                  ') and (' + str(round((func[j](self.KPA_rotation_angle_vs_rack(self.rack_stroke)))[1],2)) + 
                  ', '+ str(round((func[j](self.KPA_rotation_angle_vs_rack(self.rack_stroke)))[0],2))+')')
        plt.legend(legend)
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid()
    def plotmyfn(self, funcY, funcX, title, legend, ylabel, xlabel):
        reference = self.reference()
        num = len(funcY)
        for j in range(num):
            X2 = np.array([])
            y2 = np.array([])
            t2 = np.round(self.KPA_rotation_angle_vs_rack(self.rack_stroke),2)
            max = np.round(self.KPA_rotation_angle_vs_rack(-self.rack_stroke))
            opop = int(max - t2)
            for i in range(0,opop*reference.conversionstep):
                t2 = t2 + reference.step
                X2 = np.append(X2,funcX(t2))
                y2 = np.append(y2,funcY[j](t2))
            X2 = X2.reshape(-1,1)
            minim = round(np.min(y2),1)
            maxim = round(np.max(y2),1)
            plt.plot(X2,y2)
            plt.scatter( funcX(0),funcY[j](0))
            print('Initial ' + legend[j*2] + ' is ' + str(round(funcY[j](0),2)) )
            print('Extreme Values are ' + str(round(funcY[j](max),2)) + ' and ' + str(round((funcY[j](self.KPA_rotation_angle_vs_rack(self.rack_stroke))),2)))
            print('Range for ' + legend[j*2] + ' is ' + '['+ str(minim)+ ', '+str(maxim)+']')
            print('Average Absolute Value is ' + str(np.round(np.average(np.abs(y2)),2)))
        plt.legend(legend)
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid()
    