This code defines a Python class named **`Vehicle`** which is designed to model and analyse vehicle dynamics, steering, wheel loads, and geometric characteristics. It utilises several libraries for numerical methods, regression, and plotting, including `numpy`, `scipy.optimize.fsolve`, `scipy.integrate.solve_ivp`, `sklearn.linear_model.LinearRegression`, `matplotlib.pyplot`, `sklearn.preprocessing.PolynomialFeatures`, `scipy.integrate`, `scipy.interpolate.interp1d`, and `scipy.misc.derivative`.

Here is a documentation of the key components and methods within the provided code excerpts:

**Class: `Vehicle`**

*   **Purpose:** Encapsulates methods and properties related to vehicle dynamics, steering, wheel loads, and geometric calculations. It uses numerical methods, regressions, and physical equations to model vehicle behaviour under different conditions. It provides insights into steering and suspension systems.

*   **Constructor (`__init__`)**
    *   **Inputs:** Takes numerous input parameters defining the vehicle's geometry (`r_A`, `r_B`, `r_C`, `r_O`, `r_K`, `r_La`, `r_Lb`, `r_strut`, `r_Ua`, `r_Ub`), suspension/tire properties (`slr`, `dlr`, `initial_camber`, `toe_in`, `wheel_rate_f`, `wheel_rate_r`, `tire_stiffness_f`, `tire_stiffness_r`, `tirep`, `tiredata`), physical characteristics (`tw`, `wb`, `GVW`, `CG_height`, `I_w`, `I_ss`), steering parameters (`assumed_rack_stroke`, `pinion`), dynamic analysis factors (`CF_Loads`, `CF_Stiffnessrad`, `CF_pneumatictrail`), speed (`speed`), and linkage effort (`linkage_effort`). Note that speed is converted from km/h to m/s.
    *   **Initialisation:**
        *   Sets various input parameters as instance variables (`self.align_factor`, `self.thetaforcamber`, `self.assumed_rack_stroke`, `self.pinion`, `self.speed`, `self.I_w`, `self.I_ss`, `self.CF_Loads`, `self.CF_Stiffnessrad`, `self.CF_pneumatictrail`, `self.linkage_friction_contribution_on_steering`).
        *   Creates two internal objects (likely instances of a helper class or a simplified `VehicleState` object) using the `create_object` class method: `self.static` (using `slr`) and `self.dynamic` (using `dlr`).
        *   Initialises the `dynamic_analysis` flag.
        *   Calls `regression_model` to train models for inverse calculations.
        *   Calculates and stores the initial `rack_stroke`.
        *   Initialises arrays (`slipangles`, `Flguess`, `Frguess`, `Rlguess`, `Rrguess`) used for storing guesses in dynamic calculations.
        *   Initialises `patch_radius_left`, `patch_radius_right`, `tempdynamicsolution`, `tempdynamictheta`, and `move`.
        *   Calls `trainslipangles`.

*   **Class Method: `create_object(cls, ...)`**
    *   **Purpose:** A factory method to create lightweight objects (likely representing a specific state or configuration of the vehicle geometry). This seems to be used internally by the main `Vehicle` class constructor to set up static and dynamic states.
    *   **Inputs:** Similar geometric, suspension, and tire inputs as the main constructor, including `tire_radius` (which could be `slr` or `dlr`).
    *   **Actions:** Creates an empty object (`obj`), assigns input parameters as instance variables to `obj`, calculates additional points (`r_D`, `r_T`, `r_W`) based on initial geometry, tire radius, initial camber, and toe-in. Calculates and initialises the Kingpin Axis (`KPA`, `currKPA`). Initialises numerous arrays (`dpK`, `dpT`, `dpO`, `dpW`, `dpA`, `dpB`, `dpnewB`, `dpC`, `dpdz`, `dpfvsa`, `dpsvsa`) to store computed positions and displacements across different KPA angles. Sets up helper variables (`mindp`, `maxdp`, `step`, `zeropos`, `conversionstep`) for indexing these arrays. Assigns `CG_height`. Returns the configured `obj`.

*   **Method: `reference(self)`**
    *   **Purpose:** Returns the currently active reference object, which is either the `self.static` object or the `self.dynamic` object, determined by the `self.dynamic_analysis` flag.

*   **Helper Methods:**
    *   `magnitude(vector)`: Calculates the Euclidean magnitude of a vector.
    *   `safe_normalize(U)`: Normalises a vector, returning a zero vector if the input is zero.
    *   `matrix_multiply(*matrices)`: Performs matrix multiplication.
    *   `rotation(p, x1, x2, t)`: Rotates a point `p` by an angle `t` (in degrees) about an axis defined by points `x1` and `x2`.
    *   `project_points(x, y, z, a, b, c)`: Projects points onto a plane defined by the equation `ax + by + cz = 1`.

*   **Suspension Instantaneous Centre (IC) Calculations:**
    *   `fvsa_equations(self, values)`: Defines the equations for solving for the Front View Swing Arm (FVSA) Instantaneous Center. Computes equations based on geometric points (`current_A`, `current_K`, `r_Ua`, `r_Ub`, `r_La`, `r_Lb`, `r_strut`) and parameters `la`, `mu`. Returns a list of two equations representing differences in Y and Z components. Handles strut vs. no-strut configurations.
    *   `fvsa_ic(self, curr_KPA_angle)`: Computes the coordinates of the FVSA Instantaneous Center for a given KPA angle. Uses `fsolve` to solve the `fvsa_equations` for `la` and `mu`. Handles different strut configurations. Stores and retrieves previously computed ICs in the `dpfvsa` array.
    *   `svsa_equations(self, values)`: Defines the equations for solving for the Side View Swing Arm (SVSA) Instantaneous Center. Similar to `fvsa_equations` but calculates equations based on points and parameters for the side view. Returns a list of two equations representing differences in X and Z components. Handles strut vs. no-strut configurations.
    *   `svsa_ic(self, curr_KPA_angle)`: Computes the coordinates of the SVSA Instantaneous Center for a given KPA angle. Uses `fsolve` to solve the `svsa_equations` for `la` and `mu`. Handles different strut configurations. Stores and retrieves previously computed ICs in the `dpsvsa` array.

*   **Current Point Calculations:** These methods compute the positions of various points on the suspension and wheel assembly at a specific Kingpin Axis (KPA) angle. They often rely on previously computed positions and the FVSA/SVSA instantaneous centres to determine the current position via rotation. They typically store results in the respective `dp` arrays (e.g., `dpK`, `dpA`) for efficiency.
    *   `curr_K(self, curr_KPA_angle)`: Computes the position of point K (lower ball joint) based on the KPA angle. Uses stored data (`dpK`) and ICs (`fvsa_ic`, `svsa_ic`).
    *   `curr_KPA(self, curr_KPA_angle)`: Computes the direction vector of the current Kingpin Axis (KPA) based on the positions of A and K.
    *   `curr_A(self, curr_KPA_angle)`: Computes the position of point A (upper ball joint) based on the KPA angle. Uses stored data (`dpA`) and ICs.
    *   `curr_I(self, curr_KPA_angle)`: Computes the position of point I, which is the intersection of the KPA with the ground plane.
    *   `curr_B(self, curr_KPA_angle)`: Computes the position of point B (steering arm connection point) based on the KPA angle. Uses stored data (`dpB`) and ICs.
    *   `curr_C(self, curr_KPA_angle)`: Computes the position of point C (tie rod inner point) based on the KPA angle and the known length of the tie rod connecting it to point B.
    *   `curr_W(self, curr_KPA_angle)`: Computes the position of point W (wheel centre) based on the KPA angle. Uses stored data (`dpW`) and ICs.
    *   `curr_O(self, curr_KPA_angle)`: Computes the position of point O (spindle outer point) based on the KPA angle. Uses stored data (`dpO`) and ICs.
    *   `curr_T(self, curr_KPA_angle)`: Computes the position of point T (tire contact patch) based on the position of O and the wheel inclination.

*   **`solveO(self, inputval)`**
    *   **Purpose:** A helper function used internally by `curr_A`, `curr_K`, `curr_O`, `curr_W`. It is passed to `fsolve` to find a rotation parameter (`t`) that satisfies a geometric constraint, likely related to maintaining the wheel's vertical position or spindle orientation relative to the wheel travel (`delta_z`).

*   **`delta_z(self, curr_KPA_angle)`**
    *   **Purpose:** Computes the vertical displacement (wheel travel) of the tire contact patch for a given KPA angle relative to the initial position. Stores results in `dpdz`.

*   **Bump Steer Calculation:**
    *   `solvebump(self, inputval)`: Helper equation for `bump_steer`, related to wheel position under bump.
    *   `solveB(self, inputval)`: Helper equation for `bump_steer`, related to the tie rod end point position.
    *   `bump_steer(self, curr_KPA_angle, bump)`: Calculates the bump steer angle for a given KPA angle and vertical bump displacement. Uses `fsolve` with `solveB` and `solvebump`.

*   **Rack, Steering Arm, and Tie Rod:**
    *   `rack_displacement(self, curr_KPA_angle)`: Calculates the displacement of the steering rack from its initial position based on the KPA angle.
    *   `steering_arm(self, curr_KPA_angle)`: Calculates the vector representing the steering arm, from the KPA to point B (steering arm connection point).
    *   `tierod(self, curr_KPA_angle)`: Calculates the vector representing the tie rod, connecting point C (rack end) to point B (steering arm).

*   **Wheel Orientation and Trails:**
    *   `spindle(self, curr_KPA_angle)`: Calculates the vector representing the spindle (from W to O).
    *   `wheel_inclination(self, curr_KPA_angle)`: Calculates the direction vector representing the wheel's inclination (camber direction).
    *   `wheel_centre_axis(self, curr_KPA_angle)`: Calculates the horizontal component direction of the spindle vector.
    *   `wheel_heading(self, curr_KPA_angle)`: Calculates the direction vector representing the wheel's heading (direction of travel if rolling straight).
    *   `trails(self, curr_KPA_angle)`: Calculates the vector from the intersection of the KPA with the ground plane (`curr_I`) to the tire contact patch (`curr_T`).
    *   `caster_trail(self, curr_KPA_angle)`: Calculates the caster trail (longitudinal distance) by projecting the `trails` vector onto the wheel heading direction.
    *   `scrub_radius(self, curr_KPA_angle)`: Calculates the scrub radius (lateral distance) by projecting the `trails` vector onto a vector perpendicular to the wheel heading in the ground plane.

*   **Alignment Angles:**
    *   `camber(self, curr_KPA_angle)`: Calculates the camber angle (in degrees) by finding the angle between the spindle vector and the vertical axis.
    *   `road_steer(self, curr_KPA_angle)`: Calculates the road steer angle (in degrees) which is the angle between the current wheel heading and the initial (straight-ahead) wheel heading.
    *   `wheel_angle(self, curr_KPA_angle)`: Calculates the wheel angle (in degrees) relative to the vehicle's longitudinal axis.
    *   `caster(self, curr_KPA_angle)`: Calculates the caster angle (in degrees), which describes the longitudinal tilt of the Kingpin Axis.
    *   `kpi(self, curr_KPA_angle)`: Calculates the Kingpin Inclination (KPI) angle (in degrees), which describes the lateral tilt of the Kingpin Axis.

*   **Steering Ratios:**
    *   `steering_ratio(self, curr_KPA_angle)`: Calculates the ratio of Steering Wheel Angle to Road Steer Angle. Note it includes calculations involving rack displacement and pinion circumference.
    *   `steering_wheel_kpa_ratio(self, curr_KPA_angle)`: Calculates the ratio of Steering Wheel Angle to KPA rotation angle. Similar calculation involving rack displacement and pinion.

*   **Regression Modeling and Inverse Functions:** These methods use trained regression models to quickly estimate KPA angle or rack displacement from road steer or vice-versa, often refined with `fsolve`.
    *   `helperroadsteer(self, x)`: Helper function to wrap `road_steer` for use with `fsolve`.
    *   `helperrack(self, x)`: Helper function to wrap `rack_displacement` for use with `fsolve`.
    *   `regression_model(self)`: Trains linear regression models on polynomial features to map road steer angle to KPA angle and rack displacement to KPA angle. Returns the trained models and polynomial feature objects.
    *   `KPA_rotation_angle(self, input_road_steer)`: Uses the regression model and `fsolve` with `helperroadsteer` to find the KPA rotation angle corresponding to a given road steer angle.
    *   `rack_vs_road_steer(self, input_road_steer)`: Calculates the rack displacement corresponding to a given road steer angle by first finding the KPA angle using `KPA_rotation_angle`.
    *   `KPA_rotation_angle_vs_rack(self, input_rack_stroke)`: Uses the regression model and `fsolve` with `helperrack` to find the KPA rotation angle corresponding to a given rack displacement. Includes error handling with retry logic.
    *   `road_steer_vs_rack(self, input_rack_stroke)`: Calculates the road steer angle corresponding to a given rack displacement by first finding the KPA angle using `KPA_rotation_angle_vs_rack`.

*   **Ackermann Calculations:**
    *   `ackerman_radius(self, angle)`: Calculates the turning radius for a given steer angle based on wheelbase (`wb`).
    *   `ackerman_radius_from_inner(self, inner_angle)`: Calculates the turning radius from the inner wheel's angle.
    *   `ackerman_ideal_inner(self, inner_angle, ackerman_radius)`: Calculates the *ideal* outer wheel angle for 100% Ackermann steering based on a given inner angle and turning radius.
    *   `ackerman_ideal_outer(self, outer_angle, ackerman_radius)`: Calculates the *ideal* inner wheel angle for 100% Ackermann steering based on a given outer angle and turning radius.
    *   `ackerman_percentage(self, inner, outer)`: Calculates the Ackermann percentage based on the actual inner and outer steer angles.
    *   `ackerman_vs_KPA(self, curr_KPA_angle)`: Calculates the Ackermann percentage for a given KPA angle, comparing the steer angles of the inner and outer wheels.
    *   `tcr(self, outer_angle, inner_angle)`: Calculates the True Cornering Radius (TCR) based on inner and outer wheel angles, wheelbase (`a` and `b`), and track width (`t`).
    *   `ccr(self, outer_angle, inner_angle)`: Calculates the Center of Cornering Rotation (CCR) based on inner and outer wheel angles and track width.
    *   `ideal_ccr_inner(self, inner_angle)`: Calculates the ideal CCR for 100% Ackermann based on the inner wheel angle, wheelbase, and track width.
    *   `ideal_ccr_outer(self, outer_angle)`: Calculates the ideal CCR for 100% Ackermann based on the outer wheel angle, wheelbase, and track width.

*   **Tire Contact Patch Positions:** These methods calculate the positions of the tire contact patches, primarily relative to their initial positions.
    *   `delta_T(self, curr_KPA_angle)`: Calculates the displacement vector of the tire contact patch (`curr_T`) from its initial position (`reference.r_T`).
    *   `x_R(self, curr_KPA_angle)`: Returns the X coordinate of the right tire contact patch displacement.
    *   `y_R(self, curr_KPA_angle)`: Returns the Y coordinate of the right tire contact patch displacement.
    *   `x_L(self, curr_KPA_angle)`: Returns the X coordinate of the left tire contact patch displacement, calculated based on the rack displacement corresponding to the KPA angle.
    *   `y_L(self, curr_KPA_angle)`: Returns the Y coordinate of the left tire contact patch displacement, calculated based on the rack displacement corresponding to the KPA angle. Note the negation, suggesting a symmetric calculation.
    *   `z_R(self, curr_KPA_angle)`: Returns the vertical displacement (Z coordinate) of the right tire contact patch using `delta_z`.
    *   `z_L(self, curr_KPA_angle)`: Returns the vertical displacement (Z coordinate) of the left tire contact patch using `delta_z` and the KPA angle corresponding to the opposite rack displacement.

*   **Wheel Load Calculations (Static):**
    *   `F_Lz(self, curr_KPA_angle)`, `F_Rz(self, curr_KPA_angle)`, `R_Lz(self, curr_KPA_angle)`, `R_Rz(self, curr_KPA_angle)`: Return the static vertical loads on the front left, front right, rear left, and rear right wheels, respectively, by calling `staticsolve`.
    *   `FrontLoad(self, curr_KPA_angle)`: Returns the total static front axle load.
    *   `RearLoad(self, curr_KPA_angle)`: Returns the total static rear axle load (total weight minus front axle load).
    *   `FLRR(self, curr_KPA_angle)`, `FRRL(self, curr_KPA_angle)`: Calculate the sum of loads on the front-left and rear-right wheels, and front-right and rear-left wheels respectively.
    *   `staticequation(self, x)`: Defines the set of equations to be solved for static wheel load distribution. Includes vertical force balance, longitudinal moment balance, and side-to-side moment balance equations based on wheel displacements (`zfl`, `zfr`, `zrl`, `zrr`), spring rates (`Kf`, `Kr`), weight (`W`), CG position (`a`, `b`), track width (`t`), and tire contact patch positions (`x_L`, `y_L`, `z_L`, `x_R`, `y_R`, `z_R`).
    *   `staticsolve(self, theta)`: Solves the `staticequation` using `fsolve` to find the static wheel displacements and then calculates the static vertical wheel loads for a given KPA angle (`theta`). Uses initial guesses based on static weight distribution.

*   **Wheel Force Calculations (Dynamic):**
    *   `CF_L(self, curr_KPA_angle)`, `CF_R(self, curr_KPA_angle)`, `CR_L(self, curr_KPA_angle)`, `CR_R(self, curr_KPA_angle)`: Return dynamic cornering forces for the front left, front right, rear left, and rear right wheels from `dynamicsolve`.
    *   `NF_L(self, curr_KPA_angle)`, `NF_R(self, curr_KPA_angle)`, `NR_L(self, curr_KPA_angle)`, `NR_R(self, curr_KPA_angle)`: Return dynamic normal forces (vertical loads) for the front left, front right, rear left, and rear right wheels from `dynamicsolve`.
    *   `dynamicequation(self, x)`: Defines the set of equations for dynamic wheel load and slip angle calculations. Includes parameters like speed (`V`), turning radius (`R`), CG height (`h`), slip angles (`alphafL`, `alphafR`, `alpharL`, `alpharR`), cornering stiffness (`CFL`, `CFR`, `CRL`, `CRR`), tire properties (`B`, `C`, `D`, `E`, `mu`), and Ackermann geometry. Appears to balance lateral forces and yaw moments.
    *   `dynamicsolve(self, theta)`: Solves the `dynamicequation` using `fsolve` to find dynamic wheel loads (forces) and slip angles for a given KPA angle (`theta`). Incorporates logic for initial guesses, especially for negative KPA angles using previous results from `trainslipangles`. Includes error handling with retry logic that adjusts the angle (`self.move`). Returns Fl, Fr, Rl, Rr, alphafL, alphafR, alpharL, alpharR, CFL, CFR, CRL, CRR, and SAT.
    *   `trainslipangles(self)`: A method that calculates dynamic solutions (`dynamicsolve`) for a range of negative KPA angles (-1 degree down to -49 degrees) to pre-populate initial guesses for wheel loads and slip angles, used by `dynamicsolve`.

*   **Kingpin Moment (KPM) Calculations:** These methods calculate the torque about the Kingpin Axis, arising from forces acting on the tire contact patch.
    *   `circular_contactpatch_element(self, r, phi)`: Calculates the direction of motion for a point within the circular contact patch.
    *   `linear_interpolation(input_value)`: A helper function for linear interpolation, mapping a range of input values to a scaled output range.
    *   `tire_twisting_moment_circular_static(self, r, phi)`: Calculates the contribution of a contact patch element (`r`, `phi`) to the Kingpin Moment under static conditions. Considers friction force, normal force, and camber thrust, and projects their moment onto the KPA.
    *   `kpm_circular(self, theta)`: Calculates the total static Kingpin Moment by integrating `tire_twisting_moment_circular_static` over a circular contact patch. Uses tire pressure (`tirep`) and friction coefficient (`mu`).
    *   `dynamic_element_moment_circular_right(self, r, phi)`: Calculates the contribution of a contact patch element to the dynamic Kingpin Moment for the right wheel. Uses results from the dynamic solution (`self.tempdynamicsolution`), considering normal force, cornering force, and camber thrust, projecting their moment onto the KPA.
    *   `dynamic_element_moment_circular_left(self, r, phi)`: Calculates the contribution of a contact patch element to the dynamic Kingpin Moment for the left wheel. Similar to the right side, but uses parameters for the left wheel and considers the KPA angle corresponding to the opposite rack displacement.
    *   `kpm_circular_dynamic_right(self, theta)`: Calculates the total dynamic Kingpin Moment for the right wheel by integrating `dynamic_element_moment_circular_right`. Adds the Self Aligning Torque (SAT) contribution. Handles negative input angles by calling the left-side calculation. Stores the dynamic solution (`self.tempdynamicsolution`) and patch radius.
    *   `kpm_circular_dynamic_left(self, theta)`: Calculates the total dynamic Kingpin Moment for the left wheel by integrating `dynamic_element_moment_circular_left`. Adds the SAT contribution. Handles positive input angles by calling the right-side calculation. Stores the dynamic solution (`self.tempdynamicsolution`) and patch radius.
    *   `static_kingpin_moment(self, curr_KPA_angle)`: Calls `kpm_circular` to get the static KPM.
    *   `sat(self, alphafL, alphafR, Fl, Fr)`: Calculates the Self Aligning Torque (SAT) for the front left and right wheels based on their slip angles, normal loads, and specific tire data (`B`, `C`, `D`, `E`, `CF_Loads`, `CF_Stiffnessrad`, `CF_pneumatictrail`, and interpolators).

*   **Steering Effort Calculations:**
    *   `tierod_force(self, curr_KPA_angle)`: Calculates the force in the tie rod under static conditions based on the static Kingpin Moment, steering arm geometry, tie rod direction, and KPA direction.
    *   `tierod_force_dynamic_right(self, curr_KPA_angle)`: Calculates the force in the right tie rod under dynamic conditions, based on the dynamic Kingpin Moment for the right wheel and geometry.
    *   `tierod_force_dynamic_left(self, curr_KPA_angle)`: Calculates the force in the left tie rod under dynamic conditions, based on the dynamic Kingpin Moment for the left wheel and geometry.
    *   `rack_force_dynamic(self, curr_KPA_angle)`: Calculates the total force on the steering rack under dynamic conditions by summing the Y-components of the dynamic tie rod forces.
    *   `mechanical_advantage_linkages_static(self, curr_KPA_angle)`: Calculates a component of mechanical advantage related to the linkage geometry under static conditions.
    *   `mechanical_advantage_static(self, curr_KPA_angle)`: Calculates the overall mechanical advantage of the steering system under static conditions, relating tire forces (represented by KPM) to rack force and then to pinion torque.
    *   `mechanical_advantage_dynamic(self, curr_KPA_angle)`: Calculates the overall mechanical advantage under dynamic conditions.
    *   `rack_force(self, curr_KPA_angle)`: Calculates the total force on the steering rack under static conditions by summing the Y-components of the static tie rod forces, accounting for the inner and outer wheels at the given steering angle.
    *   `static_steering_effort(self, curr_KPA_angle)`: Calculates the steering effort (torque) at the steering wheel under static conditions, based on the static rack force, pinion size, and adds a linkage friction contribution.
    *   `dynamic_steering_effort(self, curr_KPA_angle)`: Calculates the steering effort at the steering wheel under dynamic conditions, based on the dynamic rack force, pinion size, and adds a linkage friction contribution. Includes logic to handle positive and negative KPA angles by effectively mirroring the calculation for one side.

*   **Returnability Simulation:**
    *   `returnability(self, lim_time)`: Simulates the steering system's return-to-center behaviour by solving an ordinary differential equation (ODE) over a specified time limit (`lim_time`). Uses `scipy.integrate.solve_ivp`. Compares solutions based on wheel inertia (`I_w`) and steering system inertia (`I_ss`). Returns the percentage of the initial angle recovered. Also includes plotting code for the solution.
    *   `wheel_system(self, t, Y, k)`: Defines the system of ODEs (`dy1_dt`, `dy2_dt`) for the steering wheel returnability simulation, modelled based on wheel inertia (`k=I_w`). Considers the dynamic Kingpin moments, linkage friction, and steering wheel-to-KPA ratio.
    *   `steering_system(self, t, Y, k)`: Defines an alternative system of ODEs for the returnability simulation, possibly modelled based on steering system inertia (`k=I_ss`). Considers dynamic steering effort and friction.

*   **Plotting Functions:**
    *   `plotmycoordinates(self, func, title, legend, ylabel, xlabel)`: Generates plots of 2D coordinates (like point positions) as a function of KPA angle (derived from rack stroke limits). Plots the trajectory, marks initial and extreme points, and prints their coordinates and ranges.
    *   `plotmyfn(self, funcY, funcX, title, legend, ylabel, xlabel)`: Generates plots of a function's output (`funcY`) against another function's output (`funcX`), typically both dependent on the KPA angle (derived from rack stroke limits). Plots the function, marks the initial point, and prints initial values, extreme values, range, and average absolute value.