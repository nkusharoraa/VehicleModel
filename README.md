# Vehicle Dynamics Modeling

## Repository Link

[Steering Design](https://github.com/nkusharoraa/SteeringDesign)

## Project Overview

The `Vehicle` class is designed to model and analyze various aspects of vehicle dynamics, including steering geometry, suspension kinematics, and wheel loads. It uses numerical methods to solve equations related to vehicle kinematics and dynamics, aiming to provide a comprehensive understanding of vehicle behavior under different conditions.

## Objectives

1. **Model Initialization**:
   - Initialize vehicle parameters such as wheelbase, trackwidth, and suspension components.
   
2. **Steering Geometry Analysis**:
   - Compute critical parameters related to steering geometry, including Kingpin Axis (KPA) and caster.
   
3. **Suspension Kinematics**:
   - Solve equations for Front View Swing Arm (FVSA) and Side View Swing Arm (SVSA) suspensions.
   - Compute initial conditions for suspension systems.
   
4. **Vehicle Dynamics Analysis**:
   - Analyze vehicle dynamics based on kinematic equations and numerical solutions.
   
5. **Visualization and Interpretation**:
   - Visualize key results and parameters to interpret vehicle behavior.
   - Assess the impact of various parameters on vehicle performance.

6. **Implementation and Documentation**:
   - Implement vehicle dynamics calculations using Python.
   - Document the code, methodologies, and key findings for future reference.

## Example

```python
# Create a Vehicle instance with example parameters
vehicle = Vehicle(
    r_A=np.array([0, 0, 0]),
    r_B=np.array([1, 0, 0]),
    r_C=np.array([2, 0, 0]),
    r_O=np.array([0, 1, 0]),
    r_K=np.array([0, 0, 1]),
    slr=300,
    initial_camber=2,
    tw=1500,
    wb=2500,
    GVW=2000,
    b=1000,
    wheel_rate_f=50,
    wheel_rate_r=50,
    tire_stiffness_f=100,
    tire_stiffness_r=100,
    pinion=3,
    tirep=32,
    dila=5,
    r_La=np.array([0, 0, 0]),
    r_Lb=np.array([0, 0, 0]),
    r_strut=np.array([0, 0, 0]),
    r_Ua=np.array([0, 0, 0]),
    r_Ub=np.array([0, 0, 0])
)
```
Please put appropriate values to instantiate an object.



## Visualization Insights

Effective visualization helps in understanding vehicle dynamics and suspension behavior:

### 1. Steering Geometry

Visualize steering parameters such as:
- Kingpin Axis (KPA)
- Caster angle

### 2. Suspension Kinematics

Illustrate the kinematics of FVSA and SVSA systems:
- Suspension movement
- Geometry changes

### 3. Vehicle Dynamics

Plot key dynamics metrics to assess vehicle performance:
- Steering angles
- Load distribution

## Skills Utilized

- Vehicle dynamics modeling
- Suspension kinematics analysis
- Numerical methods and optimization
- Python programming for vehicle dynamics calculations
- Visualization of vehicle performance metrics

## Potential Impact

- Improved understanding of vehicle behavior and performance.
- Enhanced vehicle design and optimization.
- Data-driven insights for suspension and steering system improvements.

## Python Code

Here's the Python script for the `Vehicle` class and its methods.



### Inductive Development Flow

Define $` \zeta = [\alpha_L, \beta_L] `$ as the domain for the right wheel Kingpin angle (in degrees), where:
- $` \alpha_L `$ is the lock angle when steered right
- $` \beta_L `$ is the lock angle when steered left

For example, for QUTE, $` \zeta = [-47.31^\circ, 37.28^\circ] `$.

#### Case: $` \theta = 0^\circ `$

We know the initial position vectors:
$` (\mathbf{r}_K)_0, (\mathbf{r}_A)_0, (\mathbf{r}_B)_0, (\mathbf{r}_O)_0, (\mathbf{r}_T)_0, (\mathbf{r}_W)_0, (\mathbf{r}_C)_0, (\mathbf{r}_{ICF})_0, (\mathbf{r}_{ICS})_0 `$

Here, $` (\mathbf{r}_M)_0 `$ represents the initial position vector of point M.

#### For $` \theta = 0.1^\circ `$:

1. **Rotation about KPA**:
   - Find $` (\mathbf{r}_B), (\mathbf{r}_O), (\mathbf{r}_T), (\mathbf{r}_W) `$ as per the rotation about the Kingpin axis (KPA) given by $` (\mathbf{r}_A)_0 - (\mathbf{r}_K)_0 `$.

2. **Virtual Bump/Droop**:
   - Find $` (\mathbf{r}_T), (\mathbf{r}_W) `$ as per a virtual bump/droop, keeping the same Z-coordinate.

3. **Road-Steer Angle**:
   - Determine the road-steer angle as per $` \Delta (\mathbf{r}_{TW}) `$.

4. **Virtual Bump/Droop for Fixed Ground**:
   - Find the virtual bump/droop $` (\delta z)_{\theta=0.1^\circ} `$.

5. **Update Positions**:
   - Update $` (\mathbf{r}_O) `$ after accounting for the virtual bump/droop $` (\delta z)_{\theta=0.1^\circ} `$.
   - Find $` (\mathbf{r}_K), (\mathbf{r}_A) `$ based on the rotation about the KPA given by $` (\mathbf{r}_{ICF})_0 - (\mathbf{r}_{ICS})_0 `$ and the virtual bump/droop $` (\delta z)_{\theta=0.1^\circ} `$.
   - Update $` (\mathbf{r}_B) `$ considering the forced suspension path from $` \Delta (\mathbf{r}_K) `$ (or $` \Delta (\mathbf{r}_A) `$ if the steering arm is connected to point A).

6. **Tie-Rod Length Intersection**:
   - Find $` (\mathbf{r}_C) `$ based on the intersection with $` (\mathbf{r}_C)_0 + \lambda \mathbf{j} `$, where $` \lambda `$ is a scalar.

7. **Rack Displacement**:
   - Calculate the rack displacement as per $` \Delta (\mathbf{r}_C) `$.

#### For a General $` \theta = \theta_0 \in \zeta `$:

Assume that after memoization, we know the position vectors:
$` (\mathbf{r}_K)_{\theta_0}, (\mathbf{r}_A)_{\theta_0}, (\mathbf{r}_B)_{\theta_0}, (\mathbf{r}_O)_{\theta_0}, (\mathbf{r}_T)_{\theta_0}, (\mathbf{r}_W)_{\theta_0}, (\mathbf{r}_C)_{\theta_0}, (\mathbf{r}_{ICF})_{\theta_0}, (\mathbf{r}_{ICS})_{\theta_0}, (\delta z)_{\theta_0} `$

Where $` (\mathbf{r}_M)_{\theta_0} `$ represents the position vector of point M at $` \theta = \theta_0 `$.

For $` \theta = \theta_0 \pm 0.1^\circ `$ :

1. **Rotation about KPA**:
   - Find $` (\mathbf{r}_B), (\mathbf{r}_O), (\mathbf{r}_T), (\mathbf{r}_W) `$ as per the rotation about the KPA given by $` (\mathbf{r}_A) - (\mathbf{r}_K) `$ at $` \theta = \theta_0 `$.

2. **Virtual Bump/Droop**:
   - Find $` (\mathbf{r}_T) `$ keeping the same Z-coordinate as $` (\mathbf{r}_T)_{\theta_0} `$.

3. **Total Road-Steer Angle**:
   - Determine the total road-steer angle as per $` \Delta (\mathbf{r}_{TW}) = (\mathbf{r}_{TW})_{\theta_0 \pm 0.1^\circ} - (\mathbf{r}_{TW})_0 `$.

4. **Virtual Bump/Droop for Fixed Ground**:
   - Find $` (\delta z)_{\theta_0 \pm 0.1^\circ} - (\delta z)_{\theta_0} `$.

5. **Update Positions**:
   - Update $` (\mathbf{r}_O) `$ after accounting for the virtual bump/droop $` (\delta z)_{\theta_0 \pm 0.1^\circ} - (\delta z)_{\theta_0} `$.
   - Find $` (\mathbf{r}_K), (\mathbf{r}_A) `$ based on the rotation about the KPA given by $` (\mathbf{r}_{ICF}) - (\mathbf{r}_{ICS}) `$ at $` \theta = \theta_0 `$ and the virtual bump/droop $` (\delta z)_{\theta_0 \pm 0.1^\circ} - (\delta z)_{\theta_0} `$.
   - Update $` (\mathbf{r}_B) `$ considering the forced suspension path from $` \Delta (\mathbf{r}_K) = (\mathbf{r}_K)_{\theta_0 \pm 0.1^\circ} - (\mathbf{r}_K)_{\theta_0} `$.

6. **Tie-Rod Length Intersection**:
   - Find $` (\mathbf{r}_C) `$ based on the intersection with $` (\mathbf{r}_C)_{\theta_0} + \lambda \mathbf{j} `$, where $` \lambda `$ is a scalar.

7. **Total Rack Displacement**:
   - Calculate the total rack displacement as per $` \Delta (\mathbf{r}_C) = (\mathbf{r}_C)_{\theta_0 \pm 0.1^\circ} - (\mathbf{r}_C)_{\theta_0} `$.

### Memoization

To facilitate this process, store the variables from $` 0 `$ to $` \alpha_L `$ in increments of $` 0.1^\circ `$ and from $` 0 `$ to $` \beta_L `$ in decrements of $` 0.1^\circ `$.


### Prerequisites

Ensure you have the following libraries installed:

- `numpy`
- `scipy`
- `matplotlib`

```bash
pip install numpy scipy matplotlib
```
