import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Load data from Excel
data = pd.read_excel('tiretrainingdata.xlsx')
alphaprime = data['alphaprime'].values
CFprime = data['CFprime'].values
SATprime = data['SATprime'].values

# Define the given equation
def CFprime_equation(alphaprime, B, C, D, E):
    return D * np.sin(C * np.arctan(B * ((alphaprime) - E * (alphaprime) + (E/B) * np.arctan(B * (alphaprime)))))

# Initial guesses for B, C, D, E
initial_guess_CFprime = [0.714, 1.40, 1.00, -0.20]

# Curve fitting
popt, pcov = curve_fit(
    CFprime_equation, alphaprime, CFprime, p0=initial_guess_CFprime, maxfev=50000000
)


# Extracting the optimal parameters
B_opt, C_opt, D_opt, E_opt = popt

print(f"Optimal parameters: B = {B_opt}, C = {C_opt}, D = {D_opt}, E = {E_opt}")
print(f"{B_opt}, {C_opt}, {D_opt}, {E_opt}")
# Plot the data and the fitted curve
# Plot the data and the fitted curve
plt.figure(figsize=(8,6))

# Scatter plot for the original data points
plt.scatter(
    alphaprime,
    CFprime,
    label="Data",
    color="#FFA07A",  # Soft salmon color
    edgecolor="#8B0000",  # Dark red edge for contrast
    s=70,
    alpha=0.85
)

# Line plot for the fitted curve
plt.plot(
    alphaprime,
    CFprime_equation(alphaprime, *popt),
    label="Fitted Curve",
    color="#4682B4",  # Soft steel blue
    linewidth=2
)

# Labels and Title
plt.xlabel("Alpha Prime ", fontsize=14, fontweight="bold")
plt.ylabel("CF Prime", fontsize=14, fontweight="bold")
plt.title("CF Curve Fitting", fontsize=16, fontweight="bold")

# Grid with lighter tones
plt.grid(color="#D3D3D3", linestyle="--", linewidth=0.7, alpha=0.6)

# Legend for clarity
plt.legend(fontsize=12, loc="best", frameon=True, shadow=False)

# Improve tick formatting
plt.tick_params(axis="both", which="major", labelsize=12)

# Add minor ticks for better scaling
plt.minorticks_on()
plt.tick_params(axis="both", which="minor", direction="in", length=5)

# Show the final plot
plt.tight_layout()
plt.show()

# Define the given equation
def SATprime_equation(alphaprime, B, C, D, E):
    return D * np.sin(C * np.arctan(B * ((alphaprime) - E * (alphaprime) + (E/B) * np.arctan(B * (alphaprime)))))

# Initial guesses for B, C, D, E
initial_guess_SATprime = [0.852,2.30,0.51,-2.75]

# Curve fitting
popt, pcov = curve_fit(
    SATprime_equation, alphaprime, SATprime, p0=initial_guess_SATprime, maxfev=50000000
)
# Extracting the optimal parameters
B_opt, C_opt, D_opt, E_opt = popt

print(f"Optimal parameters: B = {B_opt}, C = {C_opt}, D = {D_opt}, E = {E_opt}")
print(f"{B_opt}, {C_opt}, {D_opt}, {E_opt}")
# Plot the data and the fitted curve
# Plot the data and the fitted curve
plt.figure(figsize=(8,6))

# Scatter plot for the original data points
plt.scatter(
    alphaprime,
    SATprime,
    label="Data",
    color="#FFA07A",  # Soft salmon color
    edgecolor="#8B0000",  # Dark red edge for contrast
    s=70,
    alpha=0.85
)

# Line plot for the fitted curve
plt.plot(
    alphaprime,
    SATprime_equation(alphaprime, *popt),
    label="Fitted Curve",
    color="#4682B4",  # Soft steel blue
    linewidth=2
)

# Labels and Title
plt.xlabel("Alpha Prime", fontsize=14, fontweight="bold")
plt.ylabel("SAT Prime", fontsize=14, fontweight="bold")
plt.title("SAT Curve Fitting", fontsize=16, fontweight="bold")

# Grid with lighter tones
plt.grid(color="#D3D3D3", linestyle="--", linewidth=0.7, alpha=0.6)

# Legend for clarity
plt.legend(fontsize=12, loc="best", frameon=True, shadow=False)

# Improve tick formatting
plt.tick_params(axis="both", which="major", labelsize=12)

# Add minor ticks for better scaling
plt.minorticks_on()
plt.tick_params(axis="both", which="minor", direction="in", length=5)

# Show the final plot
plt.tight_layout()
plt.show()
