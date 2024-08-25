import control as ct
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd

s = ct.TransferFunction.s

# System parameters
m = 10
k = 0.5

# System transfer function
P = 1/(s*(m*s+k))

# Lead compensator parameters
kl = 0.4
tau_p = 1
tau_z = 18

# Lead compensator transfer function
C_L = kl*(tau_z*s+1)/(tau_p*s+1)

# Close-loop transfer function
G_L = ct.feedback(C_L*P, 1)

# Plot root-locus - Lead compensator
plt.figure()
ct.root_locus(C_L*P)

# Plot Bode diagram
plt.figure()
ct.bode(C_L*P)

# Calculate the gain and phase margins - Lead compensator
gm, pm, gc, pc = ct.margin(C_L*P)
gm_dB = 20*np.log10(gm)

# Print margins
print(f"Gain Margin: {gm_dB:.3g} dB at frequency {gc:.3g} rad/sec")
print(f"Phase Margin: {pm:.3g} deg at frequency {pc:.3g} rad/sec")
print(f"Delay Margin: {((pm*math.pi/180)/pc):.3g} seconds")

# Step response - nominal - output
t_L, yout_L = ct.step_response(G_L, T=20)
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t_L, yout_L, label="Lead compensator")
plt.ylabel("Position [m]")
plt.grid()
plt.title("Step response - nominal")

# Find transfer function from setpoint to control input (torque)
G_L_u = C_L/(1+C_L*P)

# Step response - nominal - control input
t_L, uout_L = ct.step_response(G_L_u, T=20)
plt.subplot(2, 1, 2)
plt.plot(t_L, uout_L, label="Lead compensator")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid()

plt.show()

# To run after the C code

# Read data from data.txt into a pandas DataFrame
data = pd.read_csv('data.txt', delim_whitespace=True, names=["Time", "Command", "Response", "Setpoint"])

# Plot response from C code
plt.subplot(2, 1, 1)
plt.plot(t_L, yout_L, label=f"Response - Ideal")
plt.plot(data["Time"], data["Response"], label=f"Response - from C Code")
plt.plot(data["Time"], data["Setpoint"], '--', label=f"Setpoint")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()

# Plot controller output
plt.subplot(2, 1, 2)
plt.plot(t_L, uout_L, label=f"Command - Ideal")
plt.plot(data["Time"], data["Command"], label=f"Command - from C Code")

plt.xlabel("Time [s]")
plt.ylabel("F [N]")
plt.legend()
plt.grid()

# Show plots
plt.show()