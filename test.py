import control as ctrl
import scipy as spy
import matplotlib.pyplot as plt
from scipy.integrate import simps
import numpy as np

J = 3.2284E-6
b = 3.5077E-6
K = 0.0274
R = 4
L = 2.75E-6

# Definir a função de transferência do motor
s = ctrl.TransferFunction.s
motor = ctrl.TransferFunction(K / (s*((J*s + b) * (L*s + R) + K*K)))
#Ganho estatico
kp =  motor.dcgain()
#Ganho do Controlador
kc = 0.01

g = ctrl.feedback(kc*motor, 1)

# Delay time calculation for closed loop system
_, y1 = ctrl.step_response(g)
td = next(x for x, val in enumerate(y1) if val >= 0.5 * y1[-1]) / 100  # Approximation

# Sensor transfer function adjustment based on td
tau = (1/5) * 1.704
hs = 1 / (tau * s + 1)

# Open loop transfer function correction
functiontransmalhaopen = kc * motor * hs

# Closed loop transfer function with sensor in feedback path
functiontransmalhaclose = ctrl.feedback(kc * motor, hs)

# Step response for open and closed loop
x, y = ctrl.step_response(functiontransmalhaopen)
x1, y1 = ctrl.step_response(functiontransmalhaclose)

# Step information for closed loop
stepInfo = ctrl.step_info(functiontransmalhaclose)
for i, j in stepInfo.items():
    print(f'{i} : {j}')

# Closed-loop transfer function is already calculated as functiontransmalhaclose
# Plotting for comparison
plt.figure(figsize=(10, 5))

def calculate_performance_indices(t, response):
    e = 1 - response  # Assuming unit step input, so error = 1 - output
    ISE = simps(e**2, t)
    IAE = simps(np.abs(e), t)
    ITAE = simps(t * np.abs(e), t)
    ITSE = simps(t * e**2, t)
    return ISE, IAE, ITAE, ITSE

# Define a function to simulate the closed-loop response for a given Kc
def simulate_response(Kc):
    g = ctrl.feedback(Kc*motor, 1)
    t, response = ctrl.step_response(g, np.arange(0, 10, 0.01))
    return t, response

# Iterate over a range of Kc values to find the optimal Kc for each performance index
Kc_values = np.linspace(0.01, 1, 100)  # Example range of Kc values
optimal_results = []

for Kc in Kc_values:
    t, response = simulate_response(Kc)
    ISE, IAE, ITAE, ITSE = calculate_performance_indices(t, response)
    optimal_results.append((Kc, ISE, IAE, ITAE, ITSE))

print (">>>>>>> ISE, IAE, ITAE, ITSE", ISE, IAE, ITAE, ITSE)
# Open Loop Response
plt.subplot(1, 2, 1)
plt.plot(x, y, label='Open Loop')
plt.title('Open Loop Response')
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.legend()

# Closed Loop Response
plt.subplot(1, 2, 2)
plt.plot(x1, y1, label='Closed Loop')
plt.title('Closed Loop Response')
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.legend()

plt.tight_layout()
plt.show()

print(">>>>>>>>>> KP", kp)