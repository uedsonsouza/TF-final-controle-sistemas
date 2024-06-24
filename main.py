import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# Step 1: Define the Motor Transfer Function (G(s))
# Assuming motor parameters from the CTMS example
J = 3.2284E-6 # moment of inertia
b = 3.5077E-6   # motor viscous friction constant
K = 0.0274 # motor torque constant
R = 4    # electric resistance
L = 2.75E-6   # electric inductance

s = ctrl.TransferFunction.s
motor = ctrl.TransferFunction(K / ((J*s + b) * (L*s + R) + K**2))
print(">>>>> MOTOR",motor)

# Step 2: Calculate Kp (DC Gain of the Plant)
kp =  motor.dcgain()
print(">>>>> kp",kp)

# Step 3: Define the Proportional Controller (Kc)
kc = 1 / kp
print (">>>>> kc",kc)

# Step 4: Define the Sensor Transfer Function (H(s))
# Assuming initial τ based on given instructions
tau = (1/5) * 1.94  # Placeholder for tௗ, adjust based on stability
hs = 1 / (tau*s + 1)
print(">>>>> hs",hs)
print(">>>>> tau",tau)

# Step 5: Calculate the Open Loop Transfer Function
open_loop = kc * motor * hs
print(">>>>> open_loop",open_loop)
# Step 6: Stability Analysis (simplified here)
# Normally, you'd check for poles in the right half-plane or use a Nyquist plot
closed_loop = (motor * kc) / (1 + motor * kc * hs)

time_cl, response_cl = ctrl.step_response(open_loop, T=np.linspace(0, 2, 1000))

# Find Td: Time to reach 50% of the final value
final_value = response_cl[-1]
half_final_value = 0.5 * final_value
Td_index = np.where(response_cl >= half_final_value)[0][0]  # Index of the first occurrence
Td = time_cl[Td_index]

print(f'Td: {Td} seconds')
# Step 7: Drawing the Open Loop Block Diagram is conceptual and not included in code

stepInfo = ctrl.step_info(closed_loop)
for i, j in stepInfo.items():
    print(f'{i} : {j}')
    
time, response = ctrl.step_response(open_loop)
time_cl, response_cl = ctrl.step_response(closed_loop)
plt.figure()
plt.plot(time_cl, response_cl, label='Closed-Loop Response')
plt.title('Closed-Loop Step Response')
plt.xlabel('Time (seconds)')
plt.ylabel('Response')
plt.legend()

plt.plot(time, response)
plt.title('Open Loop Step Response')
plt.xlabel('Time (seconds)')
plt.ylabel('Response')
plt.show()