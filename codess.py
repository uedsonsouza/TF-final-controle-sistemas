import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from scipy.integrate import simps

# Parâmetros do motor
J = 3.2284E-6
b = 3.5077E-6
K = 0.0274
R = 4
L = 2.75E-6

# Definir a função de transferência do motor
s = ctrl.TransferFunction.s
motor = ctrl.TransferFunction(K / (s*((J*s + b) * (L*s + R) + K*K)))
print(">>>>> MOTOR", motor)

# Calcular Kp (Ganho DC do motor)
kp = 0.01
print(">>>>> kp", kp)

# Definir o controlador proporcional (Kc)
kc_initial = 1 / kp
print(">>>>> kc_initial", kc_initial)

# Definir a função de transferência do sensor (H(s))
tau = (1/5) * 1.94
hs = 1 / (tau*s + 1)
print(">>>>> hs", hs)
print(">>>>> tau", tau)

# Função para calcular os índices de desempenho
def calculate_performance_indices(t, response):
    e = 1 - response  # Sinal de erro
    ISE = simps(e**2, t)
    IAE = simps(np.abs(e), t)
    ITAE = simps(t * np.abs(e), t)
    ITSE = simps(t * e**2, t)
    return ISE, IAE, ITAE, ITSE

# Simular e analisar o desempenho para uma faixa de valores de Kc
Kc_values = np.linspace(0.1, 10, 100)
performance_indices = []

for kc in Kc_values:
    closed_loop = (motor * kc) / (1 + motor * kc * hs)
    t, response = ctrl.step_response(closed_loop, T=np.linspace(0, 10, 1000))
    indices = calculate_performance_indices(t, response)
    performance_indices.append(indices)

# Converter para array numpy para análise mais fácil
performance_indices = np.array(performance_indices)

# Encontrar Kc ótimo para cada índice de desempenho
optimum_Kc = Kc_values[np.argmin(performance_indices, axis=0)]

# Plotar respostas ao degrau unitário para valores ótimos de Kc
plt.figure(figsize=(10, 8))
for i, kc in enumerate(optimum_Kc):
    closed_loop = (motor * kc) / (1 + motor * kc * hs)
    t, response = ctrl.step_response(closed_loop, T=np.linspace(0, 10, 1000))
    plt.plot(t, response, label=f'Optimum Kc for {["ISE", "IAE", "ITAE", "ITSE"][i]}: {kc:.2f}')

plt.title('Unit Step Responses for Optimum Kc Values')
plt.xlabel('Time (seconds)')
plt.ylabel('Response')
plt.legend()
plt.show()

# Exibir valores ótimos de Kc
for i, kc in enumerate(optimum_Kc):
    print(f'Optimum Kc for {["ISE", "IAE", "ITAE", "ITSE"][i]}: {kc:.2f}')

# Análise de estabilidade (simplificada aqui)
closed_loop = (motor * kc_initial) / (1 + motor * kc_initial * hs)
time_cl, response_cl = ctrl.step_response(closed_loop, T=np.linspace(0, 2, 1000))

# Encontrar Td: Tempo para atingir 50% do valor final
final_value = response_cl[-1]
half_final_value = 0.5 * final_value
Td_index = np.where(response_cl >= half_final_value)[0][0]  # Índice da primeira ocorrência
Td = time_cl[Td_index]
print(f'Td: {Td} seconds')

# Informações da resposta ao degrau
stepInfo = ctrl.step_info(closed_loop)
for i, j in stepInfo.items():
    print(f'{i} : {j}')

# Plotar respostas ao degrau de malha aberta e fechada
open_loop = kc_initial * motor * hs
time, response = ctrl.step_response(open_loop)
time_cl, response_cl = ctrl.step_response(closed_loop)

plt.figure()
plt.plot(time_cl, response_cl, label='Closed-Loop Response')
plt.title('Closed-Loop Step Response')
plt.xlabel('Time (seconds)')
plt.ylabel('Response')
plt.legend()

plt.plot(time, response, label='Open Loop Response')
plt.title('Open Loop Step Response')
plt.xlabel('Time (seconds)')
plt.ylabel('Response')
plt.legend()
plt.show()
