import control as ctrl
import scipy as spy
import matplotlib.pyplot as plt

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

# Tal 
tal = (1/5)*1.94

# Função de Transferencia do Sensor
hs = 1/(tal*s+1)

#Função de Transferencia em Malha Aberta
functiontransmalhaopen = kc*motor+hs

#Função de Transferencia em Malha Fechada
functiontransmalhaclose = ctrl.feedback(kc*motor, hs)


x, y = ctrl.step_response(functiontransmalhaopen)
x1, y1 = ctrl.step_response(functiontransmalhaclose)

stepInfo = ctrl.step_info(functiontransmalhaclose)
for i, j in stepInfo.items():
    print(f'{i} : {j}')

plt.plot(x, y)
plt.plot(x1, y1)

plt.show()

print(">>>>>>>>>> KP",kp)