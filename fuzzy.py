import skfuzzy as fuzz
import numpy as np
from skfuzzy import control as ctrl
import matplotlib as plt
import time
from scipy.integrate import odeint

# Definiowanie modelu zbiornika za pomoca funkcji
def tank(levels,t,pump,valve):
    h1 = max(0.0,levels[0])
    h2 = max(0.0,levels[1])
    c1 = 0.08 # Współczynnik napełniania zbiornika
    c2 = 0.04 # Współczynnik odpływu
    dhdt1 = c1 * (1.0-valve) * pump - c2 * np.sqrt(h1)
    dhdt2 = c1 * valve * pump + c2 * np.sqrt(h1) - c2 * np.sqrt(h2)

    # warunki przelania
    if h1>=1.0 and dhdt1>0.0:
        dhdt1 = 0
    if h2>=1.0 and dhdt2>0.0:
        dhdt2 = 0

    dhdt = [dhdt1, dhdt2]

    return dhdt


# Inicjacja  danych ( poziom )
h0 = [0.0,0.0]

# Czas badania
tf = 15

t = np.linspace(0,tf,tf+1)
pump = np.zeros((tf+1))

# valve = 0, woda wpływa do górnego zbiornika
# valve = 1, woda wpływa do dolnego zbiornika
valve = 0

# Rejestracja wyników
y = np.empty((tf+1,2))
y[0,:] = h0

# plt.figure(figsize=(6,4.8))
# plt.ion()
# plt.show()

sp = np.zeros(tf+1)
sp[5:] = 0.5

# Obsługa czasu i przerw
tm = np.zeros(tf+1)
sleep_max = 1.01
start_time = time.time()
prev_time = start_time

# Generowanie uniwersum
x_waterLevel = ctrl.Antecedent(np.arange(0.00, 1.00, 1), "level")
x_waterInput = ctrl.Consequent(np.arange(0, 0.08, 1), "flow")

x_waterLevel.automf(3)

# Klasyfikacja jakości
x_waterInput['slow'] = fuzz.trimf(x_waterInput.universe, [0.00, 0.00, 0.05])
x_waterInput['medium'] = fuzz.trimf(x_waterInput.universe, [0.00, 0.04, 0.08])
x_waterInput['fast'] = fuzz.trimf(x_waterInput.universe, [0.04, 0.08, 0.08])


# x_waterInput['slow'].view()

rule1 = ctrl.Rule(x_waterLevel['poor'], x_waterInput['fast'])
rule2 = ctrl.Rule(x_waterLevel['average'], x_waterInput['medium'])
rule3 = ctrl.Rule(x_waterLevel['good'], x_waterInput['slow'])


# Szykowanie obiektu do symulacji
floating_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
floating = ctrl.ControlSystemSimulation(floating_ctrl)
# floating.input['level'] = 0.03

#x_waterInput.view(sim=floating)

# Symulacja zbiornika
for i in range(tf):

    pump[i] = h0[1]
    # Określenie pompy i zaworu
    inputs = (pump[i],valve)
    h = odeint(tank,h0,[0,1],inputs)
    # Rejestracja wyników
    y[i+1,:] = h[-1,:]
    h0 = h[-1,:]
    floating.input['level'] = h
    floating.compute()
    print(floating.output['flow'])

    # Przerwa
    sleep = sleep_max - (time.time() - prev_time)
    if sleep>=0.01:
        time.sleep(sleep-0.01)
    else:
        time.sleep(0.01)

    ct = time.time()
    dt = ct - prev_time
    prev_time = ct
    tm[i+1] = ct - start_time



