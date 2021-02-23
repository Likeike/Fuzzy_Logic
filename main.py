import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from simple_pid import PID

Kc = 2.00
Ki = 0.04
Kd = 0.00
pid = PID(Kc, Ki, Kd)  # PID

# Dolny i górny limit kontrolera na wyjściu
oplo = 0.0
ophi = 1.0
pid.output_limits = (oplo, ophi)

# Czas próbkowania PID
# pid.sample_time = 1.0
pid.sample_time = 1.0


# Definiowanie modelu zbiornika za pomoca funkcji
def tank(levels, t, pump, valve):
    h1 = max(0.0, levels[0])
    h2 = max(0.0, levels[1])
    c1 = 0.08  # Współczynnik napełniania zbiornika
    c2 = 0.04  # Współczynnik odpływu
    dhdt1 = c1 * (1.0 - valve) * pump - c2 * np.sqrt(h1)
    dhdt2 = c1 * valve * pump + c2 * np.sqrt(h1) - c2 * np.sqrt(h2)

    # warunki przelania
    if h1 >= 1.0 and dhdt1 > 0.0:
        dhdt1 = 0
    if h2 >= 1.0 and dhdt2 > 0.0:
        dhdt2 = 0

    dhdt = [dhdt1, dhdt2]

    return dhdt


# Inicjacja  danych ( poziom )
h0 = [0.0, 0.0]

# Czas badania
tf = 300

t = np.linspace(0, tf, tf + 1)
pump = np.zeros((tf + 1))

# valve = 0, woda wpływa do górnego zbiornika
# valve = 1, woda wpływa do dolnego zbiornika
valve = 0

# Rejestracja wyników
y = np.empty((tf + 1, 2))
y[0, :] = h0

plt.figure(figsize=(6, 4.8))
plt.ion()
plt.show()

sp = np.zeros(tf + 1)
sp[5:] = 0.5

# Obsługa czasu i przerw
tm = np.zeros(tf + 1)
sleep_max = 1.01
start_time = time.time()
prev_time = start_time

# Symulacja zbiornika
for i in range(tf):
    # PID
    pid.setpoint = sp[i]
    pump[i] = pid(h0[1])

    pump[i] = h0[1]
    # Określenie pompy i zaworu
    inputs = (pump[i], valve)
    h = odeint(tank, h0, [0, 1], inputs)
    # Rejestracja wyników
    y[i + 1, :] = h[-1, :]
    h0 = h[-1, :]

    # # wyświetlanie wyniku
    # plt.clf()
    # plt.subplot(3,1,1)
    # plt.plot(t[0:i],y[0:i,0],'b-',label=r'$h_1$ PV')
    # plt.ylabel('Górny zbiornik (m)')
    # plt.legend(loc='best')
    # plt.subplot(3,1,2)
    # plt.plot(t[0:i],y[0:i,1],'r--',label=r'$h_2$ PV')
    # plt.plot(t[0:i],sp[0:i],'k:',label=r'$h_2$ SP')
    # plt.ylabel('Dolny zbiornik (m)')
    # plt.legend(loc='best')
    # plt.subplot(3,1,3)
    # plt.plot(t[0:i],pump[0:i],'k-',label='pompa')
    # plt.legend(loc='best')
    # plt.ylabel('Pompa')
    # plt.xlabel('Czas (sec)')
    # plt.pause(0.01)
    #
    # # Przerwa
    # sleep = sleep_max - (time.time() - prev_time)
    # if sleep>=0.01:
    #     time.sleep(sleep-0.01)
    # else:
    #     time.sleep(0.01)
    #
    # ct = time.time()
    # dt = ct - prev_time
    # prev_time = ct
    # tm[i+1] = ct - start_time

# Export danych
time_col = t.reshape(-1, 1)
pump_col = pump.reshape(-1, 1)
h2_col = y[:, 1].reshape(-1, 1)
my_data = np.concatenate((time_col, pump_col, h2_col), axis=1)
np.savetxt('pid_data.txt', my_data, delimiter=',')
