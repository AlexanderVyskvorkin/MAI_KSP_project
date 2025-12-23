import krpc
import math
import time
import matplotlib.pyplot as plt
import numpy as np

# Время расчётов
total_time = 300
time_1 = 60
time_2 = 9
time_3 = 112
time_4 = 54
time_5 = total_time - time_1 - time_2 - time_3 - time_4

# Характеристики ракеты
m_first_0 = 10074
m_second_0 = 3924
Ft_first_100 = 205161
Ft_first_25 = Ft_first_100 * 0.25
Ft_second = 90 * 10**3
Isp_first = 265
Isp_second = 345 
Kerbin_mass = 5.2915793 * 10**22
G = 6.67430 * 10**-11
g0 = 9.81

# Расход топлива
k_first_100 = Ft_first_100 / (g0 * Isp_first)
k_first_25 = Ft_first_25 / (g0 * Isp_first)
k_second = Ft_second / (g0 * Isp_second)

# Аэродинамика 
Cf = 0.03
radius = 0.65
S = math.pi * (radius) ** 2

#Константы для вычислений
dt = 0.01
airM = 0.029
ro0 = 1.2255
R = 8.31
T = 300
Kerbin_radius = 600000

# Вычисления и построение графиков
time_points = np.arange(0, total_time + dt, dt)
Vx = 0
Vy = 0
Velocity = 0
m = m_first_0
height = 0
alpha = 0
beta = 0

Vx_values = [Vx]
Vy_values = [Vy]
Velocity_values = [0]
height_values = [height]

for t in time_points:

    # Ускорение свободного падения в зависимости от высоты
    if height > 0:
        g = G * Kerbin_mass / (Kerbin_radius + height)**2
    else:
        g = g0

    k_first_100 = Ft_first_100 / (g * Isp_first)
    k_first_25 = Ft_first_25 / (g * Isp_first)
    k_second = Ft_second / (g * Isp_second)

    # Изменения тяги и массы 
    if t < time_1:
        m = m_first_0 - k_first_100 * t
        Ft = Ft_first_100
    elif t < time_1 + time_2:
        time_diff = t - time_1
        m = m_first_0 - k_first_100 * time_1 - k_first_25 * time_diff
        Ft = Ft_first_25
    elif t < time_1 + time_2 + time_3:
        m = m_first_0 - k_first_100 * time_1 - k_first_25 * time_2
        Ft = 0
    elif t < time_1 + time_2 + time_3 + time_4:
        m = m_second_0 - k_second * (t - time_1 - time_2 - time_3)
        Ft = Ft_second
    elif t > time_1 + time_2 + time_3 + time_4:
        m = m_second_0 - k_second * (time_1 + time_2 + time_3 + time_4)
        Ft = 0

    # Расчёты

    # Углы
    if height < 250 and t < time_1:
        alpha = 0
    elif height < 50000 and t < time_1:
        alpha = (height - 250)/(50000 - 250) * math.pi/2
    elif t > time_1 + time_2 + time_3:
        alpha = math.pi/2

    if t > time_1 + time_2 + time_3:
        beta = math.pi / 2
    elif Velocity != 0:
        beta = math.acos(Vy/math.sqrt(Vx**2 + Vy**2))

    # Сила сопротивления воздуха
    if height < 70000:
        ro = (airM * ro0) / (R * T) * np.exp(-g * airM * height / (R * T))

        Fc = Cf * ro * Velocity**2 * S / 2
    else:
        Fc = 0

    # Ускорение
    
    ax = (Ft * math.sin(alpha) - Fc * math.sin(beta)) / m
    ay = (Ft * math.cos(alpha) - Fc * math.cos(beta)) / m - g

    Vx = Vx + ax * dt
    Vy = Vy + ay * dt
    Velocity = math.sqrt(Vx**2 + Vy**2)


    if t > 194:
        Vy = 0
    if t > time_1 + time_2 + time_3 + time_4 + time_5:
        Vx = math.sqrt(G * Kerbin_mass / (Kerbin_radius + height))


    height = height + Vy * dt
    Vx_values.append(Vx)
    Vy_values.append(Vy)
    height_values.append(height)
    Velocity_values.append(Velocity)


plot_time_points = list(time_points) + [total_time]



figure, ax = plt.subplots()
Speed = []
Height = []
Time = []

conn = krpc.connect(name='To the Mun')
vessel = conn.space_center.active_vessel
ut = conn.add_stream(getattr, conn.space_center, 'ut')

Orbit = vessel.orbit.body.reference_frame

altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

# Подготовка к взлёту
turn_altitude_start = 250
turn_altitude_end = 50000
target_altitude = 100000

vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('Поехали!')

# Начало взлёта
vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

# Взлёт до нужного апоцентра
turn_angle = 0

while True:
    # Cобираем данные о полёте
    Speed.append(vessel.flight(Orbit).speed)
    Height.append(vessel.flight(Orbit).mean_altitude)
    Time.append(vessel.met)

    # Поворачиваем
    if turn_altitude_start < altitude() < turn_altitude_end:
         altitude_diff_parameter = ((altitude() - turn_altitude_start) /
                                    (turn_altitude_end - turn_altitude_start))
         new_turn_angle = altitude_diff_parameter * 90
         if abs(new_turn_angle - turn_angle) > 0.5:
              turn_angle = new_turn_angle
              vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)

    if apoapsis() > target_altitude * 0.9:
        print('Приближаемся к нужному апоцентру')
        break

# На небольшой тяге достигаем нужного апоцентра и отключаем двигатель
vessel.control.throttle = 0.25
while apoapsis() < target_altitude:
    # Cобираем данные о полёте
    Speed.append(vessel.flight(Orbit).speed)
    Height.append(vessel.flight(Orbit).mean_altitude)
    Time.append(vessel.met)

print('Нужный апоцентр достигнут')
vessel.control.throttle = 0.0

# Отделяем первую ступень на высоте 70 000 км
while altitude() < target_altitude * 0.7:
     # Cобираем данные о полёте
    Speed.append(vessel.flight(Orbit).speed)
    Height.append(vessel.flight(Orbit).mean_altitude)
    Time.append(vessel.met)

vessel.control.activate_next_stage()
print('Отделили первую ступень')

# Рассчёты для манёвра
print('Рассчитываем манёвр для формирования орбиты')
GM = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
v1 = math.sqrt(GM*((2.0/r)-(1.0/a1)))
v2 = math.sqrt(GM*(1.0/r))
delta_v = v2 - v1
maneuver_Kerbin = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Формула Циолковского для расчёта времени работы двигателя
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v/Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

# Ждём начала манёвра
print('Ждём времени начала манёвра для выхода на орбиту')
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.0)
while burn_ut - ut() > 10:
    # Cобираем данные о полёте
    Speed.append(vessel.flight(Orbit).speed)
    Height.append(vessel.flight(Orbit).mean_altitude)
    Time.append(vessel.met)

# Направляем ракету для манёвра
print('Направляем ракету для манёвра')
vessel.auto_pilot.reference_frame = maneuver_Kerbin.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Манёвр
print('Готовы к манёвру')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.0) > 1:
    # Cобираем данные о полёте
    Speed.append(vessel.flight(Orbit).speed)
    Height.append(vessel.flight(Orbit).mean_altitude)
    Time.append(vessel.met)

print('Выполняем манёвр')
vessel.control.throttle = 1.0
time.sleep(burn_time)

# Заканчиваем
vessel.control.throttle = 0.0
maneuver_Kerbin.remove()
print('Вышли на орбиту Кербина')

while True:
    # Cобираем данные о полёте
    Speed.append(vessel.flight(Orbit).speed)
    Height.append(vessel.flight(Orbit).mean_altitude)
    Time.append(vessel.met)
    if vessel.met > 300:
        break



fig, axes = plt.subplots(2, 1, figsize=(12, 10))

# Высота
axes[0].plot(plot_time_points, height_values, 'b-', linewidth=2, label='Математическая модель')
axes[0].plot(Time, Height, 'r-', linewidth=2, label='KSP')
axes[0].set_xlabel('Время, сек')
axes[0].set_ylabel('Высота, м')
axes[0].set_title('Сравнение высоты полёта')
axes[0].legend()
axes[0].grid(True, alpha=0.3)
axes[0].set_xlim(0, max(max(Time), total_time))

# Скорость
axes[1].plot(plot_time_points, Velocity_values, 'b-', linewidth=2, label='Математическая модель')
axes[1].plot(Time, Speed, 'r-', linewidth=2, label='KSP')
axes[1].set_xlabel('Время, сек')
axes[1].set_ylabel('Скорость, м/с')
axes[1].set_title('Сравнение скорости полёта')
axes[1].legend()
axes[1].grid(True, alpha=0.3)
axes[1].set_xlim(0, max(max(Time), total_time))

plt.tight_layout()
plt.show()