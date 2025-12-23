import krpc
import math
import time
import matplotlib.pyplot as plt

figure, ax = plt.subplots()
Speed = []
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
    Time.append(vessel.met)

print('Нужный апоцентр достигнут')
vessel.control.throttle = 0.0

# Отделяем первую ступень на высоте 70 000 км
while altitude() < target_altitude * 0.7:
     # Cобираем данные о полёте
    Speed.append(vessel.flight(Orbit).speed)
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
    Time.append(vessel.met)
    if vessel.met > 300:
        break

ax.plot(Time, Speed)
plt.xlabel('Время, сек')
plt.ylabel('высота, м')
plt.grid(color='black') 
plt.show()