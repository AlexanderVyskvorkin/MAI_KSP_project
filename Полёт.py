import krpc
import math
import time

conn = krpc.connect(name='To the Mun')
vessel = conn.space_center.active_vessel
ut = conn.add_stream(getattr, conn.space_center, 'ut')

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
     pass
print('Нужный апоцентр достигнут')
vessel.control.throttle = 0.0

# Отделяем первую ступень на высоте 70 000 км
while altitude() < target_altitude * 0.7:
     pass
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
conn.space_center.warp_to(burn_ut - 10)

# Направляем ракету для манёвра
print('Направляем ракету для манёвра')
vessel.auto_pilot.reference_frame = maneuver_Kerbin.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Манёвр
print('Готовы к манёвру')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.0) > 1:
    pass
print('Выполняем манёвр')
vessel.control.throttle = 1.0
time.sleep(burn_time)

# Заканчиваем
vessel.control.throttle = 0.0
maneuver_Kerbin.remove()
print('Вышли на орбиту Кербина')





kerbin = conn.space_center.bodies['Kerbin']
mun = conn.space_center.bodies['Mun']

r1 = vessel.orbit.semi_major_axis
r2 = mun.orbit.semi_major_axis
GM = kerbin.gravitational_parameter

# Вычисляем время перелёта
transfer_time = math.pi * math.sqrt(((r1 + r2) / 2.0)**3 / GM)


# Вычисляем текущий угол между кораблём и положением Муны в момент встречи
mun_future_ut = conn.space_center.ut + transfer_time
mun_future_orbit = mun.orbit.position_at(mun_future_ut, kerbin.reference_frame)


# Вычисляем текущий угол между ракетой и положением Муны
vessel_pos = vessel.position(kerbin.reference_frame)
mun_pos = mun.position(kerbin.reference_frame)

dot = vessel_pos[0]*mun_pos[0] + vessel_pos[2]*mun_pos[2]
det = vessel_pos[0]*mun_pos[2] - vessel_pos[2]*mun_pos[0]
current_angle = math.atan2(det, dot)
    
if current_angle < 0:
    current_angle += 2*math.pi


# Вычисляем угол для манёвра к Муне
maneuver_angle = math.pi * (1 - math.sqrt((1/(8*r2**3)) * (r1 + r2)**3))


# Вычисляем угловые скорости
vessel_omega = math.sqrt(GM / r1**3)
mun_omega = math.sqrt(GM / r2**3)


# Вычисляем разность углов
angle_diff = maneuver_angle - current_angle


# Если угол отрицательный
if angle_diff < 0:
    angle_diff += 2*math.pi


# Вычисляем время до нужного положения Муны
relative_omega = vessel_omega - mun_omega

time_to_maneuver = (2*math.pi - angle_diff) / abs(relative_omega)


# Корректируем, если время отрицательное
if time_to_maneuver < 0:
    time_to_maneuver += 2*math.pi / abs(relative_omega)


# Корректируем для неотрицательного перицентра
time_to_maneuver -= 90


# Вычисляем дельта V для манёвра к Муне
delta_v = math.sqrt(GM / r1) * (math.sqrt(2*r2/(r1 + r2)) - 1)


# Формула Циолковского
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate


# Добавляем манёвр
to_the_mun_node = vessel.control.add_node(conn.space_center.ut + time_to_maneuver, prograde=delta_v)


# Ждём времени начала манёвра
print('Ждём времени начала манёвра')
burn_ut = conn.space_center.ut + time_to_maneuver - (burn_time/2.0)
conn.space_center.warp_to(burn_ut - 15)


# Направляем ракету для манёвра
print('Направляем ракету для манёвра')
vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = to_the_mun_node.burn_vector(vessel.orbital_reference_frame)
vessel.auto_pilot.wait()


# Выполняем манёвр
print('Готовы к манёвру')
while conn.space_center.ut < to_the_mun_node.ut - (burn_time / 2.0):
    pass

print('Выполняем манёвр')
vessel.control.throttle = 1.0
time.sleep(burn_time)


# Заканчиваем
vessel.control.throttle = 0.0
to_the_mun_node.remove()
print('Летим до Муны')





# Ждём входа в сферу действия тяготения Муны
print('Ждём входа в сферу действия Муны')
conn.space_center.warp_to(conn.space_center.ut + vessel.orbit.time_to_soi_change)
while vessel.orbit.body.name != 'Mun':
    pass
print('Вошли в сферу действия тяготения Муны')


# Если в результате прошлого манёвра мы подлетим слишком близко к поверхности Муны, придётся выходить на орбиту меньшего радиуса
if vessel.orbit.periapsis >= 100000:
    target_altitude = 100000
else:
    terget_altitude = vessel.orbit.periapsis
target_orbit_radius = mun.equatorial_radius + target_altitude


# Ждём попадания в перицентр
conn.space_center.warp_to(conn.space_center.ut + vessel.orbit.time_to_periapsis - 30)

# Считаем дельта V
GM_mun = mun.gravitational_parameter

current_periapsis = vessel.orbit.periapsis
current_sma = vessel.orbit.semi_major_axis


current_velocity = math.sqrt(GM_mun * (2/current_periapsis + 1/abs(current_sma)))

target_velocity = math.sqrt(GM_mun / target_orbit_radius)
delta_v = current_velocity - target_velocity


# Формула Циолковского
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82 
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate


# Рассчитываем время начала торможения
burn_start_ut = conn.space_center.ut + vessel.orbit.time_to_periapsis - (burn_time / 2.0)


# Добавляем манёвр
mun_orbit_node = vessel.control.add_node(burn_start_ut + burn_time/2.0, prograde=-delta_v)


# Ждём времени начала манёвра
while burn_start_ut - conn.space_center.ut > 15:
    pass


# Направляем ракету для манёвра
print('Направляем ракету для манёвра')
vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = mun_orbit_node.burn_vector(vessel.orbital_reference_frame)
vessel.auto_pilot.wait()


# Выполняем манёвр
print('Готовы к манёвру')
while conn.space_center.ut < burn_start_ut:
    pass

print('Выполняем манёвр')
vessel.control.throttle = 1.0
time.sleep(burn_time)


# Заканчиваем манёвр
vessel.control.throttle = 0.0
mun_orbit_node.remove()
time.sleep(5)


# Заканчиваем
vessel.control.activate_next_stage()
vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = (1, 0, 0)
vessel.auto_pilot.wait()
vessel.auto_pilot.disengage()

print('Спутник успешно выведен на орбиту Муны')