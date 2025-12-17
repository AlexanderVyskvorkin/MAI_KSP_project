import matplotlib.pyplot as plt
import numpy as np
import krpc
import time
import math

# ОДНО подключение для всего
conn = krpc.connect(name='To the Mun')
vessel = conn.space_center.active_vessel

# Подготовка данных для графиков KSP
Time_ksp = []
Height_ksp = []
Speed_ksp = []
Orbit = vessel.orbit.body.reference_frame

# Продолжаем собирать данные во время полета
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

# Взлёт до нужного апоцентра и сбор данных
turn_angle = 0

while True:
    # Собираем данные для графиков
    Height_ksp.append(vessel.flight(Orbit).mean_altitude)
    Speed_ksp.append(vessel.flight(Orbit).speed)
    Time_ksp.append(vessel.met)
    
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
    # Продолжаем собирать данные
    Height_ksp.append(vessel.flight(Orbit).mean_altitude)
    Speed_ksp.append(vessel.flight(Orbit).speed)
    Time_ksp.append(vessel.met)
    pass
print('Нужный апоцентр достигнут')
vessel.control.throttle = 0.0

# Отделяем первую ступень на высоте 70 000 км
while altitude() < target_altitude * 0.7:
    # Продолжаем собирать данные
    Height_ksp.append(vessel.flight(Orbit).mean_altitude)
    Speed_ksp.append(vessel.flight(Orbit).speed)
    Time_ksp.append(vessel.met)
    pass
vessel.control.activate_next_stage()
print('Отделили первую ступень')

# Рассчёты для манёвра
print('Рассчитываем манёвр для формирования орбиты')
GM = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
v1 = math.sqrt(GM*((2.0/r)-(1.0/a1)))
v2 = math.sqrt(GM*((2.0/r)-(1.0/r)))
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

# Строим графики совмещенные
print("Строим совмещенные графики...")

# математическая модель (три этапа)
if len(Time_ksp) > 0:
    # Используем время из KSP для модели
    # Ниже данные, взятые из KSP
    full_time = min(110, Time_ksp[-1])  # общее время моделирования
    phase1_time = 60  # 60 секунд на полной тяге
    phase2_time = 9   # 9 секунд на 25% тяги
    phase3_time = full_time - (phase1_time + phase2_time)  # оставшееся время по инерции
    m0_model = 10.1 * 10 ** 3  # начальная масса ракеты в кг
    Ft_model_full = 205.16 * 10 ** 3   # полная тяга ускорителя 1-ой ступени в Н
    Ft_model_low = Ft_model_full * 0.25  # 25% от полной тяги 1-ой ступени в Н
    Isp = 265 # удельный импульс в сек
    G_model = 9.81 # ускорение свободного падения
    k_model_full = Ft_model_full / (Isp * G_model)  # скорость расхода топлива на полной тяге в кг/с
    k_model_low = Ft_model_low / (Isp * G_model)    # скорость расхода на 25% тяге в кг/с
    alpha1 = np.pi / 2
    alpha2 = np.pi / 4
    # Угол поворачивается только на первых двух этапах
    b_model = (alpha1 - alpha2) / (phase1_time + phase2_time)
    Cf_model = 0.03  # коэффициент лобового сопротивления
    diametr = 1.3
    S_model = math.pi * (diametr / 2) ** 2  # площадь лобового сопротивления

    # Константы
    e = 2.718 # экспонента
    shag = 0.1  # шаг интегрирования в сек

    molar_mass = 0.029 # молярная масса воздуха (кг/моль)
    R = 8.31 # газовая постоянная
    T = 300 # Температура атмосферы в К
    P_0 = 10 ** 5 # давление на уровне моря в Па

    # Массивы для математической модели
    y_values = []
    velocity_model = []
    
    # Начальные условия
    x = 0
    y = 0
    vx = 0
    vy = 0
    
    # Создаем временную шкалу для модели
    model_time_points = np.arange(0, full_time, shag)
    
    # ЭТАП 1: ПОЛНАЯ ТЯГА (0-60 сек) 
    # ЭТАП 2: 25% ТЯГИ (60-69 сек)
    # ЭТАП 3: ИНЕРЦИЯ (69+ сек) 
    
    for t in model_time_points:
        # Определяем текущий этап
        if t <= phase1_time:
            # ЭТАП 1: Полная тяга
            current_mass = m0_model - k_model_full * t
            current_thrust = Ft_model_full
            current_k = k_model_full
        elif t <= phase1_time + phase2_time:
            # ЭТАП 2: 25% тяги
            time_in_phase1 = phase1_time
            time_in_phase2 = t - phase1_time
            # Масса с учетом расхода на этапе 1 и этапе 2
            current_mass = m0_model - (k_model_full * time_in_phase1 + k_model_low * time_in_phase2)
            current_thrust = Ft_model_low
            current_k = k_model_low
        else:
            # ЭТАП 3: Движение по инерции
            time_in_phase1 = phase1_time
            time_in_phase2 = phase2_time
            # Масса постоянная (без израсходованного топлива)
            current_mass = m0_model - (k_model_full * time_in_phase1 + k_model_low * time_in_phase2)
            current_thrust = 0  # нет тяги
            current_k = 0 # нет скорости расхода топлива
        
        # Расчет аэродинамического сопротивления
        Ro = (molar_mass * P_0) / (R * T) * np.exp(-G_model * molar_mass * y / (R * T))
        f_cx = Cf_model * S_model * (Ro * (vx ** 2) * 0.5)
        f_cy = Cf_model * S_model * (Ro * (vy ** 2) * 0.5)
        
        # Угол наклона (поворачивается только на первых двух этапах)
        if t <= phase1_time + phase2_time:
            angle = np.pi/2 - b_model * t # угол уменьшается линейно со временем
        else:
            angle = np.pi/2 - b_model * (phase1_time + phase2_time) # угол фиксированный
        
        # Ускорения
        if current_thrust > 0:
            ax = (current_thrust * np.cos(angle) - f_cx) / current_mass
            ay = (current_thrust * np.sin(angle) - f_cy) / current_mass - G_model
        else:
            # Только сопротивление и гравитация
            ax = (-f_cx) / current_mass
            ay = (-f_cy) / current_mass - G_model
        
        # Интегрирование (метод Эйлера)
        vx = vx + ax * shag # новая скорость по Х
        vy = vy + ay * shag # новая скорость по Y 
        x = x + vx * shag # новая координата по X
        y = y + vy * shag # новая координата по Y
        
        y_values.append(y)
        velocity_model.append((vx ** 2 + vy ** 2) ** 0.5)
    
    # Преобразуем в numpy массивы
    y_values = np.array(y_values)
    velocity_model = np.array(velocity_model)
    
    # Рассчитываем параметры на момент перехода между этапами
    idx_60s = int(phase1_time / shag)
    idx_69s = int((phase1_time + phase2_time) / shag)
    
    y_at_60s = y_values[idx_60s] if idx_60s < len(y_values) else y_values[-1]
    v_at_60s = velocity_model[idx_60s] if idx_60s < len(velocity_model) else velocity_model[-1]
    y_at_69s = y_values[idx_69s] if idx_69s < len(y_values) else y_values[-1]
    v_at_69s = velocity_model[idx_69s] if idx_69s < len(velocity_model) else velocity_model[-1]
    
    mass_at_60s = m0_model - k_model_full * phase1_time
    mass_at_69s = mass_at_60s - k_model_low * phase2_time
    
    # ===== ГРАФИК ВЫСОТЫ (СОВМЕЩЕННЫЙ) =====
    plt.figure(figsize=(14, 6))
    
    # Левый график - высота
    plt.subplot(1, 2, 1)
    
    # График из KSP (синий)
    plt.plot(Time_ksp, Height_ksp, 'b-', linewidth=2, label='KSP-модель', alpha=0.7)
    
    # График из модели (красный пунктир)
    plt.plot(model_time_points, y_values, 'r-', linewidth=2, label='Математическа модель (3 этапа)', alpha=0.7)
    
    # Вертикальные линии разделения этапов
    plt.axvline(x=phase1_time, color='orange', linestyle=':', linewidth=1, alpha=0.5, label='25% тяги')
    plt.axvline(x=phase1_time+phase2_time, color='green', linestyle=':', linewidth=1, alpha=0.5, label='Выкл. двигатель')
    
    plt.xlabel("Время, сек")
    plt.ylabel("Высота, м")
    plt.title("Сравнение высоты: KSP vs Трехэтапная модель")
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Правый график - скорость
    plt.subplot(1, 2, 2)
    
    # График из KSP (синий)
    plt.plot(Time_ksp, Speed_ksp, 'b-', linewidth=2, label='KSP (факт)', alpha=0.7)
    
    # График из модели (красный пунктир)
    plt.plot(model_time_points, velocity_model, 'r-', linewidth=2, label='Модель (3 этапа)', alpha=0.7)
    
    # Вертикальные линии разделения этапов
    plt.axvline(x=phase1_time, color='orange', linestyle=':', linewidth=1, alpha=0.5, label='25% тяги')
    plt.axvline(x=phase1_time+phase2_time, color='green', linestyle=':', linewidth=1, alpha=0.5, label='Выкл. двигатель')
    
    plt.xlabel("Время, сек")
    plt.ylabel("Скорость, м/с")
    plt.title("Сравнение скорости: KSP vs Трехэтапная модель")
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    print("Графики построены успешно!")
else:
    print("Нет данных из KSP для построения графиков")

# Продолжаем полет к Муне
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


# Корректируем для неотрицательного перицентра (Это очень сложно посчитать из-за влияния тяготения Муны и т.д. значение эмпирическое)
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
time.sleep(burn_time - 0.5)


# Заканчиваем манёвр
vessel.control.throttle = 0.0
mun_orbit_node.remove()
time.sleep(5)


# Заканчиваем
vessel.control.activate_next_stage()
vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = (0, -1, 0)
vessel.auto_pilot.wait()
vessel.auto_pilot.disengage()

print('Спутник успешно выведен на орбиту Муны')