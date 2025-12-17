import matplotlib.pyplot as plt
import numpy as np
import math

# ===== ПАРАМЕТРЫ МОДЕЛИ =====
total_time = 110  # общее время моделирования в сек
phase1_time = 60  # 60 секунд на полной тяге
phase2_time = 9   # 9 секунд на 25% тяги
phase3_time = total_time - (phase1_time + phase2_time)  # время по инерции

m0 = 10.1 * 10 ** 3  # начальная масса корабля в кг
Ft_model_full = 205.16 * 10 ** 3   # полная тяга ускорителя в Н
Ft_model_low = Ft_model_full * 0.25  # 25% от полной тяги
Isp = 265  # удельный импульс
G_model = 9.81  # ускорение свободного падения

# Скорость расхода топлива
k_model_full = Ft_model_full / (Isp * G_model)  # скорость расхода топлива на полной тяге в кг/с
k_model_low = Ft_model_low / (Isp * G_model)    # скорость расхода на 25% тяге в кг/с

# Траектория
alpha1 = np.pi / 2  # начальный угол 90°
alpha2 = np.pi / 4  # конечный угол 45°
b_model = (alpha1 - alpha2) / (phase1_time + phase2_time)  # изменение угла рад/с

# Аэродинамика 
Cf = 0.03  # коэффициент сопротивления
diametr = 1.3  # диаметр ракеты
S_model = math.pi * (diametr / 2) ** 2  # площадь лобового сопротивления

# Константы
shag = 0.1  # шаг интегрирования в сек
e = 2.718 # экспонента
molar_mass = 0.029 # молярная масса воздуха (кг/моль)
R = 8.31 # газовая постоянная
T = 300 # Температура атмосферы в К
P_0 = 10 ** 5 # давление на уровне моря в Па

# ===== ИНТЕГРИРОВАНИЕ =====
x_values = [0]
y_values = [0]
vx_values = [0]
vy_values = [0]

x = 0
y = 0
vx = 0
vy = 0

# Временные точки
time_points = np.arange(0, total_time, shag)

for t in time_points:
    # Определяем текущий этап
    if t <= phase1_time:
        # Этап 1: полная тяга
        current_mass = m0 - k_model_full * t
        current_thrust = Ft_model_full
    elif t <= phase1_time + phase2_time:
        # Этап 2: 25% тяги
        time_in_phase1 = phase1_time
        time_in_phase2 = t - phase1_time
        current_mass = m0 - (k_model_full * time_in_phase1 + k_model_low * time_in_phase2)
        current_thrust = Ft_model_low
    else:
        # Этап 3: движение по инерции
        time_in_phase1 = phase1_time
        time_in_phase2 = phase2_time
        current_mass = m0 - (k_model_full * time_in_phase1 + k_model_low * time_in_phase2)
        current_thrust = 0
    
    # Плотность воздуха
    Ro = (molar_mass * P_0) / (R * T) * np.exp(-G_model * molar_mass * y / (R * T))
    
    # Сила сопротивления
    f_cx = Cf * S_model * (Ro * (vx ** 2) * 0.5)
    f_cy = Cf * S_model * (Ro * (vy ** 2) * 0.5)
    
    # Угол наклона
    if t <= phase1_time + phase2_time:
        angle = alpha1 - b_model * t
    else:
        angle = alpha1 - b_model * (phase1_time + phase2_time)
    
    # Ускорения
    if current_thrust > 0:
        ax = (current_thrust * np.cos(angle) - f_cx) / current_mass
        ay = (current_thrust * np.sin(angle) - f_cy) / current_mass - G_model
    else:
        ax = (-f_cx) / current_mass
        ay = (-f_cy) / current_mass - G_model
    
    # Интегрирование
    vx = vx + ax * shag
    vy = vy + ay * shag
    x = x + vx * shag
    y = y + vy * shag
    
    # Сохраняем значения
    vx_values.append(vx)
    vy_values.append(vy)
    x_values.append(x)
    y_values.append(y)

# ===== РАСЧЕТ СКОРОСТИ =====
velocity = []
for i in range(len(vx_values)):
    v = np.sqrt(vx_values[i] ** 2 + vy_values[i] ** 2)
    velocity.append(v)

# Временные точки для графиков (с учетом шага)
plot_time_points = list(time_points) + [total_time]

# ===== ГРАФИК 1: ВЫСОТА =====
plt.figure(1, figsize=(10, 6))
plt.plot(plot_time_points, y_values, 'b-', linewidth=2)
plt.axvline(x=phase1_time, color='orange', linestyle='--', linewidth=1)
plt.axvline(x=phase1_time+phase2_time, color='green', linestyle='--', linewidth=1)
plt.xlabel("Время, с")
plt.ylabel("Высота, м")
plt.title("Высота ракеты от времени")
plt.grid(True, alpha=0.3)

# ===== ГРАФИК 2: СКОРОСТЬ =====
plt.figure(2, figsize=(10, 6))
plt.plot(plot_time_points, velocity, 'b-', linewidth=2)
plt.axvline(x=phase1_time, color='orange', linestyle='--', linewidth=1)
plt.axvline(x=phase1_time+phase2_time, color='green', linestyle='--', linewidth=1)
plt.xlabel("Время, с")
plt.ylabel("Скорость, м/с")
plt.title("Скорость ракеты от времени")
plt.grid(True, alpha=0.3)

plt.show()