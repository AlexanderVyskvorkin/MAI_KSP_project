import matplotlib.pyplot as plt
import numpy as np
import math


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

# график высоты
plt.figure(1, figsize=(10, 6))
plt.plot(plot_time_points, height_values, 'r-', linewidth=2)
plt.xlabel("Время, с")
plt.ylabel("Высота, м")
plt.title("Высота ракеты от времени")
plt.grid(False)

# график скорости
figure, ax = plt.subplots()
ax.plot(plot_time_points, Velocity_values, 'r-', linewidth=2)  # Используем plot_time_points вместо time_points
ax.set_xlabel('Время, сек')
ax.set_ylabel('Скорость, м/с')
ax.set_title('Скорость ракеты от времени')
ax.grid(True, color='black', alpha=0.3)

plt.show()