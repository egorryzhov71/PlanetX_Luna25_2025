import krpc
import time
import math
import os
import threading

# --- Вспомогательные функции ---

def clear_screen():
    # Очищает экран терминала
    os.system('cls' if os.name == 'nt' else 'clear')

def dot(u, v):
    # Скалярное произведение двух векторов
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]

def mag(v):
    # Евклидова норма вектора
    return math.sqrt(dot(v, v))

def cross(u, v):
    # Векторное произведение двух векторов
    return (u[1]*v[2] - u[2]*v[1],
            u[2]*v[0] - u[0]*v[2],
            u[0]*v[1] - u[1]*v[0])

def angle_between(u, v):
    # Возвращает угол между векторами (в градусах)
    cosine = dot(u, v) / (mag(u) * mag(v))
    cosine = max(-1, min(1, cosine))
    return math.degrees(math.acos(cosine))

# --- Настройка kRPC и глобальные переменные ---

conn = krpc.connect(name="kRPC Script")
vessel = conn.space_center.active_vessel
kerbin = vessel.orbit.body  # Предполагается, что тело – Kerbin
mun = conn.space_center.bodies['Mun']

# --- Функция логирования данных о полете ---

def flight_logger():
    # Путь для сохранения файла с данными
    file_path = os.path.join("data", "KSP_Stats.txt")
    # Создаем директорию, если ее нет
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    # Записываем заголовок файла
    with open(file_path, "w", encoding="utf-8") as f:
        f.write("time, mass, altitude, speed\n")
    # Бесконечный цикл логирования
    while True:
        try:
            flight = vessel.flight(kerbin.reference_frame)
            t = vessel.met
            m = vessel.mass
            alt = flight.mean_altitude
            speed = flight.speed
            log_line = f"{t:.1f}, {m:.1f}, {alt:.1f}, {speed:.1f}\n"
            with open(file_path, "a", encoding="utf-8") as f:
                f.write(log_line)
        except Exception as e:
            print("Ошибка логгера:", e)
        time.sleep(1)

# Запуск фонового потока для логирования
logger_thread = threading.Thread(target=flight_logger, daemon=True)
logger_thread.start()

# --- Основные функции сценария ---

def wait_before_start():
    clear_screen()
    timer = 10
    while timer > 0:
        print(timer)
        time.sleep(1)
        clear_screen()
        timer -= 1
    print("ПОЕХАЛИ!")
    time.sleep(1)

def first_stage():
    clear_screen()
    print("Запуск первой ступени: набор высоты")
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)
    vessel.control.throttle = 1
    vessel.control.activate_next_stage()
    
    fuel_index = vessel.resources.names.index("LiquidFuel")
    current_stage = vessel.control.current_stage
    while vessel.resources_in_decouple_stage(fuel_index, current_stage).amount >= 1:
        flight = vessel.flight(kerbin.reference_frame)
        altitude = flight.mean_altitude
        new_heading = 90 - 70 * (altitude / 50000)
        vessel.auto_pilot.target_pitch_and_heading(90, new_heading)
        time.sleep(0.2)
    vessel.auto_pilot.target_pitch_and_heading(90, 20)
    print("Топливо первой ступени закончилось, отсоединяю ступень")
    vessel.control.throttle = 0
    time.sleep(2)
    vessel.control.activate_next_stage()
    time.sleep(5)

def second_stage():
    clear_screen()
    print("Запуск второй ступени")
    vessel.control.throttle = 1
    while vessel.flight(kerbin.reference_frame).mean_altitude <= 70000:
        time.sleep(0.2)
    print("Сброс обтекателей")
    vessel.control.activate_next_stage()
    while vessel.orbit.apoapsis_altitude <= 202500:
        time.sleep(0.2)
    vessel.control.throttle = 0
    time.sleep(1)
    vessel.control.activate_next_stage()
    
def circle_orbit():
    clear_screen()
    print("Выход на апоцентр в 200 км")
    vessel.auto_pilot.target_pitch_and_heading(90, 0)
    while vessel.orbit.time_to_apoapsis >= 20:
        clear_screen()
        print("До апоцентра осталось: {:.1f} с".format(vessel.orbit.time_to_apoapsis))
        time.sleep(0.2)
    vessel.control.throttle = 1
    while vessel.orbit.periapsis_altitude <= 200000:
        clear_screen()
        print("Апоцентр: {:.1f} м, перицентр: {:.1f} м".format(vessel.orbit.apoapsis_altitude, vessel.orbit.periapsis_altitude))
        time.sleep(0.2)
    vessel.control.throttle = 0
    time.sleep(3)
    vessel.control.activate_next_stage()
    time.sleep(3)

def check_can_go_to_moon():
    vector_kerbin_mun = mun.position(kerbin.reference_frame)
    vector_kerbin_ship = vessel.position(kerbin.reference_frame)
    angle_ship_moon = angle_between(vector_kerbin_mun, vector_kerbin_ship)
    
    flight = vessel.flight(kerbin.reference_frame)
    ship_up = flight.up
    ship_velocity = vessel.velocity(kerbin.reference_frame)
    cross_vec = cross(ship_up, ship_velocity)
    ship_to_mun = (
        mun.position(kerbin.reference_frame)[0] - vector_kerbin_ship[0],
        mun.position(kerbin.reference_frame)[1] - vector_kerbin_ship[1],
        mun.position(kerbin.reference_frame)[2] - vector_kerbin_ship[2]
    )
    angle_adjust = angle_between(cross_vec, ship_to_mun)
    if angle_adjust > 90:
        angle_ship_moon = -angle_ship_moon

    flight_ship = vessel.flight(kerbin.reference_frame)
    a1 = (mag(vector_kerbin_mun) + flight_ship.mean_altitude + kerbin.equatorial_radius) / 2.0
    a2 = mag(vector_kerbin_mun)
    angle_true = 180 * (1 - (a1 / a2) ** 1.5)

    clear_screen()
    print("Текущий угол: {:.2f}°".format(angle_ship_moon))
    print("Необходимый угол: {:.2f}°".format(angle_true))
    return abs(angle_ship_moon - angle_true) < 3

def go_to_moon():
    clear_screen()
    print("Перелет к Луне: подлет к точке отлета")
    while not check_can_go_to_moon():
        time.sleep(0.5)
    print("Начинаю полет к Луне")
    prograde = vessel.velocity(kerbin.reference_frame)
    vessel.auto_pilot.target_direction(prograde)
    time.sleep(3)
    mun_altitude = mun.orbit.semi_major_axis - kerbin.equatorial_radius
    while vessel.orbit.apoapsis_altitude <= mun_altitude:
        throttle_val = 1 - 0.9 * (vessel.orbit.apoapsis_altitude / mun_altitude)
        vessel.control.throttle = max(0, min(throttle_val, 1))
        time.sleep(0.2)
    vessel.control.throttle = 0

def find_x():
    clear_screen()
    b1 = vessel.mass / vessel.max_thrust
    flight_mun = vessel.flight(mun.reference_frame)
    altitude = flight_mun.mean_altitude
    b2 = (flight_mun.speed ** 2) / (2 * altitude) if altitude != 0 else 0
    g = (6.67e-11 * 9.76e20) / ((200000 + altitude) ** 2)
    true_force = b1 * (b2 + g)
    work_force = true_force
    if true_force > 0.5:
        work_force = 0.5 + (true_force - 0.5) * 3
    if work_force > 1:
        work_force = 1
    denominator = (vessel.max_thrust * true_force / vessel.mass) - g
    t = flight_mun.speed / denominator if denominator != 0 else float('inf')

    print("Начало посадки")
    print("Скорость: {:.1f} м/с".format(flight_mun.speed))
    print("Высота: {:.1f} м".format(altitude))
    print("Масса: {:.1f} кг".format(vessel.mass))
    print("Гравитация: {:.2f} м/с²".format(g))
    print("Необходимая тяга: {:.1f}%".format(true_force * 100))
    print("Фактическая тяга: {:.1f}%".format(work_force * 100))
    print("Ориентировочно через: {:.1f} с".format(t))
    return work_force

def on_the_moon():
    clear_screen()
    print("Полет к Луне")
    while vessel.orbit.body != mun:
        time.sleep(0.5)
    print("В орбите Луны")
    time.sleep(10)
    vessel.control.activate_next_stage()  # Отсоединение ступени спуска
    if vessel.orbit.periapsis_altitude > 1000:
        print("Корректировка орбиты")
        vessel.auto_pilot.target_pitch_and_heading(90, 0)
        while vessel.orbit.periapsis_altitude >= 1000:
            vessel.control.throttle = 1
            print("Перицентр: {:.1f} м".format(vessel.orbit.periapsis_altitude))
            time.sleep(0.2)
            clear_screen()
        time.sleep(5)
        vessel.control.throttle = 0
        print("Подготовка к посадке")
    while vessel.flight(mun.reference_frame).mean_altitude >= 100000:
        time.sleep(0.2)
    print("Начало посадки")
    surface_velocity = vessel.velocity(mun.reference_frame)
    retrograde = (-surface_velocity[0], -surface_velocity[1], -surface_velocity[2])
    vessel.auto_pilot.target_direction(retrograde)
    vessel.control.gear = True
    time.sleep(5)
    while vessel.flight(mun.reference_frame).mean_altitude > 1:
        vessel.control.throttle = 1 * find_x()
        time.sleep(0.2)
    vessel.auto_pilot.disengage()
    vessel.control.throttle = 0
    clear_screen()
    print("Посадка завершена, миссия успешна!")
    while True:
        time.sleep(1)

# --- Основная последовательность ---

def main():
    vessel.auto_pilot.reference_frame = kerbin.reference_frame
    wait_before_start()
    first_stage()
    second_stage()
    circle_orbit()
    go_to_moon()
    on_the_moon()

if __name__ == '__main__':
    main()
