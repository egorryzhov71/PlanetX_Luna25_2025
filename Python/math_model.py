import os
import numpy as np

# =============================================================================
# Константы
# =============================================================================
class Constants:
    """
    Содержит параметры моделирования, физические константы и параметры ракеты
    для Союз-2.1б с модулем Фрегат, запускающей полезную нагрузку Луна-25.
    """
    # Параметры моделирования
    TIME_STEP = 0.02       # [с] шаг времени для каждого обновления симуляции
    COUNT_SKIP = 7000      # количество шагов интегрирования, используемых для оценки траектории
    MULTIPLIER = 100       # множитель для внутренних интеграционных циклов

    # Физические константы
    G = 6.67e-11           # [м^3/(кг*с^2)] гравитационная постоянная
    P0 = 101325            # [Па] атмосферное давление на уровне моря
    M_AIR = 0.02898        # [кг/моль] молярная масса воздуха
    R_UNIVERSAL = 8.314    # [Дж/(моль*K)] универсальная газовая постоянная
    R_SPECIFIC = 287.05    # [Дж/(кг*K)] удельная газовая постоянная для сухого воздуха
    T0 = 288.2             # [K] опорная температура

    # Параметры Земли (упрощённые/масштабированные)
    EARTH_MASS = 5.29e22   # [кг] (масштабированное значение)
    EARTH_RADIUS = 600000  # [м] радиус Земли
    EARTH_ANG_VEL = 2.91e-4  # [рад/с] угловая скорость Земли

    # Параметры первой ступени Soyuz‑2.1b (бустеры + блок-ядро)
    DRY_MASS_FIRST_STAGE = 30000       # [кг] сухая масса первой ступени
    FUEL_MASS_FIRST_STAGE = 268800     # [кг] масса топлива первой ступени
    Q_FIRST_STAGE = 572                # [кг/с] номинальная скорость расхода топлива
    # Тяга (Н): линейная интерполяция с учётом давления
    F0_FIRST_STAGE = 813 * 4 * 1000      # [Н] тяга на уровне моря
    F1_FIRST_STAGE = 1000 * 4 * 1000     # [Н] (расчётная тяга в вакууме)
    # Удельный импульс (с): I0 – вакуумный, I1 – на уровне моря.
    I0_FIRST_STAGE = 313               # [с] вакуумный удельный импульс
    I1_FIRST_STAGE = 256               # [с] удельный импульс на уровне моря

    # Параметры верхней ступени Fregat
    DRY_MASS_SECOND_STAGE = 4000       # [кг] сухая масса Fregat
    FUEL_MASS_SECOND_STAGE = 8000      # [кг] масса топлива для Fregat
    Q_SECOND_STAGE = 11.15             # [кг/с] номинальная скорость расхода (приблизительно)
    F0_SECOND_STAGE = 35000            # [Н] тяга (Fregat работает в вакууме)
    F1_SECOND_STAGE = 35000            # [Н] (без атмосферных изменений)
    I0_SECOND_STAGE = 320              # [с] удельный импульс в вакууме
    I1_SECOND_STAGE = 320              # [с] (не используется, так как Fregat не работает в атмосфере)

    # Полезная нагрузка: Luna‑25
    OTHER_MASS = 1200                  # [кг] масса посадочного модуля Luna‑25

    # Аэродинамические свойства (приблизительно)
    C_D = 0.115                        # коэффициент сопротивления
    A_EFF = 1.77                       # [м^2] эффективная площадь поперечного сечения


# =============================================================================
# Вспомогательные функции для работы с векторами
# =============================================================================
class VectorMath:
    """Статические методы для общих операций с 3D-векторами."""
    
    @staticmethod
    def length(v):
        """Возвращает Евклидову норму вектора v."""
        return np.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    
    @staticmethod
    def angle(v1, v2):
        """
        Возвращает угол (в радианах) между векторами v1 и v2.
        """
        len1, len2 = VectorMath.length(v1), VectorMath.length(v2)
        if len1 and len2:
            dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
            cos_angle = np.round(dot/(len1*len2), 5)
            return np.arccos(cos_angle)
        return 0.0
    
    @staticmethod
    def project_vector_onto_vector(v1, v2):
        """Возвращает проекцию вектора v1 на вектор v2."""
        dot = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
        denom = v2[0]**2 + v2[1]**2 + v2[2]**2
        return [v2[0]*dot/denom, v2[1]*dot/denom, v2[2]*dot/denom]
    
    @staticmethod
    def project_vector_onto_plane(v, normal):
        """Возвращает проекцию вектора v на плоскость, перпендикулярную вектору normal."""
        len_v = VectorMath.length(v)
        len_n = VectorMath.length(normal)
        angle_val = VectorMath.angle(v, normal)
        return [
            v[0] - normal[0] * len_v * np.cos(angle_val) / len_n,
            v[1] - normal[1] * len_v * np.cos(angle_val) / len_n,
            v[2] - normal[2] * len_v * np.cos(angle_val) / len_n
        ]
    
    @staticmethod
    def rotate_vector(v, angle_deg):
        """
        Поворачивает вектор v в плоскости x-z на угол angle_deg (в градусах).
        """
        angle_rad = np.deg2rad(angle_deg)
        if v[2] < 0:
            alpha = np.arctan(v[0]/v[2]) - angle_rad + np.pi
        elif v[2] > 0:
            alpha = np.arctan(v[0]/v[2]) - angle_rad
        else:
            alpha = -angle_rad
        return [np.sin(alpha), 0, np.cos(alpha)]
    
    @staticmethod
    def find_angle(v):
        """
        Возвращает угол вектора v относительно оси z (в плоскости x-z).
        """
        if v[2] < 0:
            return np.arctan(v[0]/v[2]) + np.pi
        elif v[2] > 0:
            return np.arctan(v[0]/v[2])
        else:
            return np.pi/2


# =============================================================================
# Ракета (Математическая модель)
# =============================================================================
class Rocket:
    """
    Представляет состояние и физику летательного аппарата.
    В этой модели:
      - Стадия 0 — первая ступень Союз-2.1б.
      - Стадия 1 — верхняя ступень Фрегат.
    Полезная нагрузка Луна-25 добавляется как OTHER_MASS.
    """
    # Начальные условия: расположена на поверхности Земли с начальной касательной скоростью
    time = 0.0
    position = [Constants.EARTH_RADIUS, 0, 0]
    velocity = [0, 0, Constants.EARTH_ANG_VEL * Constants.EARTH_RADIUS]
    
    # Фиксированные управляющие установки (для упрощения)
    throttle = 1.0
    steering = 90.0  # [градусов]
    
    # Стадия (0: первая ступень, 1: верхняя ступень)
    stage = 0
    fuel_first_stage = Constants.FUEL_MASS_FIRST_STAGE
    fuel_second_stage = Constants.FUEL_MASS_SECOND_STAGE

    @staticmethod
    def altitude():
        """Возвращает текущую высоту над поверхностью Земли [м]."""
        return VectorMath.length(Rocket.position) - Constants.EARTH_RADIUS

    @staticmethod
    def gravity():
        """Возвращает ускорение свободного падения на текущей высоте [м/с²]."""
        alt = Rocket.altitude()
        return Constants.G * Constants.EARTH_MASS / (Constants.EARTH_RADIUS + alt)**2

    @staticmethod
    def temperature():
        """Возвращает атмосферную температуру [K] по кусочно-линейной модели."""
        h = Rocket.altitude()
        if h <= 11000:
            return Constants.T0 - 6.5 * h / 1000
        elif h <= 20000:
            return Constants.T0 - 71.5
        elif h <= 50000:
            return Constants.T0 - 71.5 + 54 * (h - 20000) / 30000
        elif h <= 80000:
            return Constants.T0 - 17.5 - 72.1 * (h - 50000) / 30000
        elif h <= 100000:
            return Constants.T0 - 89.6
        return 1000

    @staticmethod
    def pressure_at_altitude():
        """
        Возвращает атмосферное давление [Па] на текущей высоте.
        (Экспоненциальное затухание используется до 100 км.)
        """
        h = Rocket.altitude()
        if h <= 100000:
            return Constants.P0 * np.exp(-Constants.M_AIR * Rocket.gravity() * h /
                                         (Constants.R_UNIVERSAL * Rocket.temperature()))
        return 0

    @staticmethod
    def density():
        """Возвращает плотность воздуха [кг/м³] на текущей высоте."""
        return Rocket.pressure_at_altitude() / (Constants.R_SPECIFIC * Rocket.temperature())

    @staticmethod
    def orbital_velocity():
        """
        Возвращает вектор орбитальной скорости [м/с] в текущем положении,
        предполагая круговое движение из-за вращения Земли.
        """
        angle = VectorMath.find_angle(Rocket.position)
        r = VectorMath.length(Rocket.position)
        return [-np.cos(angle) * Constants.EARTH_ANG_VEL * r,
                0,
                np.sin(angle) * Constants.EARTH_ANG_VEL * r]

    @staticmethod
    def relative_speed():
        """
        Возвращает скорость [м/с] ракеты относительно локальной орбитальной скорости.
        """
        orb_vel = Rocket.orbital_velocity()
        rel_vel = [Rocket.velocity[i] - orb_vel[i] for i in range(3)]
        return VectorMath.length(rel_vel)

    @staticmethod
    def total_mass():
        """
        Вычисляет общую массу [кг] аппарата.
          - Масса на стадии 0 включает: топливо первой ступени + топливо второй ступени + сухие массы обеих ступеней + полезная нагрузка.
          - После отсоединения (стадия 1) сухая масса первой ступени сбрасывается.
        """
        if Rocket.stage == 0:
            return (Rocket.fuel_first_stage + Rocket.fuel_second_stage +
                    Constants.DRY_MASS_FIRST_STAGE + Constants.DRY_MASS_SECOND_STAGE +
                    Constants.OTHER_MASS)
        elif Rocket.stage == 1:
            return (Rocket.fuel_second_stage +
                    Constants.DRY_MASS_SECOND_STAGE + Constants.OTHER_MASS)
        return Constants.OTHER_MASS

    @staticmethod
    def effective_I():
        """
        Возвращает эффективный удельный импульс [с], интерполируя между вакуумными
        и уровнем моря значениями в зависимости от окружающего давления.
        """
        pa = Rocket.pressure_at_altitude()
        if Rocket.stage == 0:
            return Constants.I0_FIRST_STAGE - (Constants.I0_FIRST_STAGE - Constants.I1_FIRST_STAGE) * pa / Constants.P0
        elif Rocket.stage == 1:
            return Constants.I0_SECOND_STAGE - (Constants.I0_SECOND_STAGE - Constants.I1_SECOND_STAGE) * pa / Constants.P0
        return 0

    @staticmethod
    def fuel_consumption_rate():
        """
        Вычисляет скорость расхода топлива [кг/с] с учётом номинального значения, скорректированного по удельному импульсу.
        """
        if Rocket.stage == 0:
            return Constants.Q_FIRST_STAGE * Constants.I0_FIRST_STAGE / Rocket.effective_I()
        elif Rocket.stage == 1:
            return Constants.Q_SECOND_STAGE * Constants.I0_SECOND_STAGE / Rocket.effective_I()
        return 0

    @staticmethod
    def thrust():
        """
        Вычисляет тягу двигателя [Н] в зависимости от окружающего давления.
        Для стадии 0 (первая ступень Soyuz) тяга интерполируется между значениями для уровня моря и вакуума.
        Для стадии 1 (Fregat) атмосферные эффекты не учитываются.
        """
        pa = Rocket.pressure_at_altitude()
        if Rocket.stage == 0:
            return Constants.F0_FIRST_STAGE - (Constants.F0_FIRST_STAGE - Constants.F1_FIRST_STAGE) * pa / Constants.P0
        elif Rocket.stage == 1:
            return Constants.F0_SECOND_STAGE - (Constants.F0_SECOND_STAGE - Constants.F1_SECOND_STAGE) * pa / Constants.P0
        return 0


# =============================================================================
# Симуляция (Обновление математической модели)
# =============================================================================
class Simulation:
    """
    Обновляет физику ракеты на каждом шаге симуляции.
    Включает работу двигателей (тяга), гравитацию, аэродинамическое сопротивление и отсоединение ступеней.
    """
    @staticmethod
    def fixed_update():
        # Увеличиваем время симуляции
        Rocket.time += Constants.TIME_STEP
        old_velocity = Rocket.velocity.copy()
        
        # --- Двигательная тяга ---
        # Вычисляем направление тяги: поворачиваем вектор положения на (90 - steering) градусов
        thrust_direction = VectorMath.rotate_vector(Rocket.position, 90 - Rocket.steering)
        acceleration = Rocket.thrust() * Rocket.throttle / Rocket.total_mass()
        Rocket.velocity = [Rocket.velocity[i] + thrust_direction[i] * acceleration * Constants.TIME_STEP
                           for i in range(3)]
        
        # --- Гравитация ---
        pos_norm = VectorMath.length(Rocket.position)
        gravity_vector = [Rocket.position[i] * Rocket.gravity() / pos_norm for i in range(3)]
        Rocket.velocity = [Rocket.velocity[i] - gravity_vector[i] * Constants.TIME_STEP for i in range(3)]
        
        # --- Аэродинамическое сопротивление ---
        speed = VectorMath.length(Rocket.velocity)
        if speed:
            drag_acc = (0.5 * Rocket.density() * speed**2 * Constants.C_D * Constants.A_EFF
                        / Rocket.total_mass())
            Rocket.velocity = [Rocket.velocity[i] - (Rocket.velocity[i] * drag_acc * Constants.TIME_STEP / speed)
                               for i in range(3)]
        
        # --- Обработка столкновения с землёй ---
        if Rocket.altitude() <= 0:
            proj = VectorMath.project_vector_onto_vector(Rocket.velocity, Rocket.position)
            if proj[0] != 0:
                Rocket.velocity = VectorMath.project_vector_onto_plane(Rocket.velocity, Rocket.position)
        
        # --- Обновление положения (метод трапеций) ---
        Rocket.position = [Rocket.position[i] + (Rocket.velocity[i] + old_velocity[i]) * Constants.TIME_STEP / 2
                           for i in range(3)]
        if Rocket.altitude() < 0:
            norm = VectorMath.length(Rocket.position)
            Rocket.position = [Rocket.position[i] * Constants.EARTH_RADIUS / norm for i in range(3)]
        
        # --- Расход топлива и отсоединение ступеней ---
        consumption = Rocket.fuel_consumption_rate() * Rocket.throttle * Constants.TIME_STEP
        if Rocket.stage == 0:
            Rocket.fuel_first_stage -= consumption
            if Rocket.fuel_first_stage < 0:
                Rocket.fuel_first_stage = 0
                # Отсоединение ступени: сбрасываем сухую массу первой ступени
                Rocket.stage = 1
                print(f"Отсоединение ступени в момент {Rocket.time:.2f} с, высота {Rocket.altitude():.2f} м")
        elif Rocket.stage == 1:
            Rocket.fuel_second_stage -= consumption
            if Rocket.fuel_second_stage < 0:
                Rocket.fuel_second_stage = 0


# =============================================================================
# Основной цикл симуляции и запись данных
# =============================================================================
if __name__ == '__main__':
    # Фиксированные управляющие установки для данной математической модели.
    Rocket.throttle = 1.0
    Rocket.steering = 90.0

    # Создаём папку "data", если она не существует.
    os.makedirs("data", exist_ok=True)

    # Формируем заголовок для лог-файла: время, общая масса, высота, относительная скорость.
    data_log = "time, mass, altitude, speed\n"
    iteration_counter = 0
    total_iterations = 7000  # Измените при необходимости длительность симуляции

    for i in range(total_iterations):
        Simulation.fixed_update()
        iteration_counter += 1

        # Записываем данные каждые 50 шагов.
        if iteration_counter == 50:
            log_line = (f"{Rocket.time:.2f}, {Rocket.total_mass():.2f}, "
                        f"{Rocket.altitude():.2f}, {Rocket.relative_speed():.2f}\n")
            data_log += log_line
            iteration_counter = 0

    # Записываем лог в файл "data/MathModel_Stats.txt"
    file_path = os.path.join("data", "MathModel_Stats.txt")
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(data_log)

    print(f"Данные сохранены в {file_path}")
