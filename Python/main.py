import krpc
import time
import _thread as thread
import math
import msgpack
from time import sleep


def monitor(vessel):
    """
    Мониторинг топлива на активном этапе корабля.
    Если топлива нет, активирует следующую стадию.

    Args:
        vessel (krpc.client.services.spacecenter.Vessel): Корабль, за которым ведется наблюдение.
    """

    sleep(3)
    while True:
        # Получаем информацию о ресурсах на текущем этапе
        resources = vessel.resources_in_decouple_stage(vessel.control.current_stage - 1, False)

        solidFuel = resources.amount("SolidFuel")
        liquidFuel = resources.amount("LiquidFuel")
        print(liquidFuel)

        if abs(liquidFuel) <= 0.1:
            vessel.control.activate_next_stage()
            print()
            print("Отделение ступени!")
            print()

        sleep(0.2)  # Проверяем состояние топлива каждые 0.2 секунды


def go_to_orbit(vessel, space_center, connection, ascentProfileConstant=1.25):
    """
    Поднимает корабль на орбиту 75 x 70 км, управляя взлетом и гравитационным поворотом.

    Args:
        vessel (krpc.client.services.spacecenter.Vessel): Корабль, который поднимается на орбиту.
        space_center (krpc.client.services.spacecenter.SpaceCenter): Объект Space Center API.
        connection (krpc.client.Connection): Подключение к серверу kRPC.
        ascentProfileConstant (float): Коэффициент, определяющий скорость гравитационного поворота.
    """
    vessel.control.rcs = True
    vessel.control.throttle = 1

    apoapsisStream = connection.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_heading = 90

    # Достигаем нужного апоцентра/завершаем гравитационный поворот
    while apoapsisStream() < 75000:
        targetPitch = 90 - ((90 / (75000 ** ascentProfileConstant)) * (apoapsisStream() ** ascentProfileConstant))
        print("Current target pitch:", targetPitch, "with apoapsis", apoapsisStream())

        # Устанавливаем автопилот
        vessel.auto_pilot.target_pitch = targetPitch

        sleep(0.1)

    vessel.control.throttle = 0

    timeToApoapsisStream = connection.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    periapsisStream = connection.add_stream(getattr, vessel.orbit, 'periapsis_altitude')

    # Ждем, пока не подойдет момент для циркуляризации орбиты
    while (timeToApoapsisStream() > 22):
        sleep(0.5)

    vessel.control.throttle = 0.5
    lastUT = space_center.ut
    lastTimeToAp = timeToApoapsisStream()

    # Циркуляризируем орбиту
    while (periapsisStream() < 70500):
        sleep(0.2)
        timeToAp = timeToApoapsisStream()
        UT = space_center.ut
        deltaTimeToAp = (timeToAp - lastTimeToAp) / (space_center.ut - lastUT)

        print("Ожидаемое изменение времени до апоцентра в секунду:", deltaTimeToAp)

        if deltaTimeToAp < -0.3:
            vessel.control.throttle += 0.03
        elif deltaTimeToAp < -0.1:
            vessel.control.throttle += 0.01

        if deltaTimeToAp > 0.2:
            vessel.control.throttle -= 0.03
        elif deltaTimeToAp > 0:
            vessel.control.throttle -= 0.01

        lastTimeToAp = timeToApoapsisStream()
        lastUT = space_center.ut

    vessel.control.throttle = 0
    print("Апоцентр: ", apoapsisStream())
    print("Перицентр: ", periapsisStream())
    print("Орбита достигнута!")
    print()


def mun_transfer(vessel, space_center, connection):
    """
    Осуществляет переход на орбиту Муны.

    Args:
        vessel (krpc.client.services.spacecenter.Vessel): Корабль, выполняющий маневр.
        space_center (krpc.client.services.spacecenter.SpaceCenter): Объект Space Center API.
        connection (krpc.client.Connection): Подключение к серверу kRPC.
    """

    vessel.control.rcs = True

    for fairing in vessel.parts.fairings:
        fairing.jettison()

    vessel.control.antennas = True  # Разворачиваем антенны

    # Вычисляем оптимальный фазовый угол для запуска к Мунe
    destSemiMajor = space_center.bodies["Mun"].orbit.semi_major_axis
    hohmannSemiMajor = destSemiMajor / 2
    neededPhase = 2 * math.pi * (1 / (2 * (destSemiMajor ** 3 / hohmannSemiMajor ** 3) ** (1 / 2)))
    optimalPhaseAngle = 180 - neededPhase * 180 / math.pi  # В градусах

    # Ожидаем подходящего момента
    phaseAngle = 1080
    vessel.auto_pilot.engage()
    vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
    vessel.auto_pilot.target_direction = (0.0, 1.0, 0.0)

    angleDec = False  # Уменьшается ли угол фазы; используется для проверки, что Муна впереди корабля
    prevPhase = 0
    while abs(phaseAngle - optimalPhaseAngle) > 1 or not angleDec:
        bodyRadius = space_center.bodies["Mun"].orbit.radius
        vesselRadius = vessel.orbit.radius

        sleep(1)

        bodyPos = space_center.bodies["Mun"].orbit.position_at(space_center.ut,
                                                               space_center.bodies["Mun"].reference_frame)
        vesselPos = vessel.orbit.position_at(space_center.ut, space_center.bodies["Mun"].reference_frame)

        bodyVesselDistance = ((bodyPos[0] - vesselPos[0]) ** 2 + (bodyPos[1] - vesselPos[1]) ** 2 + (
                    bodyPos[2] - vesselPos[2]) ** 2) ** (1 / 2)

        try:
            phaseAngle = math.acos(
                (bodyRadius ** 2 + vesselRadius ** 2 - bodyVesselDistance ** 2) / (2 * bodyRadius * vesselRadius))
        except:
            print("Ошибка области! Невозможно вычислить. Ожидайте...")
            continue  # Ошибка области
        phaseAngle = phaseAngle * 180 / math.pi

        if prevPhase - phaseAngle > 0:
            angleDec = True
            if abs(phaseAngle - optimalPhaseAngle) > 20:
                space_center.rails_warp_factor = 2
            else:
                space_center.rails_warp_factor = 0
        else:
            angleDec = False
            space_center.rails_warp_factor = 4

        prevPhase = phaseAngle

        print("Phase:", phaseAngle)

    # Рассчитываем необходимое DeltaV
    GM = vessel.orbit.body.gravitational_parameter  # Получаем параметр гравитации (GM) для Кербина
    r = vessel.orbit.radius
    a = vessel.orbit.semi_major_axis

    initialV = (GM * ((2 / r) - (1 / a))) ** (1 / 2)

    a = (space_center.bodies["Mun"].orbit.radius + vessel.orbit.radius) / 2

    finalV = (GM * ((2 / r) - (1 / a))) ** (1 / 2)

    deltaV = finalV - initialV
    print("Маневр сейчас с DeltaV:", deltaV)

    actualDeltaV = 0
    vessel.control.throttle = 1.0
    while (deltaV > actualDeltaV):  # Завершаем маневр с отклонением <= 2%
        sleep(0.5)
        r = vessel.orbit.radius
        a = vessel.orbit.semi_major_axis
        actualDeltaV = (GM * ((2 / r) - (1 / a))) ** (1 / 2) - initialV
        print("DeltaV на данный момент: ", actualDeltaV, "из нужных", deltaV)
    vessel.control.throttle = 0
    vessel.auto_pilot.disengage()

    print("Переход на Муну завершен")


def go_to_landing(vessel, space_center, connection):
    """
    Подготавливает корабль к посадке на поверхность Муны.

    Args:
        vessel (krpc.client.services.spacecenter.Vessel): Корабль, выполняющий посадку.
        space_center (krpc.client.services.spacecenter.SpaceCenter): Объект Space Center API.
        connection (krpc.client.Connection): Подключение к серверу kRPC.
    """

    vessel.control.rcs = True
    vessel.control.antennas = True
    vessel.auto_pilot.engage()
    vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
    vessel.auto_pilot.target_direction = (0.0, -1.0, 0.0)  # Указываем в сторону ретрограда (против движения)

    vessel.auto_pilot.wait()  # Ждем, пока автопилот скорректирует направление
    time_to_warp = vessel.orbit.time_to_periapsis
    space_center.warp_to(space_center.ut + time_to_warp - 30)  # Дожидаемся 30 секунд до перицентра

    vessel.auto_pilot.wait()  # Ждем, пока корабль не зафиксирует направление
    print("Запуск двигателей. Начало торможения")  # Начало торможения

    flight = vessel.flight(vessel.orbit.body.reference_frame)
    surfaceSpeed = connection.add_stream(getattr, flight, 'speed')  # Отслеживание скорости

    # Начинаем гашение скорости
    while surfaceSpeed() > 1.0:
        vessel.control.throttle = 1 - (0.95 / 1.01 ** surfaceSpeed())

        # Проверяем ошибку направления автопилота
        error = (vessel.auto_pilot.pitch_error ** 2 + vessel.auto_pilot.heading_error ** 2) ** (1 / 2)
        if error > 3:
            vessel.control.throttle = 0
            while error > 1.2:
                error = (vessel.auto_pilot.pitch_error ** 2 + vessel.auto_pilot.heading_error ** 2) ** (1 / 2)
                print("Ошибка направления:", vessel.auto_pilot.error)
                sleep(0.25)

    vessel.control.throttle = 0
    print("Подготовка к посадке...")
    print()


def begin_landing(vessel, space_center, connection):
    """
    Осуществляет финальную посадку корабля на поверхность Муны.

    Args:
        vessel (krpc.client.services.spacecenter.Vessel): Корабль, выполняющий посадку.
        space_center (krpc.client.services.spacecenter.SpaceCenter): Объект Space Center API.
        connection (krpc.client.Connection): Подключение к серверу kRPC.
    """

    deployed = False

    # Получаем текущий объект
    current_body = vessel.orbit.body
    landing_reference_frame = space_center.ReferenceFrame.create_hybrid(
        position=current_body.reference_frame, rotation=vessel.surface_reference_frame)
    flight = vessel.flight(landing_reference_frame)

    while True:
        # Определяем, сколько времени потребуется для полной остановки
        time = velocity_intercept(vessel, -flight.velocity[0])
        height = height_intercept(vessel, time, -flight.velocity[0], flight.surface_altitude)
        print("Прогнозируемая высота:", height)

        # Развертываем посадочные опоры, если приближаемся к поверхности
        if height < 1000 and time < 9 and not deployed:
            deployed = True
            vessel.control.legs = True

        if height < 8:  # Если прогнозируемая высота меньше 8 метров, завершаем спуск
            break

    # Запуск двигателя на полной тяге
    print("ЗАПУСК ДВИГАТЕЛЯ")
    vessel.control.throttle = 1
    sleep(0.1)

    # Точное торможение перед посадкой
    while abs(flight.velocity[0]) > 1:

        if flight.surface_altitude < 30:
            print("Отключение автопилота для окончательной посадки...")
            vessel.auto_pilot.disengage()

        time = velocity_intercept(vessel, -flight.velocity[0], 0.01, vessel.control.throttle)
        height = height_intercept(vessel, time, -flight.velocity[0], flight.surface_altitude, vessel.control.throttle)

        # Коррекция тяги для мягкой посадки
        if height > 3.5:
            vessel.control.throttle -= 0.005
        elif height < 0.5:
            vessel.control.throttle += 0.004

        if time < 9 and not deployed:
            print("Разворачиваем посадочные ноги...")
            deployed = True
            vessel.control.legs = True

    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)  # Пытаемся сделать ракету вертикальной
    vessel.control.throttle = 0
    print()
    print("Конечная высота:", flight.surface_altitude)
    print("Посадка!")


def velocity_intercept(vessel, initial_velocity, tolerance=0.01, thrust_multiplier=1):
    """
    Рассчитывает, сколько времени потребуется, чтобы изменить скорость до нуля при торможении двигателями.

    Использует бинарный поиск по уравнению скорости, чтобы определить, когда скорость упадет до нуля.
    Предполагает, что тяга направлена противоположно движению.

    Args:
        vessel (krpc.client.services.spacecenter.Vessel): Корабль, для которого вычисляется интерсепт скорости.
        initial_velocity (float): Начальная скорость корабля.
        tolerance (float, optional): Допустимое отклонение от нуля при расчете времени. По умолчанию 0.01.
        thrust_multiplier (float, optional): Коэффициент для изменения тяги. По умолчанию 1.

    Returns:
        float: Время, через которое скорость упадет до нуля.
    """

    current_body = vessel.orbit.body
    if initial_velocity > 0:
        initial_velocity *= -1  # Меняем знак скорости для корректного вычисления

    time = 10  # Начальное предположение времени торможения
    thrust = thrust_multiplier * 0.99 * (vessel.max_vacuum_thrust / 1000)  # Приведение тяги к тонн-секундам

    # Корректируем направление тяги в зависимости от ориентации корабля
    direction = vessel.flight(vessel.surface_reference_frame).direction
    multiplier = direction[0]
    thrust = thrust * abs(multiplier)

    gravity_accel = current_body.surface_gravity  # Гравитационное ускорение
    mass = vessel.mass / 1000  # Масса корабля в тоннах
    mass_burn_rate = approximate_mass_burn_rate(vessel)  # Скорость расхода массы

    velocity = 1  # Инициализация скорости для начала бинарного поиска

    # Границы бинарного поиска
    upper_bound = 92
    lower_bound = 0
    num_iterations = 0  # Счетчик итераций

    # Бинарный поиск времени торможения
    while abs(velocity) > tolerance and time > 0.0001 and time < 91.99:
        velocity = (-thrust / mass_burn_rate) * math.log(abs(mass - mass_burn_rate * time)) \
                   - gravity_accel * time + initial_velocity + (thrust / mass_burn_rate) * math.log(mass)

        num_iterations += 1

        if velocity < 0:
            lower_bound = time
            time = (time + upper_bound) / 2
        elif velocity > 0:
            upper_bound = time
            time = (time + lower_bound) / 2

    if num_iterations == 0:
        print(velocity)
        print(time)
        print(thrust_multiplier)
        print()
    return time


def height_intercept(vessel, time, initial_velocity, current_height, thrust_multiplier=1):
    """
    Рассчитывает предполагаемую высоту корабля после выполнения тормозного маневра.

    Args:
        vessel (krpc.client.services.spacecenter.Vessel): Корабль, для которого вычисляется интерсепт высоты.
        time (float): Время разгона/торможения.
        initial_velocity (float): Начальная скорость корабля.
        current_height (float): Текущая высота корабля.
        thrust_multiplier (float, optional): Коэффициент для изменения тяги. По умолчанию 1.

    Returns:
        float: Ожидаемая конечная высота корабля.
    """

    current_body = vessel.orbit.body

    thrust = thrust_multiplier * 0.99 * (vessel.max_vacuum_thrust / 1000)  # Приведение тяги к тонн-секундам

    # Корректировка направления тяги в зависимости от ориентации корабля
    direction = vessel.flight(vessel.surface_reference_frame).direction
    multiplier = direction[0]
    thrust = thrust * abs(multiplier)

    gravity_accel = current_body.surface_gravity
    mass = vessel.mass / 1000
    mass_burn_rate = approximate_mass_burn_rate(vessel)

    if initial_velocity > 0:
        initial_velocity *= -1

    final_height = (-1 / mass_burn_rate ** 2) * (
            mass_burn_rate * initial_velocity * (mass - mass_burn_rate * time) + thrust * math.log(mass) * (
                mass - mass_burn_rate * time)
            + gravity_accel * (((mass - mass_burn_rate * time) ** 2) / 2 - mass * (mass - mass_burn_rate * time))
            - thrust * (mass * math.log(mass - mass_burn_rate * time) - mass_burn_rate * time * math.log(
        mass - mass_burn_rate * time) - mass + mass_burn_rate * time)
    ) + current_height + (1 / mass_burn_rate ** 2) * (
                           mass_burn_rate * initial_velocity * mass + thrust * math.log(mass) * mass
                           + gravity_accel * ((mass ** 2) / 2 - mass ** 2)
                           - thrust * (mass * math.log(mass) - mass)
                   )

    return final_height


def velocity_function(vessel, initial_velocity, time, thrust):
    """
    Рассчитывает скорость корабля в определенный момент времени при заданной тяге.

    Args:
        vessel (krpc.client.services.spacecenter.Vessel): Корабль.
        initial_velocity (float): Начальная скорость корабля.
        time (float): Время, через которое определяется скорость.
        thrust (float): Значение тяги.

    Returns:
        float: Скорость корабля через указанное время.
    """

    current_body = vessel.orbit.body

    if initial_velocity > 0:
        initial_velocity *= -1

    gravity_accel = current_body.surface_gravity
    mass = vessel.mass / 1000
    mass_burn_rate = approximate_mass_burn_rate(vessel)

    try:
        velocity = (-thrust / mass_burn_rate) * math.log(mass - mass_burn_rate * time) \
                   - gravity_accel * time + initial_velocity + (thrust / mass_burn_rate) * math.log(mass)
    except:
        velocity = 0  # В случае ошибки возвращаем нулевую скорость

    return velocity


def height_function(vessel, time, initial_velocity, current_height, thrust):
    """
    Рассчитывает высоту корабля через определенное время при заданной тяге.

    Args:
        vessel (krpc.client.services.spacecenter.Vessel): Корабль.
        time (float): Время, через которое определяется высота.
        initial_velocity (float): Начальная скорость корабля.
        current_height (float): Текущая высота корабля.
        thrust (float): Значение тяги.

    Returns:
        float: Высота корабля через указанное время.
    """

    current_body = vessel.orbit.body

    gravity_accel = current_body.surface_gravity
    mass = vessel.mass / 1000
    mass_burn_rate = approximate_mass_burn_rate(vessel)

    if initial_velocity > 0:
        initial_velocity *= -1

    final_height = (-1 / mass_burn_rate ** 2) * (
            mass_burn_rate * initial_velocity * (mass - mass_burn_rate * time) + thrust * math.log(mass) * (
            mass - mass_burn_rate * time)
            + gravity_accel * (((mass - mass_burn_rate * time) ** 2) / 2 - mass * (mass - mass_burn_rate * time))
            - thrust * (mass * math.log(mass - mass_burn_rate * time) - mass_burn_rate * time * math.log(
        mass - mass_burn_rate * time) - mass + mass_burn_rate * time)
    ) + current_height + (1 / mass_burn_rate ** 2) * (
                           mass_burn_rate * initial_velocity * mass + thrust * math.log(mass) * mass
                           + gravity_accel * ((mass ** 2) / 2 - mass ** 2)
                           - thrust * (mass * math.log(mass) - mass)
                   )
    return final_height


def approximate_mass_burn_rate(vessel):
    mass_burn_rate = 0

    engines = vessel.parts.engines
    for engine in engines:
        if engine.active or engine.available_thrust > 0:
            if engine.part.name == "liquidEngine":
                mass_burn_rate += 0.078926
            elif engine.part.name == "liquidEngine3.v2" or engine.part.name == "liquidEngine3":
                mass_burn_rate += 0.017734
            elif engine.part.name == "liquidEngine2":
                mass_burn_rate += 0.068512
    return mass_burn_rate


# Устанавливаем соединение с сервером kRPC
connection = krpc.connect("Connection")
space_center = connection.space_center
vessel = space_center.active_vessel

# Запускаем поток мониторинга топлива
args = [vessel]
thread.start_new_thread(monitor, tuple(args))

"""Полностью автоматизированный полет на Муну"""

# 1. Достигаем низкой орбиты Кербина
go_to_orbit(vessel, space_center, connection, 0.5)

# 2. Маневр перевода на орбиту Муны
mun_transfer(vessel, space_center, connection)
time_to_warp = vessel.orbit.next_orbit.time_to_periapsis + vessel.orbit.time_to_soi_change
space_center.warp_to(space_center.ut + time_to_warp - 300)  # Ускоряем время на 5 минут до перицентра у Муны

# 3. Подготовка к посадке
go_to_landing(vessel, space_center, connection)

# 4. Финальная посадка
vessel.auto_pilot.engage()
vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
vessel.auto_pilot.target_direction = (0.0, -1.0, 0.0)  # Указываем ретроградно к поверхности
begin_landing(vessel, space_center, connection)

# 5. Стабилизация после посадки
print("Стабилизация...")
vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
vessel.auto_pilot.target_direction = (1.0, 0.0, 0.0)
sleep(10)
vessel.auto_pilot.disengage()
vessel.control.sas = True  # Включаем систему стабилизации
print("ГОТОВО")
