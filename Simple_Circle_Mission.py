#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Oczekiwanie na połączenie z dronem...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Dron połączony!")
            break

    print("Oczekiwanie na GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS gotowy!")
            break

    print("Uzbrajanie...")
    await drone.action.arm()

    print("Start na 15m wysokości...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print("Ustawiam wysokość 15m...")
    await drone.action.set_takeoff_altitude(30)
    await asyncio.sleep(3)

    print("Rozpoczynam lot po okręgu (promień 30m, prędkość 5m/s)...")
    # Parametry orbit:
    # - radius_m: promień okręgu w metrach
    # - velocity_ms: prędkość liniowa w m/s
    # - yaw_behavior: HOLD_FRONT_TO_CIRCLE_CENTER - dron zawsze skierowany do środka
    # - latitude_deg, longitude_deg: środek okręgu (None = aktualna pozycja)
    # - absolute_altitude_m: wysokość (None = aktualna wysokość)

    from mavsdk.action import OrbitYawBehavior

    await drone.action.do_orbit(
        radius_m=30.0,
        velocity_ms=5.0,
        yaw_behavior=OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER,
        latitude_deg=float('nan'),  # użyj aktualnej pozycji jako środek
        longitude_deg=float('nan'),
        absolute_altitude_m=float('nan')
    )

    print("Lecę po okręgu przez 60 sekund...")
    await asyncio.sleep(60)  # Lot po okręgu przez 1 minutę

    print("Kończę orbitę i lądowanie...")
    await drone.action.land()

    async for is_in_air in drone.telemetry.in_air():
        if not is_in_air:
            print("Dron wylądował!")
            break

    print("Misja zakończona!")


if __name__ == "__main__":
    asyncio.run(run())