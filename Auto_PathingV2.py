#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


async def run():
    # Utwórz instancję drona i połącz się (poprawiona składnia)
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

    # Pobierz aktualną pozycję jako punkt startowy
    async for position in drone.telemetry.position():
        start_lat = position.latitude_deg
        start_lon = position.longitude_deg
        print(f"Pozycja startowa: {start_lat}, {start_lon}")
        break

    # Zdefiniuj trasę (waypoints) z dodanym vehicle_action
    mission_items = []

    # Waypoint 1: Start (pozycja startowa + 10m wysokości)
    mission_items.append(MissionItem(
        start_lat,  # szerokość geograficzna
        start_lon,  # długość geograficzna
        10,  # wysokość (metry)
        10,  # prędkość (m/s)
        True,  # is_fly_through
        float('nan'),  # gimbal pitch
        float('nan'),  # gimbal yaw
        MissionItem.CameraAction.NONE,
        float('nan'),  # loiter_time_s
        float('nan'),  # camera_photo_interval_s
        float('nan'),  # acceptance_radius_m
        float('nan'),  # yaw_deg
        float('nan'),  # camera_photo_distance_m
        MissionItem.VehicleAction.NONE  # DODANY PARAMETR!
    ))

    # Waypoint 2: 50m na północ
    mission_items.append(MissionItem(
        start_lat + 0.00045,  # ~50m na północ
        start_lon,
        10,
        10,
        True,
        float('nan'),
        float('nan'),
        MissionItem.CameraAction.NONE,
        float('nan'),
        float('nan'),
        float('nan'),
        float('nan'),
        float('nan'),
        MissionItem.VehicleAction.NONE
    ))

    # Waypoint 3: 50m na wschód
    mission_items.append(MissionItem(
        start_lat + 0.00045,
        start_lon + 0.00075,  # ~50m na wschód
        10,
        10,
        True,
        float('nan'),
        float('nan'),
        MissionItem.CameraAction.NONE,
        float('nan'),
        float('nan'),
        float('nan'),
        float('nan'),
        float('nan'),
        MissionItem.VehicleAction.NONE
    ))

    # Waypoint 4: Powrót do początku
    mission_items.append(MissionItem(
        start_lat,
        start_lon,
        10,
        10,
        True,
        float('nan'),
        float('nan'),
        MissionItem.CameraAction.NONE,
        float('nan'),
        float('nan'),
        float('nan'),
        float('nan'),
        float('nan'),
        MissionItem.VehicleAction.NONE
    ))

    # Załaduj misję
    mission_plan = MissionPlan(mission_items)
    await drone.mission.upload_mission(mission_plan)
    print("Misja załadowana!")

    # Uzbrój drona
    print("Uzbrajanie...")
    await drone.action.arm()

    # Wystartuj misję
    print("Rozpoczynam misję...")
    await drone.mission.start_mission()

    # Monitoruj postęp misji
    async for mission_progress in drone.mission.mission_progress():
        print(f"Postęp misji: {mission_progress.current}/{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            print("Misja zakończona!")
            break

    # Lądowanie
    print("Lądowanie...")
    await drone.action.land()

    # Czekaj na lądowanie
    async for is_in_air in drone.telemetry.in_air():
        if not is_in_air:
            print("Dron wylądował!")
            break

    print("Misja zakończona pomyślnie!")


if __name__ == "__main__":
    asyncio.run(run())