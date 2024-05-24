#!/usr/bin/env python3

import asyncio
import numpy as np
import math
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from roboflowoak import RoboflowOak
import cv2
import time

# Define constants
MISSION = 4
STABILIZE = 13
MANUAL = 9
RTL = 5
LAND = 6
HOLD = 3

# Function to determine the pixel per inch ratio at a given distance
def pixel_to_real_world_distance(altitude):
    return 512.06 * (altitude ** -0.927)

# Function to calculate coordinates of the detected target
def get_coordinates(plane_latitude, plane_longitude, plane_bearing, altitude, target_x, target_y):
    earth_radius = 6371  # Earth's radius in kilometers
    frame_center_x = 320
    frame_center_y = 320

    pixel_distance_conversion = 1 / pixel_to_real_world_distance(altitude) * 0.0254
    real_y_difference = pixel_distance_conversion * (target_y - frame_center_y)
    real_x_difference = pixel_distance_conversion * (target_x - frame_center_x)

    square_theta = np.arctan2(real_y_difference, real_x_difference)
    square_theta -= np.pi / 2
    square_theta -= np.deg2rad(plane_bearing)

    if square_theta < 0:
        square_theta += 2 * np.pi

    distance = np.sqrt(real_x_difference**2 + real_y_difference**2) / 1000

    target_longitude, target_latitude = great_circle_target(
        np.deg2rad(plane_longitude), np.deg2rad(plane_latitude), square_theta, distance, earth_radius
    )

    return np.rad2deg(target_longitude), np.rad2deg(target_latitude)

# Great circle formula to calculate destination coordinates
def great_circle_target(lon1, lat1, bearing, dist, R):
    lat2 = math.asin(math.sin(lat1) * math.cos(dist / R) +
                     math.cos(lat1) * math.sin(dist / R) * math.cos(bearing))
    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(dist / R) * math.cos(lat1),
                             math.cos(dist / R) - math.sin(lat1) * math.sin(lat2))
    return lon2, lat2

async def update_telemetry(drone):
    global plane_lat, plane_lon, plane_bearing, plane_relative_alt
    async for position in drone.telemetry.position():
        plane_lat = position.latitude_deg
        plane_lon = position.longitude_deg
    async for heading in drone.telemetry.heading():
        plane_bearing = heading.heading_deg
    async for altitude in drone.telemetry.relative_altitude():
        plane_relative_alt = altitude.relative_altitude_m * 39.3701  # Convert meters to inches

async def process_vision(rf, output_file):
    while True:
        t0 = time.time()
        result, frame, raw_frame, depth = rf.detect()
        predictions = result["predictions"]

        t = time.time() - t0
        inference_time = 1 / t
        print("INFERENCE TIME IN MS ", inference_time)

        for p in predictions:
            x = p.json()['x']
            y = p.json()['y']
            width = p.json()['width']
            height = p.json()['height']
            print("Center X:", x, "Center Y:", y, "Width:", width, "Height:", height)

            target_lon, target_lat = get_coordinates(
                plane_lat, plane_lon, plane_bearing, plane_relative_alt, x, y
            )

            if p.json()['class'] == "RedSquare":
                with open(output_file, "a") as file:
                    file.write(f"Target lat: {target_lat}, Target long: {target_lon}, Plane lat: {plane_lat}, Plane lon: {plane_lon}, plane alt: {plane_relative_alt}, plane bearing: {plane_bearing}, pixel coord x: {x}, pixel coord y: {y}\n")
        
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) == ord('q'):
            break

async def run():
    # Connect to the drone
    drone = System(mavsdk_server_address='localhost', port=50051)
    await drone.connect()

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    async for position in drone.telemetry.position():
        break
    await drone.action.set_takeoff_altitude(2)
    start_lat, start_lon = position.latitude_deg, position.longitude_deg
    await drone.mission.clear_mission()
    await drone.mission.set_return_to_launch_after_mission(False)
    mission_items = [
        MissionItem(35.328972, -120.752941, 2, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE),
        MissionItem(35.328000, -120.751989, 2, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE),
        MissionItem(35.328972, -120.752941, 2, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE),
        MissionItem(start_lat, start_lon, 2, 10, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE)
    ]

    mission_plan = MissionPlan(mission_items)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await asyncio.sleep(30)
    await drone.action.arm()
    
    # await drone.action.takeoff()
    # await asyncio.sleep(5)
    # await drone.mission.start_mission()
    
    await monitor_flight_mode(drone)
    await termination_task

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")

async def monitor_flight_mode(drone):
    previous_flight_mode = None
    flag = False

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            print(f"Flight mode changed to: {flight_mode}")
            if flight_mode.value == STABILIZE or flight_mode.value == MANUAL:
                print("Flight mode changed from MISSION to MANUAL. Pausing mission...")
                await drone.mission.pause_mission()
            elif flight_mode.value == HOLD:
                print("Switched to hold mode")
                await drone.action.hold()
            else:
                await drone.mission.start_mission()
            previous_flight_mode = flight_mode
        async for mission_progress in drone.mission.mission_progress():
            if mission_progress.current == mission_progress.total:
                await drone.action.land()
                flag = True
            break
        if flag:
            break

async def observe_is_in_air(drone, running_tasks):
    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            return

if __name__ == '__main__':
    # Run the asyncio loop
    asyncio.run(run())

    # Output file for detected coordinates
    output_file = "output.txt"

    # Initialize RoboflowOak
    rf = RoboflowOak(model="redsquare-gwdyn", confidence=0.50,
                     overlap=0.5, version="1",
                     api_key="N5Xs9o02pFDsaVc5pjcd", rgb=True, depth=False,
                     device=None, device_name="NGCP", blocking=True)

    # Start telemetry update and vision processing tasks
    asyncio.run(update_telemetry(drone))
    asyncio.run(process_vision(rf, output_file))
