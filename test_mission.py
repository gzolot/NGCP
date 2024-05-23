#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

import numpy as np
import math

from roboflowoak import RoboflowOak
import cv2
import time

# This function determines the pixel per inch ratio at a given distance. 
# We used inches as this was a convenient unit for measuring the 2x2 red square, 
# and each tile in the EE bldg is exactly 12 inches for measuring distance. 

# input altitude must be in inches
def pixel_to_real_world_distance(altitude):
    return 512.06 * (altitude ** -0.927)

# altitude: inches. This is the Relative altitude of drone from takeoff
# lat/lon must be formatted in degrees like this:
    # plane_lat = 35.2994616
    # plane_lon = -120.6630686
# plane_bearing must be in degrees measured clockwise from North
def get_coordinates(plane_latitude, plane_longitude, plane_bearing, altitude, target_x, target_y):
    
    earth_radius = 6371  # Earth's radius in kilometers

    # center of 640x640 camera frame. 
    frame_center_x = 320 
    frame_center_y = 320

    # Calculate real-world x and y distance of the square relative to the drone (center of camera frame)
    pixel_distance_conversion = 1 / pixel_to_real_world_distance(altitude) * 0.0254 # 1/ (pixel per inch) * 0.0254 = meter per pixel
    real_y_difference = pixel_distance_conversion * (target_y - frame_center_y)
    real_x_difference = pixel_distance_conversion * (target_x - frame_center_x)

    
    square_theta = np.arctan2(real_y_difference, real_x_difference) # get angle of square vector in camera frame of reference
    square_theta -= np.pi / 2  # adjust angle wrt plane nose (y-axis in camera frame of reference)
    square_theta -= np.deg2rad(plane_bearing)  # offset by plane bearing to get square bearing wrt North

    # Keep in range 0 to 2pi. This represents clockwise angle (bearing) from North
    if square_theta < 0:
        square_theta += 2 * np.pi

    distance = np.sqrt(real_x_difference**2 + real_y_difference**2) / 1000  # Convert distance to kilometers

    target_longitude, target_latitude = great_circle_target(
        np.deg2rad(plane_longitude), np.deg2rad(plane_latitude), square_theta, distance, earth_radius
    )

    return np.rad2deg(target_longitude), np.rad2deg(target_latitude)

# This function uses the great circle formula to obtain a destination coordinate 
def great_circle_target(lon1, lat1, bearing, dist, R):
    lat2 = math.asin(math.sin(lat1) * math.cos(dist / R) + 
                     math.cos(lat1) * math.sin(dist / R) * math.cos(bearing))
    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(dist / R) * math.cos(lat1),
                             math.cos(dist / R) - math.sin(lat1) * math.sin(lat2))
    return lon2, lat2


if __name__ == '__main__':

    # get this data from pixhawk:
    print("\nTesting on data gathered on 5.20.24\n")
    plane_lat =  35.2994636
    plane_lon = -120.6630624
    plane_bearing = 80.9
    plane_relative_alt =  40 * 12 # convert to inches 

    # Open the file in write mode
    with open("output.txt", "w") as file:
        # instantiating an object (rf) with the RoboflowOak module
        rf = RoboflowOak(model="redsquare-gwdyn", confidence=0.50,
                         overlap=0.5, version="1",
                         api_key="N5Xs9o02pFDsaVc5pjcd", rgb=True, depth=False,
                         device=None, device_name="NGCP", blocking=True)
        
        while True:
            t0 = time.time()
            result, frame, raw_frame, depth = rf.detect()  # verify size of capture frame is 640x640
            predictions = result["predictions"]
            
            t = time.time() - t0
            inference_time = 1 / t
            print("INFERENCE TIME IN MS ", inference_time)
            
            # Write inference time to file
            #file.write(f"INFERENCE TIME IN MS {inference_time}\n")
            
            for p in predictions:
                x = p.json()['x']
                y = p.json()['y']
                width = p.json()['width']
                height = p.json()['height']
                print("Center X:", x, "Center Y:", y, "Width:", width, "Height:", height)
                
                target_lon, target_lat = get_coordinates(
                    plane_lat, plane_lon, plane_bearing, plane_relative_alt, x, y
                )

                # Write predictions to file
                if p.json()['class'] == "RedSquare":
                    file.write(f"Center X: {x}, Center Y: {y}, Width: {width}, Height: {height}\n")
            
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) == ord('q'):
                break

# The file will be automatically closed when the with block is exited







    ######################################

    # data gathered on 5.20.24
    print("\nTesting on data gathered on 5.20.24\n")
    plane_lat =  35.2994636
    plane_lon = -120.6630624

    # Quadrant 1
    square_x2 = 593.0
    square_y2 = 152.0 
    square_lat2 = 35.2994375
    square_lon2 = -120.6630039
    
    target_lon, target_lat = get_coordinates(
        plane_lat, plane_lon, plane_bearing, plane_relative_alt, square_x2, square_y2
    )
    print(f"\nPlane coordinates: Latitude = {plane_lat}, Longitude = {plane_lon}")
    print(f"Target Coordinates as calculated: Latitude = {target_lat}, Longitude = {target_lon}")
    print(f"Target Coordinates as measured: Latitude = {square_lat2}, Longitude = {square_lon2}")

    # Quadrant 4
    square_x3 = 577.0
    square_y3 = 420.5
    square_lat3 =  35.2994310
    square_lon3 =  -120.66230195 

    target_lon, target_lat = get_coordinates(
        plane_lat, plane_lon, plane_bearing, plane_relative_alt, square_x3, square_y3
    )
    print(f"\nPlane coordinates: Latitude = {plane_lat}, Longitude = {plane_lon}")
    print(f"Target Coordinates as calculated: Latitude = {target_lat}, Longitude = {target_lon}")
    print(f"Target Coordinates as measured: Latitude = {square_lat3}, Longitude = {square_lon3}")


    square_x4 = 86.0
    square_y4 = 321.0
    square_lat4 =  352994927
    square_lon4 = -120.6630437
    
    target_lon, target_lat = get_coordinates(
        plane_lat, plane_lon, plane_bearing, plane_relative_alt, square_x4, square_y4
    )
    print(f"\nPlane coordinates: Latitude = {plane_lat}, Longitude = {plane_lon}")
    print(f"Target Coordinates as calculated: Latitude = {target_lat}, Longitude = {target_lon}")
    print(f"Target Coordinates as measured: Latitude = {square_lat4}, Longitude = {square_lon4}")


    square_x5 = 133.5
    square_y5 = 59.0
    square_lat5 =  35.2995012
    square_lon5 =  -120.6630149 

    target_lon, target_lat = get_coordinates(
        plane_lat, plane_lon, plane_bearing, plane_relative_alt, square_x5, square_y5
    )
    print(f"\nPlane coordinates: Latitude = {plane_lat}, Longitude = {plane_lon}")
    print(f"Target Coordinates as calculated: Latitude = {target_lat}, Longitude = {target_lon}")
    print(f"Target Coordinates as measured: Latitude = {square_lat5}, Longitude = {square_lon5}")

    square_x6 =  315.5
    square_y6 =  45.0 
    square_lat6 =  35.2994810
    square_lon6 =  -120.6630268

    target_lon, target_lat = get_coordinates(
        plane_lat, plane_lon, plane_bearing, plane_relative_alt, square_x6, square_y6
    )
    print(f"\nPlane coordinates: Latitude = {plane_lat}, Longitude = {plane_lon}")
    print(f"Target Coordinates as calculated: Latitude = {target_lat}, Longitude = {target_lon}")
    print(f"Target Coordinates as measured: Latitude = {square_lat6}, Longitude = {square_lon6}")


    square_x7 =  353.0
    square_y7 = 217.5 
    square_lat7 = 35.2994619
    square_lon7 =  -120.6630570 

    target_lon, target_lat = get_coordinates(
        plane_lat, plane_lon, plane_bearing, plane_relative_alt, square_x7, square_y7
    )
    print(f"\nPlane coordinates: Latitude = {plane_lat}, Longitude = {plane_lon}")
    print(f"Target Coordinates as calculated: Latitude = {target_lat}, Longitude = {target_lon}")
    print(f"Target Coordinates as measured: Latitude = {square_lat7}, Longitude = {square_lon7}")




MISSION = 4
STABILIZE = 13
MANUAL = 9
RTL = 5
LAND = 6
HOLD = 3


async def run():
    #drone = System()
    #await drone.connect(system_address="udp://:14540")
    
    # await geofence.run()
    
    drone = System(mavsdk_server_address='localhost', port=50051)
    await drone.connect()

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))
    async for position in drone.telemetry.position():
        break
    await drone.action.set_takeoff_altitude(2)
    start_lat, start_lon = position.latitude_deg, position.longitude_deg
    drone.mission.clear_mission()
    await drone.mission.set_return_to_launch_after_mission(False)
    mission_items = []
    drone.action.set_takeoff_altitude(2)
    # mission_items.append(MissionItem(start_lat - 0.0001,
    #                                  start_lon,
    #                                  2,
    #                                  10,
    #                                  True,
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  MissionItem.CameraAction.NONE,
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  MissionItem.VehicleAction.NONE))
    # mission_items.append(MissionItem(start_lat,
    #                                  start_lon,
    #                                  2,
    #                                  10,
    #                                  True,
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  MissionItem.CameraAction.NONE,
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  MissionItem.VehicleAction.NONE))
    # mission_items.append(MissionItem(start_lat,
    #                                  start_lon + 0.0001,
    #                                  2,
    #                                  10,
    #                                  True,
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  MissionItem.CameraAction.NONE,
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  MissionItem.VehicleAction.NONE))
    # mission_items.append(MissionItem(start_lat + 0.0001,
    #                                  start_lon,
    #                                  2,
    #                                  10,
    #                                  True,
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  MissionItem.CameraAction.NONE,
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  float('nan'),
    #                                  MissionItem.VehicleAction.NONE))
    
    mission_items.append(MissionItem(35.328972, 
                                     -120.752941,
                                     2,
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
                                     MissionItem.VehicleAction.NONE))
    mission_items.append(MissionItem(35.328000, 
                                     -120.751989,
                                     2,
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
                                     MissionItem.VehicleAction.NONE))
    mission_items.append(MissionItem(35.328972, 
                                     -120.752941,
                                     2,
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
                                     MissionItem.VehicleAction.NONE))
    mission_items.append(MissionItem(start_lat,
                                     start_lon,
                                     2,
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
                                     MissionItem.VehicleAction.NONE))

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
    
    #await drone.action.takeoff()
    # await asyncio.sleep(5)
    # await drone.mission.start_mission()
    
    await monitor_flight_mode(drone)
    await termination_task

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")

async def monitor_flight_mode(drone):
    previous_flight_mode = None
    flag = False

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            print(f"Flight mode changed to: {flight_mode}")
            if flight_mode.value == STABILIZE or flight_mode.value == MANUAL:
                print("Flight mode changed from MISSION to MANUAL. Pausing mission...")
                await drone.mission.pause_mission()
            # elif flight_mode == RTL:
            #     break
            elif flight_mode.value == HOLD:
                print("switched to hold mode")
                await drone.action.hold()
            else:
                await drone.mission.start_mission()
            previous_flight_mode = flight_mode
        async for mission_progress in drone.mission.mission_progress():
        
            if mission_progress.current == mission_progress.total:
                await drone.action.land()
                flag = True
            break
        if flag is True:
            break
            
            
async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

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


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
