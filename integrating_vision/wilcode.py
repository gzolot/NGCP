import asyncio
import mavsdk
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from roboflowoak import RoboflowOak
import math
import numpy as np

MISSION = 4
STABILIZE = 13
MANUAL = 9
RTL = 5
LAND = 6
HOLD = 3

async def telemetry(drone):
    global altitude
    global plane_latitude
    global plane_longitude
    global plane_bearing
    while(True):
        async for position in drone.telemetry.position():
            plane_latitude = position.latitude_deg
            plane_longitude = position.longitude_deg
            altitude = position.relative_altitude_m
            break
        async for heading in drone.telemetry.Heading():
            plane_bearing = heading.heading_deg
            break
        asyncio.sleep(0.1)
    
async def run():
    # print ("hello world")
    # drone = System()
    # await drone.connect(system_address="udp://:14540")
    # await geofence.run()
    
    drone = System(mavsdk_server_address='localhost', port=50051)
    await drone.connect()

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    async for position in drone.telemetry.position():
        break
    start_lat, start_lon = position.latitude_deg, position.longitude_deg
    drone.mission.clear_mission()
    await drone.mission.set_return_to_launch_after_mission(False)
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone))
    drone.action.set_takeoff_altitude(20)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    # await asyncio.sleep(5)
    # await drone.mission.start_mission()
    await drone.action.arm()
    await telemetry(drone)
    await monitor_flight_mode(drone)
    await vision()
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
            elif flight_mode.value == MISSION:
                await drone.mission.start_mission()
            previous_flight_mode = flight_mode
        async for mission_progress in drone.mission.mission_progress():
            if mission_progress.current == mission_progress.total:
                await drone.action.hold()
                flag = True
            break
        if flag is True:
            break
            
            
async def observe_is_in_air(drone):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            await asyncio.get_event_loop().shutdown_asyncgens()

            return
        
def pixel_to_real_world_distance(altitude):
    return 512.06 * (altitude ** -0.927)

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

async def vision():
    rf = RoboflowOak(model="redsquare-gwdyn", confidence=0.50,
    overlap=0.5, version="1",
    api_key="N5Xs9o02pFDsaVc5pjcd", rgb=True, depth=False,
    device=None, device_name="NGCP", blocking=True)
    while True:
        # t0 = time.time()
        result, frame, raw_frame, depth = rf.detect()

        frame_height, frame_width = frame.shape[:2]
        print("Frame Dimensions: {}x{}".format(frame_width, frame_height))

        predictions = result["predictions"]
        

        #{
        #    predictions:
        #    [ {
        #        x: (middle),
        #        y:(middle),
        #        width: ,
        #        height: ,
        #        depth: ###->,
        #        confidence: ,
        #        class: ,
        #        mask: { }
        #       }
        #    ]
        #}
        #frame - frame after preprocs, with predictions
        #raw_frame - original frame from your OAK
        #depth - depth map for raw_frame, center-rectified
        # to the center camera 
        # To access specific values within "predictions" use:
        # p.json()[a] for p in predictions
        # set "a" to the index you are attempting to access
        # Example: accessing the "y"-value:
        # p.json()['y'] for p in predictions
        
        # t = time.time()-t0
        # print("INFERENCE TIME IN MS ", 1/t)
        
        for p in predictions:
            x = p.json()['x']
            y = p.json()['y']
            width = p.json()['width']
            height = p.json()['height']
            print("Center X:", x, "Center Y:", y, "Width:", width, "Height:", height)
            # 
            # TODO: 
            # 1. Acquire plane gps coordinates, altitude, and bearing.
            #       a. Assumption: bearing is clockwise angle in degrees from true NORTH. 
            # 2. apply offset so that plane coordinates corresponds to the camera frame center (depends on camera mounting location and size of drone)
            # 3. fill in values for get_coordinates() function call below. 
            #
            if (p.json()['class'] == 'RedSquare'):
                target_longitude, target_latitdue = get_coordinates(plane_latitude, plane_longitude, plane_bearing, altitude, x, y)
                

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
