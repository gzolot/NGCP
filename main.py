#initialize drone object via mavsdk
from mavsdk import System
import asyncio

#initialize GCS
# from datetime import datetime
# from Telemetry.RabbitMQ import TelemetryRabbitMQ
# from Types.Telemetry import Telemetry
# from Types.Geolocation import Coordinate
# import time

#initialize computer vision object via roboflowoak
from roboflowoak import RoboflowOak
import cv2
import numpy as np

import math
#global parameters_______________________________________________________________________________________
#takeoff altitude in meters
takeoff_altitude = 2
relative_fligth_altitude = 2
home_altitude = 0.0
current_altitude = 0.0
current_lat = 0.0
current_lon = 0.0
current_pitch = 0.0
current_yaw = 0.0
current_roll = 0.0
current_speed = 0.0
current_battery = 0.0
current_heading = 0.0
found = False #global for predictions

# This function determines the pixel per inch ratio at a given distance. 
# We used inches as this was a convenient unit for measuring the 2x2 red square, 
# and each tile in the EE bldg is exactly 12 inches for measuring distance. 
def pixel_to_real_world_distance(altitude):
    return 512.06 * (altitude ** -0.927)

# altitude requires units of inches
# lat/lon must be formatted (looks like this would require offset by 10^7 from pixhawk coords): 
    # plane_lat = 35.2994616
    # plane_lon = -120.6630686
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
# given a starting coord, bearing, and distance
def great_circle_target(lon1, lat1, bearing, dist, R):
    lat2 = math.asin(math.sin(lat1) * math.cos(dist / R) + 
                     math.cos(lat1) * math.sin(dist / R) * math.cos(bearing))
    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(dist / R) * math.cos(lat1),
                             math.cos(dist / R) - math.sin(lat1) * math.sin(lat2))
    return lon2, lat2


def vision(result_queue):
    rf = RoboflowOak(model="redsquare-gwdyn", confidence=0.5, overlap=0.5,
    version="1", api_key="N5Xs9o02pFDsaVc5pjcd", rgb=True,
    depth=False, device=None, device_name="FRA", blocking=True)
    # Running our model and displaying the video output with detections
    timefound = 0
    while True:
        #t0 = time.time()
        # The rf.detect() function runs the model inference
        result, frame, raw_frame, depth = rf.detect()
        predictions = result["predictions"]

        #update the x and y coordinates:
        x,y = 100, 200
        result_queue.put_nowait((x,y))
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
        #depth - depth map for raw_frame, center-rectified to the center camera
        #To access specific values within "predictions" use: [p.json() for p[a] in predictions]
        # set "a" to the index value you are attempting to access
        # Example: accessing the "y"-value: [p.json() for p[1] in predictions]
        
        # timing: for benchmarking purposes
        # t = time.time()-t0
        # print("FPS ", 1/t)
        #print("PREDICTIONS ", [p.json() for p in predictions])
        if predictions is not None:
            if found is True:
                timefound+=1
            found = True
        elif predictions is None:
            found = False
            timefound = 0
        # setting parameters for depth calculation
        #max_depth = np.amax(depth)
        #cv2.imshow("depth", depth/max_depth)
        # displaying the video feed as successive frames
        #cv2.imshow("frame", frame)
    
        # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
        if cv2.waitKey(1) == ord('q'):
            break

async def coordinate_producer(executor, result_queue):
    loop = asyncio.get_running_loop()
    while True:
        # Wait for new coordinates from the computer vision task
        x, y = await loop.run_in_executor(executor, result_queue.get)
        await coordinate_queue.put((x, y))

async def write_telemetry_data_tofile():
    #write telemetry global data to text file
    #initialize global telemetry data
    global current_altitude
    global current_lat
    global current_lon
    global current_pitch
    global current_yaw
    global current_roll
    global current_speed
    global current_battery
    #create a new file
    with open("telemetry_data.txt", "w") as file:
        while(True):
            file.write(f"current_lat: {current_lat}, current_lon: {current_lon}, current_altitude: {current_altitude}, current_pitch: {current_pitch}, current_yaw: {current_yaw}, current_roll: {current_roll}, current_speed: {current_speed}, current_battery: {current_battery}\n")
            await asyncio.sleep(1)

async def initialize_drone():
    global takeoff_altitude
    global home_altitude
    #local initialization
    # drone = System()
    # await drone.connect(system_address="udp://:14540")

    #nano initialization
    drone = System(mavsdk_server_address='localhost', port=50051)
    await drone.connect()

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
    
    print_altitude_task = asyncio.ensure_future(print_altitude(drone))
    print_attitude_task = asyncio.ensure_future(print_attitude(drone))
    print_battery_and_speed_task = asyncio.ensure_future(print_battery_and_speed(drone))
    print_heading_task = asyncio.ensure_future(print_heading(drone))
    write_telemetry_data_task = asyncio.ensure_future(write_telemetry_data_tofile())
    
    print("fetching amsl altitude at home location......")
    async for terrain_info in drone.telemetry.home():
        home_altitude = terrain_info.absolute_altitude_m
        break

    #set the takeoff altitude
    print(f"-- Setting takeoff altitude to {takeoff_altitude} meters")
    await drone.action.set_takeoff_altitude(takeoff_altitude)
    
    return drone

async def print_altitude(drone):
    """ Prints the altitude when it changes """

    global current_altitude
    global current_lat
    global current_lon
    rounded_previous_altitude = None
    rounded_altitude = None
    async for position in drone.telemetry.position():
        altitude = position.relative_altitude_m
        rounded_altitude = round(altitude)
        if rounded_altitude != rounded_previous_altitude:
            rounded_previous_altitude = rounded_altitude
            # print(f"Altitude: {altitude}\tLatitude: {position.latitude_deg}\tLongitude: {position.longitude_deg}")
        current_altitude = altitude
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
#constantly prints altitude and latitude/longitude of the drone

async def print_attitude(drone):
    global current_pitch, current_yaw, current_roll
    async for attitude in drone.telemetry.attitude_euler():
        current_pitch = attitude.pitch_deg
        current_yaw = attitude.yaw_deg
        current_roll = attitude.roll_deg
        # print(f"Pitch: {current_pitch}, Yaw: {current_yaw}, Roll: {current_roll}")

async def print_heading(drone):
    global current_heading
    async for heading in drone.telemetry.heading():
        current_heading = heading.heading
        # print(f"Heading: {current_heading}")

async def print_battery_and_speed(drone):
    global current_speed, current_battery
    async for battery in drone.telemetry.battery():
        current_battery = battery.remaining_percent * 100
        # print(f"Battery life: {current_battery}%")
    async for velocity in drone.telemetry.velocity_ned():
        current_speed = (velocity.north_m_s**2 + velocity.east_m_s**2 + velocity.down_m_s**2) ** 0.5
        # print(f"Speed: {current_speed} m/s")

#function generates a list of coordinates that the drone will follow
#the list is based on two points that compose two opposite corners of a rectangle
#the drone will sweep throught the rectangle by following a snake pattern
#the sweeps argument represents the number of u turns the drone will make so if the value is 2 the drone path will make a single S shape
#and if the value is 3 the drown will make a w shape
async def generate_path(start_lat, start_lon, end_lat, end_lon, sweeps, step_size):
    path = []
    #generate longitude step size that evenly divides the distance between the two points
    lon_turn_length = (abs(start_lon - end_lon) / sweeps)
    if (lon_turn_length // step_size) == 0:
        lon_step_size = lon_turn_length
    else:
        lon_step_size = lon_turn_length/(lon_turn_length // step_size)
    #print(f"lon_turn_length: {lon_turn_length}, lon_step_size: {lon_step_size}")

    #generate latitude step size that evenly divides the distance between the two points
    lat_num_steps = (abs(start_lat - end_lat) // step_size)
    if(lat_num_steps == 0):
        lat_num_steps = 1
    lat_step_size = (abs(start_lat - end_lat) / lat_num_steps)
    #print(f"lat_num_steps: {lat_num_steps}, lat_step_size: {lat_step_size}")

    current_lat = start_lat
    current_lon = start_lon
    #generate the first sweep
    for i in range(0, sweeps*2+1):
        #print(f"i: {i}")
        if i % 2 == 0:
            if current_lat != end_lat:
                while current_lat < end_lat:
                    #for j in range(start_lat, end_lat+lat_step_size, lat_step_size):
                    #print(f"current_lat: {current_lat}, current_lon: {current_lon}")
                    path.append((current_lat, current_lon))
                    current_lat += lat_step_size
                current_lat = end_lat
            else:
                while current_lat > start_lat:
                    #for j in range(end_lat, start_lat-lat_step_size, -lat_step_size):
                    #print(f"current_lat: {current_lat}, current_lon: {current_lon}")
                    path.append((current_lat, current_lon))
                    current_lat -= lat_step_size
                current_lat = start_lat
            if i == sweeps*2:
                    path.append((current_lat, current_lon))
        else:
            #print(f"current_lon: {current_lon}, current_lon + lon_turn_length: {current_lon + lon_turn_length}")
            top_lon = current_lon + lon_turn_length
            while current_lon < (top_lon):
                #for j in range(current_lon, current_lon+lon_turn_length+lon_step_size, lon_step_size):
                #print(f"current_lat: {current_lat}, current_lon: {current_lon}")
                path.append((current_lat, current_lon))
                current_lon += lon_step_size
            current_lon = top_lon
    return path

#async function that will move drone based on an imput of a bunch of coordinates or a path
async def move_drone(drone, path, altitude):
    for coord in path:
        await drone.action.goto_location(coord[0], coord[1], altitude, 0)
        #loop until drone reaches desired location
        margin_of_error = 0.00003  # Adjust as needed
        print(f"-- Waiting for drone to reach {coord}")
        #utlizing global variables
        while True:
            # Use global variables updated by print_altitude
            global current_lat, current_lon
            if abs(current_lat - coord[0]) < margin_of_error and abs(current_lon - coord[1]) < margin_of_error:
                print(f"-- Drone reached ({current_lat}, {current_lon})")
                break
            await asyncio.sleep(0.5)  # Pause briefly to allow for updates
        
        #not utilizing global variables 
        # async for location in drone.telemetry.position():
        #     if abs(location.latitude_deg - coord[0]) < margin_of_error and abs(location.longitude_deg - coord[1]) < margin_of_error:
        #         print(f"-- Drone reached ({location.latitude_deg}, {location.longitude_deg})")
        #         break

#async function that will move the drone to the next location on a path 
async def move_to_next_location(drone, path, next_index, altitude):
    await drone.action.goto_location(path[next_index][0], path[next_index][1], altitude, 0)
    #loop until drone reaches desired location
    margin_of_error = 0.00003  # Adjust as needed
    print(f"-- Waiting for drone to reach {path[next_index]}")

    #utilizing global variables
    while True:
        # Use global variables updated by print_altitude
        global current_lat, current_lon
        if abs(current_lat - path[next_index][0]) < margin_of_error and abs(current_lon - path[next_index][1]) < margin_of_error:
            print(f"-- Drone reached ({current_lat}, {current_lon})")
            break
        await asyncio.sleep(0.5)  

    #not utilizing globals
    # async for location in drone.telemetry.position():
    #     if abs(location.latitude_deg - path[next_index][0]) < margin_of_error and abs(location.longitude_deg - path[next_index][1]) < margin_of_error and abs(location.relative_altitude_m - altitude) < 1.0:
    #         print(f"-- Drone reached ({location.latitude_deg}, {location.longitude_deg}, {location.relative_altitude_m})")
    #         break

async def run():
    global home_altitude
    global current_altitude
    global current_lat
    global current_lon 
    global current_pitch
    global current_yaw
    global current_roll
    global current_speed
    global current_battery
    global found

    drone = await initialize_drone()

    #status_text_task = asyncio.create_task(print_status_text(drone))

    #sleep for a minute
    await asyncio.sleep(60)

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    #print current drone location once
    async for location in drone.telemetry.position():
        print(f"-- Current Position: {location.latitude_deg}, {location.longitude_deg}")
        break

    #wait until drone reaches takoff altitude
    # async for altitude in drone.telemetry.position():
    #     if abs(altitude.relative_altitude_m - takeoff_altitude) < 1.0:
    #         break
    #wait for 10 seconds
    await asyncio.sleep(10)

    #flight test________________________________________________________________________________________
    #home location
    home_lat = current_lat
    home_lon = current_lon

    #move the drone 5 meters to the left
    flying_altitude = home_altitude + relative_fligth_altitude
    await drone.action.goto_location(current_lat, current_lon + 0.00005, flying_altitude, 0)
    #wait for drone to move to that location
    await asyncio.sleep(10)

    #move drone to home location
    await drone.action.goto_location(home_lat, home_lon, flying_altitude, 0)
    #wait for drone to move to that location
    await asyncio.sleep(10)
    '''
    start_lat = current_lat
    start_lon = current_lon
    print(f"start_lat: {start_lat}, start_lon: {start_lon}")
    end_lat = start_lat + 0.001
    end_lon = start_lon + 0.001
    flying_altitude = home_altitude + relative_fligth_altitude
    sweeps = 3
    step_size = 0.0005
    path = await generate_path(start_lat, start_lon, end_lat, end_lon, sweeps, step_size)
    index = 1
    path_length = len(path)
    print(path)

    #telemetry setup
    #tel = TelemetryRabbitMQ("ERU", "localhost")

    drone_move_task = asyncio.ensure_future(move_drone(drone, path[1:], flying_altitude))

    # infinite while loop that moves the drone to the next location on the path
    while True:

        #update telemetry data
        # data = Telemetry(
        #     pitch=current_pitch,
        #     yaw=current_yaw,
        #     roll=current_roll,
        #     speed=current_speed,
        #     altitude=current_altitude,
        #     batteryLife=current_battery,
        #     currentCoordinate=Coordinate(latitude=current_lat, longitude=current_lon),
        #     lastUpdated=datetime.now()
        # )
        # tel.publish(data)

        if(drone_move_task.done()):
            print("drone move task done")
            break
        await asyncio.sleep(1)
        print(f"current_lat: {current_lat}, current_lon: {current_lon}, current_altitude: {current_altitude}, current_pitch: {current_pitch}, current_yaw: {current_yaw}, current_roll: {current_roll}, current_speed: {current_speed}, current_battery: {current_battery}")


    #print current drone location once
    async for location in drone.telemetry.position():
        print(f"-- Current Position: {location.latitude_deg}, {location.longitude_deg}")
        break
'''
    print("-- Landing")
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
    # start_lat = 0
    # start_lon = 0
    # end_lat = 10.000
    # end_lon = 10.000
    # sweeps = 3
    # step_size = 1
    # path = asyncio.run(generate_path(start_lat, start_lon, end_lat, end_lon, sweeps, step_size))
    # print(path)