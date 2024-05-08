#initialize drone object via mavsdk
from mavsdk import System
import asyncio
# from roboflowoak import RoboflowOak
# import cv2
# import numpy as np

#global parameters_______________________________________________________________________________________
#takeoff altitude in meters
takeoff_altitude = 10
home_altitude = 0
current_altitude = 0
current_lat = 0
current_lon = 0
found = False #global for predictions

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


async def initialize_drone():
    global takeoff_altitude
    global home_altitude
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
            current_altitude = altitude
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            print(f"Altitude: {altitude}\tLatitude: {position.latitude_deg}\tLongitude: {position.longitude_deg}")
#constantly prints altitude and latitude/longitude of the drone

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
    print(f"latitude num steps is {lat_num_steps}")
    lat_step_size = (abs(start_lat - end_lat) / lat_num_steps)
    print(f"latitude step size is {lat_step_size}")
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
async def move_drone(drone, path):
    for coord in path:
        await drone.action.goto_location(coord[0], coord[1], takeoff_altitude, 0)
        #loop until drone reaches desired location
        margin_of_error = 0.00002  # Adjust as needed
        #print(f"-- Waiting for drone to reach {coord}")
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
    #print(f"-- Waiting for drone to reach {path[next_index]}")

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

    drone = await initialize_drone()

    #status_text_task = asyncio.create_task(print_status_text(drone))

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
    
    start_lat = current_lat
    start_lon = current_lon
    print(f"start_lat: {start_lat}, start_lon: {start_lon}")
    end_lat = start_lat + 0.001
    end_lon = start_lon + 0.001
    flying_altitude = home_altitude + 30.0
    sweeps = 3
    step_size = 0.0005
    path = await generate_path(start_lat, start_lon, end_lat, end_lon, sweeps, step_size)
    index = 1
    path_length = len(path)
    print(path)
    
    # print(f"attempting to move drone to lat: {current_lat + 0.0005}, lon: {current_lon + 0.0005}, altitude: {flying_altitude}")
    # await drone.action.goto_location(current_lat + 0.0005, current_lon + 0.0005, flying_altitude, 0)
    # while (1):
    #     #do nothing
    #     await asyncio.sleep(1)

    # infinite while loop that moves the drone to the next location on the path
    while True:
        #move to next location
        await move_to_next_location(drone, path, index, flying_altitude)
        index += 1
        if index == path_length:
            break

    # await move_drone(drone, path)


    # #utilizing current location to move drone 5 meters to the left
    # target_longitude = location.longitude_deg - 0.0005
    # print(f"target_longitude: {target_longitude}")
    # await drone.action.goto_location(location.latitude_deg, target_longitude, location.absolute_altitude_m, 0)

    # #loop until drone reaches desired location
    # margin_of_error = 0.00001  # Adjust as needed
    # print("-- Waiting for drone to reach desired location")
    # async for location in drone.telemetry.position():
    #     if abs(location.longitude_deg - target_longitude) < margin_of_error:
    #         break

    #print current drone location once
    async for location in drone.telemetry.position():
        print(f"-- Current Position: {location.latitude_deg}, {location.longitude_deg}")
        break

    print("-- Landing")
    await drone.action.land()

    # status_text_task.cancel()
    # try:
    #     # Await the task to handle its cancellation
    #     await status_text_task
    # except asyncio.CancelledError:
    #     print("-- print_status_text task cancelled")

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