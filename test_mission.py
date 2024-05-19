#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
import geofence
MISSION = 4
MANUAL = 9
RTL = 5
LAND = 6
HOLD = 3


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    # await geofence.run()
    
    # drone = System(mavsdk_server_address='localhost', port=50051)
    # await drone.connect()

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
    mission_items = []
    drone.action.set_takeoff_altitude(2)
    mission_items.append(MissionItem(start_lat - 0.00008,
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
    await drone.action.arm()
    
    #await drone.action.takeoff()
    # await asyncio.sleep(5)

   
    
    await monitor_flight_mode(drone)
    await drone.action.land()
    await termination_task

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")

async def monitor_flight_mode(drone):
    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            print(f"Flight mode changed to: {flight_mode}")
            if flight_mode == MANUAL:
                print("Flight mode changed from MISSION to MANUAL. Pausing mission...")
                await drone.mission.pause_mission()
            elif flight_mode == RTL:
                break
            elif flight_mode == HOLD:
                await drone.action.hold()
            else:
                await drone.mission.start_mission()
            previous_flight_mode = flight_mode
        if drone.mission.is_mission_finished():
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
