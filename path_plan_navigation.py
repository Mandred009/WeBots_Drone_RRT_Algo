from RRT import RRT_INIT
from pymavlink import mavutil
import time


def get_curr_coord():
    latitude=0
    longitude=0
    for i in range(0,10):
        msg=connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        latitude=msg.lat
        longitude=msg.lon
    
    return latitude,longitude

def do_planning(end_pts):
    path=RRT_INIT(end_pts)
    path=path[-1::-1]
    return path


if __name__=="__main__":
    TARGET_LAT_CHANGE=4500
    TARGET_LON_CHANGE=0

    connection = mavutil.mavlink_connection('udpin:localhost:14550')
    connection.wait_heartbeat()
    print(f"Heartbeat from system {connection.target_system}, component {connection.target_component}")

    mode = connection.mode_mapping().get('GUIDED')
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode)
    print("GUIDED mode set.")
    time.sleep(1)

    CURR_LAT,CURR_LON=get_curr_coord()

    TARGET_LAT=CURR_LAT+TARGET_LAT_CHANGE
    TARGET_LON=CURR_LON+TARGET_LON_CHANGE

    dist_lat=(TARGET_LAT_CHANGE/10**7)*(111*1000)
    dist_lon=(TARGET_LON_CHANGE/10**7)*(111*1000)

    print(dist_lat)
    print(dist_lon)

    path=do_planning((dist_lon,dist_lat))

    # Arm
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    print("Arming command sent.")

    time.sleep(2)

    # Takeoff to 5 meters
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, 5)
    print("Takeoff command sent.")
    while(True):
        msg=connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print(msg.relative_alt, msg.lat, msg.lon)
        if(msg.relative_alt==5000):
            break

    print("Starting Waypoint Nav")
    for i in path:
        diff_x=i[0][1]-i[1][1]
        diff_y=-i[0][0]+i[1][0]

        print(diff_x,diff_y)

        CURR_LAT+=((diff_x/1000)/111)*(10**7)
        CURR_LON+=((diff_y/1000)/111)*(10**7)

        connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message
                    (10, connection.target_system,
                    connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                    int(0b110111111000), int(CURR_LAT), int(CURR_LON), 5, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
        
        while(True):
            msg=connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if((abs(msg.lat-CURR_LAT)<100 and abs(msg.lon-CURR_LON)<100) or (abs(TARGET_LAT-CURR_LAT)<100 and abs(TARGET_LON-CURR_LON)<100)):
                # print("Point")
                break

    print("Reached")

    time.sleep(3)

    print("Landing")
    for i in range(5,-1,-1):
        connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message
                            (10, connection.target_system,
                            connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                            int(0b110111111000), int(CURR_LAT), int(CURR_LON), i, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
        
        time.sleep(1)

    time.sleep(2)

    # Disarm
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196, 0, 0, 0, 0, 0)
    print("Done Navigation")







