import time
import cv2
from imutils.video import VideoStream
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from numpy import array

#vehicle = connect('udp:127.0.0.1:14551',wait_ready=True)
vehicle = connect('/dev/ttyACM0',baud=115200,wait_ready=True)

def send_ned_velocity(Cx, Cy, velocity_z) :
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        Cx, Cy, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    vehicle.send_mavlink(msg)

def camera() :
    
    counter = 0
    cap = VideoStream(src=0).start()
    low_blue = array([2, 49, 57])
    up_blue = array([15, 255, 255])

    while True :
        frame = cap.read()
        blur_frame = cv2.GaussianBlur(frame,(5,5), 0) #  5 adalah intensitas blur
        hsv = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low_blue, up_blue)

        _, contours, _= cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        area=[cv2.contourArea(contour) for contour in contours]

        if (not(area==[])) :
            idx=area.index(max(area))
            cv2.drawContours(blur_frame, contours[idx], -1, (0, 0, 255), 3)
            (x,y),radius = cv2.minEnclosingCircle(contours[idx])
            cv2.circle(blur_frame,(int(x), int(y)),int(radius),(0,255,0),3)
            Cy=(x-320)/1000
            Cx=-(y-240)/1000
            print('X=='+str(Cx)+' && '+'Y=='+str(Cy))
            send_ned_velocity(Cx,Cy,0.4)
        else :
            send_ned_velocity(0,0,0.2)
            print("BLANK")
            counter +=1
            if (counter==10) :
                break

        cv2.line(blur_frame,(0,240),(640,240),(255,255,255),2)
        cv2.line(blur_frame,(320,0),(320,480),(255,255,255),2)
        cv2.imshow("gausian", blur_frame)
        time.sleep(0.2)
        
        key = cv2.waitKey(1)
        if (key == 27) :
            break
        
        if (vehicle.location.global_relative_frame.alt<=2.5) :
            break
        
    cv2.destroyAllWindows()
    cap.stop()    

def move_servo(pin, value):
  msg = vehicle.message_factory.command_long_encode(
	0, 0,    # target_system, target_component
	mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
	0, #confirmation
	pin,    # servo number
	value,          # servo position between 1000 and 2000
	0, 0, 0, 0, 0)    # param 3 ~ 7 not used
  vehicle.send_mavlink(msg)

def jarak(awal,akhir):
    dlat = awal.lat - akhir.lat
    dlon = awal.lon -akhir.lon
    return(( dlat**2 + dlon**2 )**0.5)

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not(vehicle.is_armable):
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not(vehicle.armed):
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if (vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95):
            print("Reached target altitude")
            break
        time.sleep(1)

# Eksekusi 
arm_and_takeoff(5)
print("Take off Selesai")

# Hover for 2 seconds
vehicle.airspeed = 5

#lock Log
srv_v=[[(9,1500),(10,1650),(11,1300),(12,1500)],[(9,550),(10,800),(10,2500),(11,550),(11,2200),(12,600),(12,2560)]]
for elem in srv_v[0] :
    move_servo(elem[0],elem[1])
print("Log Locked")

for i in range(2,5) :
    with open("wp.waypoints") as wk :
        g=wk.readlines()
        point = LocationGlobalRelative(float(g[i].split('\t')[8]), float(g[i].split('\t')[9]), 5)
    vehicle.simple_goto(point)
    while(jarak(vehicle.location.global_relative_frame,point)>0.00001):
        print(jarak(vehicle.location.global_relative_frame, point))
        time.sleep(0.7)
    print("!!!SAMPAI!!!>>Wp>>"+str(i-1))
    time.sleep(0.1)
    camera()
    print("Dropping Log")
    move_servo(srv_v[1][i-2][0],srv_v[1][i-2][1])
    time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
#camera()
