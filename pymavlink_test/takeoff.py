
#%%
from pymavlink import mavutil
import time
 
# Connect to PX4 SITL
connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")
 
# Wait for the heartbeat signal
connection.wait_heartbeat()
print("Heartbeat received. System is online.")
 
# Set PX4 mode to GUIDED (use mode number)
PX4_MODE_GUIDED = 4  
connection.mav.set_mode_send(
    connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    PX4_MODE_GUIDED
)
print("Mode set to GUIDED.")
 
time.sleep(2)
 
# Arm the UAV
connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
print("UAV is armed.")
 
# Wait for arming confirmation
while not connection.motors_armed():
    time.sleep(1)
 
# Send takeoff command
altitude = 10
connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,  # Confirmation
    0,  # Min pitch
    0,  # Empty param
    0,  # Empty param
    0,  # Empty param
    0,  # Latitude
    0,  # Longitude
    altitude  # Altitude
)
 
print(f"Takeoff command sent to {altitude}m altitude.")
 
# Wait for altitude confirmation
time.sleep(10)
msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
print(f"Current Altitude: {msg.relative_alt / 1000} meters")
 
print("Takeoff complete.")
# %%
