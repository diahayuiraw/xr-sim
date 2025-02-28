
#%%
from pymavlink import mavutil
import time

# Establish connection to the MAVLink device (adjust as needed)
# Example: "/dev/ttyUSB0", baud=57600 for serial, or "udp:127.0.0.1:14550" for UDP
connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")

# Wait for the heartbeat to ensure communication is established
print("Waiting for heartbeat from the UAV...")
connection.wait_heartbeat()
print("Heartbeat received. System is online.")

# Set system mode to GUIDED (PX4: OFFBOARD)
print("Setting mode to GUIDED...")
mode = 'STABILIZED'  # Change to 'OFFBOARD' for PX4-based UAVs
mode_id = connection.mode_mapping().get(mode)

if mode_id is None:
    raise Exception(f"Mode {mode} not found in mode mapping.")

connection.mav.set_mode_send(
    connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id[0]
)
print(f"Mode set to {mode}.")

# Send ARM command
print("Arming the UAV...")
connection.mav.command_long_send(
    connection.target_system,  # Target system ID
    connection.target_component,  # Target component ID
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command: ARM/DISARM
    0,  # Confirmation
    1,  # 1 to arm, 0 to disarm
    0, 0, 0, 0, 0, 0  # Unused parameters
)

# Wait for arming confirmation
while True:
    msg = connection.recv_match(type='HEARTBEAT', blocking=True)
    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("UAV is armed.")
        break
    time.sleep(1)

# Send takeoff command (if needed)
altitude = 10  # Desired takeoff altitude in meters
print(f"Sending takeoff command to {altitude}m altitude...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff command
    0,  # Confirmation
    0, 0, 0, 0, 0, 0, 1000  # Altitude as the last parameter
)

print("Takeoff command sent.")

# Optionally, monitor takeoff status
while True:
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg and msg.relative_alt / 1000.0 >= altitude * 0.95:  # Altitude check
        print(f"UAV has reached {altitude}m altitude.")
        break
    time.sleep(1)

print("UAV is airborne.")

# %%
