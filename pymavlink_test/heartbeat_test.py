#%%
from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14540')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
while True:
    the_connection.wait_heartbeat()
    try: 
        altitude=the_connection.messages['GLOBAL_POSITION_INT'].alt  # Note, you can access message fields as attributes!
        print("Altitude is " + str(altitude))
        print(altitude/1000)
  
    except:
        print('No GPS_RAW_INT message received')

    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
    
