# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

def change_mode(self, mode):
        """ Change the operation mode of the pixhawk
        mode is a string specifing one of the avalible flight modes
        """
        # Check if mode is available
        if mode not in self._mav.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self._mav.mode_mapping().keys()))
            exit(1)

        # Get mode ID
        mode_id = self._mav.mode_mapping()[mode]
        # Set new mode
        self._mav.mav.set_mode_send(
            self._mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

        # Check ACK
        ack = False
        ack_msg_id = mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE
        while not ack:
            # Wait for ACK command
            ack_msg = self._mav.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            # Check if command in the same in `set_mode`
            if ack_msg['command'] != ack_msg_id:
                continue

            # Print the ACK result !
            res = mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']]
            print(res.description)
            ack = True