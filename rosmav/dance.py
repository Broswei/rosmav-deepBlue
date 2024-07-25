#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn


class DanceNode(Node):
    _move_forward = False
    _move_back = False
    _move_left = False
    _move_right = False
    _move_up = False
    _move_down = False
    _turn_left = False
    _turn_right = False
    _lights_on = False
    _step_counter = 0

    def __init__(self):
        super().__init__("dancing_node")

        self.command_pub = self.create_publisher(
            OverrideRCIn, "bluerov2/override_rc", 10
        )

        self.loop = self.create_timer(0.1, self._loop)

    def _set_neutral_all_channels(self):
        neutral = OverrideRCIn()
        neutral.channels = [1500] * 8 + [1000] * 2
        self.command_pub.publish(neutral)

    def _loop(self):
        if self._step_counter/10 > 60:
            self.destroy_node()
            return
        self._dance_moves()

        # See https://www.ardusub.com/developers/rc-input-and-output.html#rc-input
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        if self._move_forward:
            commands.channels[4] = 1600
        elif self._move_back:
            commands.channels[4] = 1400
        if self._move_left:
            commands.channels[5] = 1400
        elif self._move_right:
            commands.channels[5] = 1600
        if self._move_up:
            commands.channels[2] = 2000
        elif self._move_down:
            commands.channels[2] = 1000
        if self._turn_left:
            commands.channels[3] = 1250
        elif self._turn_right:
            commands.channels[3] = 1750
        if self._lights_on:
            commands.channels[8] = 2000
            commands.channels[9] = 2000
        else:
            commands.channels[8] = 1000
            commands.channels[9] = 1000
       

        self.command_pub.publish(commands)

    def _dance_moves(self):
        self._step_counter += 1

        if self._step_counter > 0:
            print (self._step_counter)

        if self._step_counter/10 < 7:
            pass

        # 1 stanza
        elif self._step_counter/10 < 18:
            self._lights_on = True
            self._move_forward = True
            self._move_back = False 

        elif self._step_counter/10 < 26:
            self._move_forward = False
            self._move_back = True # music ends on second 19

        elif self._step_counter/10 < 32: # left
            self._lights_on = False
            self._move_back = False

            self._move_left = True
            self._move_right = False

        elif self._step_counter/10 < 38:  #right
            self._move_left = False
            self._move_right = True 
        
        # 2nd stanza
        
        elif self._step_counter/10 < 40:
            self._move_right = False

            self._turn_left = True
            self._turn_right = False

        elif self._step_counter/10 < 43:
            self._turn_left = False
            self._turn_right = True

        elif self._step_counter/10 < 52:
            # keep turning right

            self._move_down = True

        # 3 stanza
        
        elif self._step_counter/10 < 54:
            self._turn_right = False
            self._move_down = False
            self._move_up = True

        elif self._step_counter/10 < 57:
            # keep up
            self.turn_left = True

        elif self._step_counter/10 < 63:
            self._move_up = False
            self._turn_left = False
            self._turn_right = True

        elif self._step_counter/10 < 65:
            self._move_up = True

        else:
            self._move_up = False



    def destroy_node(self):
        self._set_neutral_all_channels()
        self.mavlink.arducopter_disarm()
        self.mavlink.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    danceNode = DanceNode()

    try:
        rclpy.spin(danceNode)
    except KeyboardInterrupt:
        pass
    finally:
        danceNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
