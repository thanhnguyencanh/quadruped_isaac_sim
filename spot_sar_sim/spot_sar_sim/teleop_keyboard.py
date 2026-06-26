"""teleop_keyboard — drive Spot from the keyboard by publishing geometry_msgs/Twist to /cmd_vel.

A small ROS 2 node (runs with system Jazzy / rclpy, NOT Isaac's python) that turns key
presses into /cmd_vel Twist messages — the same topic the Isaac apps subscribe to
(spot_cmd_vel_app.py / spot_perception_app.py). It is just another publisher, so it must run
on the SAME ROS_DOMAIN_ID as the sim (this project standardizes on 42).

  set-and-hold model: a key sets the velocity and it KEEPS being published until you change
  it or press SPACE to stop (a terminal can't see key releases, so there is no auto-stop).

Keys
  w / s : forward / backward      (linear.x)
  a / d : turn left / right       (angular.z)
  q / e : strafe left / right     (linear.y)
  arrow keys also work (up/down = fwd/back, left/right = turn)
  space or x : STOP (zero all)
  + / - : increase / decrease the speed scale
  Ctrl-C : quit (sends a final stop)

Run:  ROS_DOMAIN_ID=42 ros2 run spot_sar_sim teleop_keyboard
"""
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Spot flat-terrain policy trained range (matches the clip limits in the Isaac apps).
VX_LIM, VY_LIM, WZ_LIM = 1.5, 1.0, 1.0

HELP = """
spot teleop — keyboard control of /cmd_vel
------------------------------------------
   w/s : forward/back      a/d : turn L/R      q/e : strafe L/R
   arrows: fwd/back/turn   space or x : STOP   +/- : speed scale
   Ctrl-C : quit
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("spot_teleop_keyboard")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.vx = self.vy = self.wz = 0.0
        self.lin_speed = 0.6   # m/s   per directional key
        self.ang_speed = 0.8   # rad/s per turn key
        self.timer = self.create_timer(0.05, self._publish)  # 20 Hz steady stream

    def _publish(self):
        msg = Twist()
        msg.linear.x = max(-VX_LIM, min(VX_LIM, self.vx))
        msg.linear.y = max(-VY_LIM, min(VY_LIM, self.vy))
        msg.angular.z = max(-WZ_LIM, min(WZ_LIM, self.wz))
        self.pub.publish(msg)

    def apply(self, key: str) -> bool:
        """Update the command from a key. Returns False to request quit."""
        if key in ("\x03",):  # Ctrl-C
            return False
        if key in ("w", "W", "UP"):
            self.vx = self.lin_speed
        elif key in ("s", "S", "DOWN"):
            self.vx = -self.lin_speed
        elif key in ("a", "A", "LEFT"):
            self.wz = self.ang_speed
        elif key in ("d", "D", "RIGHT"):
            self.wz = -self.ang_speed
        elif key in ("q", "Q"):
            self.vy = self.lin_speed
        elif key in ("e", "E"):
            self.vy = -self.lin_speed
        elif key in (" ", "x", "X"):
            self.vx = self.vy = self.wz = 0.0
        elif key in ("+", "="):
            self.lin_speed = min(VX_LIM, self.lin_speed + 0.1)
            self.ang_speed = min(WZ_LIM, self.ang_speed + 0.1)
        elif key in ("-", "_"):
            self.lin_speed = max(0.1, self.lin_speed - 0.1)
            self.ang_speed = max(0.1, self.ang_speed - 0.1)
        return True

    def status(self) -> str:
        return (f"\rvx={self.vx:+.2f} vy={self.vy:+.2f} wz={self.wz:+.2f} "
                f"| speed lin={self.lin_speed:.1f} ang={self.ang_speed:.1f}   ")


def _read_key(timeout: float) -> str:
    """Read one key (decoding arrow escape sequences) without blocking past `timeout`."""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return ""
    ch = sys.stdin.read(1)
    if ch == "\x1b":  # escape sequence (arrow keys): \x1b [ A/B/C/D
        seq = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.001)[0] else ""
        return {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}.get(seq, "\x1b")
    return ch


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    if not sys.stdin.isatty():
        node.get_logger().error("teleop_keyboard needs an interactive terminal (a TTY).")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(HELP)
    print(f"publishing /cmd_vel on ROS_DOMAIN_ID={__import__('os').environ.get('ROS_DOMAIN_ID', '0')} "
          f"— focus THIS terminal and press keys.")
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())  # cbreak: keys arrive immediately, Ctrl-C still raises
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)  # service the 20 Hz publish timer
            key = _read_key(0.05)
            if key and not node.apply(key):
                break
            if key:
                sys.stdout.write(node.status())
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        # send a final stop so Spot doesn't keep moving after we quit
        node.vx = node.vy = node.wz = 0.0
        node._publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\n[teleop] stopped.")


if __name__ == "__main__":
    main()
