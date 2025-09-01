#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from g4_interfaces.srv import Control as ControlSrv
from g4_interfaces.msg import ControlEvent

class G4ControlNode(Node):
    def __init__(self) -> None:
        super().__init__('g4_control_node')
        self.pub_event = self.create_publisher(ControlEvent, '/g4/control_events', 10)
        self.grid_reset_cli = self.create_client(Trigger, '/g4/grid_reset')
        self.srv = self.create_service(ControlSrv, '/g4/control', self.on_control)
        self.get_logger().info('G4ControlNode ready. Service: /g4/control')

    def on_control(self, req: ControlSrv.Request, res: ControlSrv.Response) -> ControlSrv.Response:
        cmd = (req.command or '').strip()
        path = (req.path or '').strip()
        ev = ControlEvent()
        ev.stamp = self.get_clock().now().to_msg()
        ev.command = cmd
        ev.path = path
        self.pub_event.publish(ev)

        if cmd == 'reset':
            if not self.grid_reset_cli.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn('/g4/grid_reset not available, event only.')
            else:
                fut = self.grid_reset_cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

        res.success = True
        res.message = f'ok: {cmd}'
        return res

def main(args=None):
    rclpy.init(args=args)
    node = G4ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
