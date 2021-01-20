
""" Simple Topological Navigation """

import time
from typing import List, Dict
import rclpy

from custom_ros2 import (
    Node,
    ActionSingleServer,
    ActionClient
)

from nav2_msgs.action import NavigateToPose
from ros2_topological_nav_interfaces.action import TopoNav
from ros2_topological_nav_interfaces.srv import AddPoint


class TopoNavNode(Node):
    """ Topological Navigation Node """

    def __init__(self):
        super().__init__("topo_nav_node")

        self.__points_dict = {}
        self.__server_canceled = False

        # param names
        nav_action_param_name = "nav_action"
        points_param_name = "points"

        # declaring params
        self.declare_parameter(nav_action_param_name,
                               "navigate_to_pose")
        self.declare_parameter(points_param_name, [])

        # getting params
        nav_action = self.get_parameter(
            nav_action_param_name).get_parameter_value().string_value
        points = self.get_parameter(
            points_param_name).get_parameter_value().string_array_value

        # load points
        self._load_points(points)

        # actions
        self.__action_client = ActionClient(
            self, NavigateToPose, nav_action)
        self.__action_server = ActionSingleServer(self,
                                                  TopoNav,
                                                  "topo_nav",
                                                  execute_callback=self.__execute_server,
                                                  cancel_callback=self.__cancel_callback
                                                  )

        # services
        self.create_service(
            AddPoint, "add_point", self.__add_point,
            callback_group=self.__action_server.callback_group)

    def _load_points(self, points: List[str]):
        """ load points of list strings into a dictionary of floats

        Args:
            points (List[str]): list of points
        """

        for i in range(len(points)):
            mod = i % 5

            coords = {
                1: "x",
                2: "y",
                3: "z",
                4: "w"
            }

            if mod == 0:
                self.__points_dict[points[i]] = {}
            else:
                self.__points_dict[points[i - mod]
                                   ][coords[mod]] = float(points[i])

    def __add_point(self, req: AddPoint.Request,
                    res: AddPoint.Response) -> AddPoint.Response:
        """ srv callback to add new points

        Args:
            req (AddPoint.Request): request with the point
            res (AddPoint.Response): overwrites an existing point?

        Returns:
            AddPoint.Response: overwrites an existing point?
        """

        overwrite = req.id in self.__points_dict
        res.overwrite = overwrite

        if not overwrite:
            self.__points_dict[req.id] = {}

        self.__points_dict[req.id]["x"] = req.x
        self.__points_dict[req.id]["y"] = req.x
        self.__points_dict[req.id]["z"] = req.x
        self.__points_dict[req.id]["w"] = req.x

        return res

    def _create_goal(self, point: Dict[str, float]) -> NavigateToPose.Goal:
        """ create a goal for ros2 navigation

        Args:
            point (Dict[float]): dictionary of floats that represent a point

        Returns:
            NavigateToPose.Goal: ros2 navigation goal
        """

        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = point["x"]
        goal.pose.pose.position.y = point["y"]
        goal.pose.pose.orientation.z = point["y"]
        goal.pose.pose.orientation.w = point["y"]

        return goal

    def destroy(self):
        """ destroy node method """

        self.__action_server.destroy()
        super().destroy_node()

    def __cancel_callback(self):
        """ cancel action server """

        self.__server_canceled = True
        self.__action_client.cancel_goal()

    def __execute_server(self, goal_handle):
        """ execute action server

        Args:
            goal_handle: goal_handle
        """

        self.__server_canceled = False
        request = goal_handle.request
        result = TopoNav.Result()

        if request.point not in self.__points_dict:
            goal_handle.abort()
            return result

        point = self.__points_dict[request.point]
        goal = self._create_goal(point)

        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

        if self.__action_client.is_succeeded():
            goal_handle.succeed()

        elif self.__action_client.is_cancelled() and self.__server_canceled:
            while goal_handle.is_active:
                time.sleep(1)
            goal_handle.canceled()

        else:
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    node = TopoNavNode()

    node.join_spin()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
