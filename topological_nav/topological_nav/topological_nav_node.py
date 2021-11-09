
""" Simple Topological Navigation """

from typing import List
import rclpy

from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose
from topological_nav_interfaces.action import TopoNav
from topological_nav_interfaces.msg import Point
from topological_nav_interfaces.srv import (
    AddPoint,
    GetPoint,
    GetPoints
)

from simple_node import Node


class TopoNavNode(Node):
    """ Topological Navigation Node """

    def __init__(self):
        super().__init__("topological_nav_node")

        self.__points_dict = {}

        # param names
        nav_action_param_name = "nav_action"
        points_param_name = "points"

        # declaring params
        self.declare_parameter(nav_action_param_name,
                               "/navigate_to_pose")
        self.declare_parameter(points_param_name, [])

        # getting params
        nav_action = self.get_parameter(
            nav_action_param_name).get_parameter_value().string_value
        points = self.get_parameter(
            points_param_name).get_parameter_value().string_array_value

        # load points
        self.load_points(points)

        # actions
        self.__action_client = self.create_action_client(
            NavigateToPose, nav_action)
        self.__action_server = self.create_action_server(TopoNav,
                                                         "navigation",
                                                         execute_callback=self.__execute_server,
                                                         cancel_callback=self.__cancel_callback
                                                         )

        # services
        self.create_service(
            AddPoint, "add_point", self.__add_point,
            callback_group=self.__action_server.callback_group)

        self.create_service(
            GetPoint, "get_point", self.__get_point,
            callback_group=self.__action_server.callback_group)

        self.create_service(
            GetPoints, "get_points", self.__get_points,
            callback_group=self.__action_server.callback_group)

    def load_points(self, points: List[str]):
        """ load points of list strings into a dictionary of floats

        Args:
            points (List[str]): list of points
        """

        for i in range(0, len(points), 5):

            self.__points_dict[points[i]] = Pose()
            self.__points_dict[points[i]].position.x = float(points[i + 1])
            self.__points_dict[points[i]].position.y = float(points[i + 2])
            self.__points_dict[points[i]].orientation.z = float(points[i + 3])
            self.__points_dict[points[i]].orientation.w = float(points[i + 4])

    def __get_point(self,
                    req: GetPoint.Request,
                    res: GetPoint.Response) -> GetPoint.Response:
        """ srv callback to get a point

        Args:
            req (GetPoint.Request): request with the point name
            res (GetPoint.Response): point

        Returns:
            GetPoint.Response: point
        """

        if req.point in self.__points_dict:
            res.point = Point()
            res.point.id = req.point
            res.point.pose = self.__points_dict[req.point]

        return res

    def __get_points(self,
                     req: GetPoints.Request,
                     res: GetPoints.Response) -> GetPoints.Response:
        """ srv callback to get all points

        Args:
            req (GetPoints.Request): empry
            res (GetPoints.Response): pointss

        Returns:
            GetPoints.Response: points
        """

        for p_id in self.__points_dict:
            point = Point()
            point.id = p_id
            point.pose = self.__points_dict[p_id]
            res.points.append(point)

        return res

    def __add_point(self,
                    req: AddPoint.Request,
                    res: AddPoint.Response) -> AddPoint.Response:
        """ srv callback to add new points

        Args:
            req (AddPoint.Request): request with the point
            res (AddPoint.Response): overwrites an existing point?

        Returns:
            AddPoint.Response: overwrites an existing point?
        """

        point = req.point
        overwrite = point.id in self.__points_dict
        res.overwrite = overwrite

        if not overwrite:
            self.__points_dict[point.id] = {}

        self.__points_dict[point.id] = point.pose

        return res

    def __cancel_callback(self):
        """ cancel action server """

        self.__action_client.cancel_goal()

    def __execute_server(self, goal_handle) -> TopoNav.Result:
        """ execute action server

        Args:
            goal_handle: goal_handle

        Returns:
            TopoNav.Result: navigation result
        """

        request = goal_handle.request
        result = TopoNav.Result()

        if request.point not in self.__points_dict:
            goal_handle.abort()
            return result

        pose = self.__points_dict[request.point]
        goal = NavigateToPose.Goal()
        goal.pose.pose = pose

        self.__action_client.wait_for_server()
        self.__action_client.send_goal(goal)
        self.__action_client.wait_for_result()

        if self.__action_client.is_succeeded():
            goal_handle.succeed()

        elif self.__action_server.is_canceled():
            self.__action_server.wait_for_canceling()
            goal_handle.canceled()

        else:
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    node = TopoNavNode()

    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
