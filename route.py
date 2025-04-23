from enum import Enum
from typing import List, Dict, Tuple

import numpy as np

from read import Read
from support import *


class VehicleType(Enum):
    VAN_ONLY = 1
    ROBOT_ONLY = 2
    VAN_CARRY_ROBOT = 3

    def __str__(self):
        if self == VehicleType.VAN_ONLY:
            return "Van Only"
        elif self == VehicleType.ROBOT_ONLY:
            return "Robot Only"
        elif self == VehicleType.VAN_CARRY_ROBOT:
            return "Van Carry Robot"
        else:
            return "Unknown Vehicle Type"


class VehicleRoute:
    """
    Represents a van and robot route of vehicle number k in 2E-VREC problem
    """

    def __init__(self, distance_matrix: np.ndarray, depot: int, unassigned_stations: List[int], customer_list,
                 station_list, customer_demand, van_params=Read().parameters()[0],
                 robot_params=Read().parameters()[1]):
        self.distance_matrix: np.ndarray = distance_matrix
        self.depot: int = depot
        self.van_params: Dict[str, float] = van_params
        self.robot_params: Dict[str, float] = robot_params

        # List of node for van k
        self._van_route: List[Tuple[int, VehicleType]] = [(self.depot, VehicleType.VAN_CARRY_ROBOT),
                                                          (self.depot, VehicleType.VAN_CARRY_ROBOT)]

        # List of node for robot k
        self._robot_route: List[Tuple[int, VehicleType]] = [(self.depot, VehicleType.VAN_CARRY_ROBOT),
                                                            (self.depot, VehicleType.VAN_CARRY_ROBOT)]

        self.unassigned_stations: List[int] = unassigned_stations
        self.customer_list: List[int] = customer_list
        self.station_list: List[int] = station_list
        self.customer_demand = customer_demand

    def get_unassigned_stations(self) -> List[int]:
        return self.unassigned_stations

    def remove_unassigned_station(self, station: int):
        if station in self.unassigned_stations:
            self.unassigned_stations.remove(station)
        else:
            raise ValueError(f"Station {station} not found in unassigned stations.")

    def set_van_route(self, van_route: List[Tuple[int, VehicleType]]):
        self._van_route = van_route

    def set_robot_route(self, robot_route: List[Tuple[int, VehicleType]]):
        self._robot_route = robot_route

    def get_van_route(self) -> List[Tuple[int, VehicleType]]:
        return self._van_route

    def get_robot_route(self) -> List[Tuple[int, VehicleType]]:
        return self._robot_route

    def get_route_cost(self) -> float:
        cost: float = 0
        # Calculate the cost of the van route
        for i in range(len(self._van_route) - 1):
            from_node = self._van_route[i][0]
            to_node = self._van_route[i + 1][0]
            cost += self.distance_matrix[from_node][to_node] / self.van_params['speed'] * self.van_params[
                'travel_cost_rate']

        # Calculate the cost of the robot route
        for i in range(len(self._robot_route) - 1):
            if self._robot_route[i][1] == VehicleType.ROBOT_ONLY:
                from_node = self._robot_route[i][0]
                to_node = self._robot_route[i + 1][0]
                cost += self.distance_matrix[from_node][to_node] / self.robot_params['speed'] * self.robot_params[
                    'travel_cost_rate']
        return cost

    def get_van_demand(self) -> float:
        """
        Calculate the total demand of the van and robot of vehicle k.
        :return: Total demand of the van route.
        """
        total_demand = sum(
            self.customer_demand.get(node[0], 0) for node in self._van_route
        )
        total_demand += sum(
            self.customer_demand.get(node[0], 0) for node in self._robot_route
            if node[1] == VehicleType.ROBOT_ONLY
        )
        return total_demand

    def get_robot_demand(self, form_station: int, to_station: int) -> float:
        """
        Calculate the total demand of the robot of vehicle k from form_station to to_station.
        :return: Total demand of the robot route.
        """
        total_demand = 0
        collecting = False

        for node_id, vehicle_type in self._robot_route:
            if node_id == form_station:
                collecting = True
            if collecting and vehicle_type == VehicleType.ROBOT_ONLY and node_id in self.customer_demand:
                total_demand += self.customer_demand[node_id]
            if node_id == to_station:
                break

        return total_demand

    # 1: Sau depot, -2: Trước depot cuối
    def add_customer_robot(self, customer: int, vehicle_type: VehicleType, position=-1):
        self._robot_route.insert(position, (customer, vehicle_type))

    def add_customer_van(self, customer: int, vehicle_type: VehicleType, position=-1):
        self._van_route.insert(position, (customer, vehicle_type))

    def remove_customer(self, customer: int, vehicle_type: VehicleType):
        """
        Remove a customer from the route.
        :param customer: The customer to remove.
        :param vehicle_type: The type of vehicle (VAN, ROBOT).
        """
        if vehicle_type == VehicleType.VAN_CARRY_ROBOT:
            self._van_route.remove((customer, vehicle_type))
        elif vehicle_type == VehicleType.ROBOT_ONLY:
            self._robot_route.remove((customer, vehicle_type))

    def add_station_robot(self, station: int, vehicle_type: VehicleType, position=-1):
        self._robot_route.insert(position, (station, vehicle_type))

    def add_station_van(self, station: int, vehicle_type: VehicleType, position=-1):
        self._van_route.insert(position, (station, vehicle_type))

    def remove_station(self, station: int, vehicle_type: VehicleType):
        """
        Remove a station from the route.
        :param station: The station to remove.
        :param vehicle_type: The type of vehicle (VAN, ROBOT).
        """
        if vehicle_type == VehicleType.VAN_CARRY_ROBOT:
            self._van_route.remove((station, vehicle_type))
        elif vehicle_type == VehicleType.ROBOT_ONLY:
            self._robot_route.remove((station, vehicle_type))

    def __str__(self):
        van_route_str = "Van Route: " + " -> ".join([f"{node[0]}" for node in self._van_route])
        robot_route_str = "Robot Route: " + " -> ".join([f"{node[0]}" for node in self._robot_route])
        return f"{van_route_str}\n{robot_route_str}"

    def open_robot_route(self, customer: int, station: int):
        if len(self._robot_route) == 2:  # Tuyến đường robot mới
            # Nối depot đến station
            self.add_station_van(station, VehicleType.VAN_CARRY_ROBOT, position=-1)
            self.add_station_robot(station, VehicleType.VAN_CARRY_ROBOT, position=-1)
            self.remove_unassigned_station(station)
            # Nối station đến customer robot
            self.add_customer_robot(customer, VehicleType.ROBOT_ONLY, position=-1)
            # Robot trở về station
            self.add_station_robot(station, VehicleType.ROBOT_ONLY, position=-1)

    def open_van_route(self, customer: int):
        if len(self._van_route) == 2:  # Tuyến đường van mới
            self.add_customer_van(customer, VehicleType.VAN_CARRY_ROBOT, position=-1)
            self.add_customer_robot(customer, VehicleType.VAN_CARRY_ROBOT, position=-1)

    def insert_customer_robot_into_route(self, customer: int):
        """
        Insert a customer into the robot route.
        :param customer: The customer to insert.
        """
        if len(self._robot_route) == 2:  # Tuyến đường van đã có khách nhưng robot chưa có
            station = station_nearest_customer(self.distance_matrix, self.unassigned_stations, customer)[0]
            # Nối tuyến đường hiện tại đến station mới
            self.add_station_robot(station, VehicleType.VAN_CARRY_ROBOT, position=-1)
            self.add_station_van(station, VehicleType.VAN_CARRY_ROBOT, position=-1)
            self.remove_unassigned_station(station)
            # Tạo tuyến đường robot mới
            self.add_customer_robot(customer, VehicleType.ROBOT_ONLY, position=-1)
            # Robot trở về station
            self.add_station_robot(station, VehicleType.ROBOT_ONLY, position=-1)
            return True
        else:  # Nếu tuyến đường robot đã có khách
            # Tìm vị trí chèn khách hàng sau cho không vi phạm ràng buộc: capacity
            for i in range(len(self._robot_route) - 1):
                if self._robot_route[i][0] in self.station_list:
                    # Thêm khách hàng vào tuyến đường robot mới sau station chưa có tuyến robot
                    if self._robot_route[i + 1][1] == VehicleType.VAN_CARRY_ROBOT:
                        self.add_customer_robot(customer, VehicleType.ROBOT_ONLY, position=i + 1)
                        # Robot trở về station
                        self.add_station_robot(self._robot_route[i][0], VehicleType.ROBOT_ONLY, position=i + 2)
                        return True
                    else:
                        # Thêm khách hàng vào tuyến robot có sẵn
                        from_station = self._robot_route[i][0]
                        to_station = -1
                        for j in range(i + 1, len(self._robot_route) - 1):
                            if self._robot_route[j][0] in self.station_list:
                                to_station = self._robot_route[j][0]
                                break
                        if to_station != -1 and self.get_robot_demand(from_station, to_station) + \
                                self.customer_demand[customer] <= self.robot_params['capacity']:
                            self.add_customer_robot(customer, VehicleType.ROBOT_ONLY, position=i+1)
                            return True
        return False

    def insert_customer_van_into_route(self, customer: int):
        """
        Insert a customer into the van route.
        :param customer: The customer to insert.
        """
        if len(self._van_route) == 2:
            self.open_van_route(customer)
            return True
        else:
            # Tìm vị trí chèn khách hàng sau cho không vi phạm ràng buộc: capacity
            if self.get_van_demand() + self.customer_demand[customer] <= self.van_params['capacity']:
                self.add_customer_van(customer, VehicleType.VAN_CARRY_ROBOT, position=-1)
                return True
        return False
