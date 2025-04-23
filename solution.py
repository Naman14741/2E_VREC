from typing import Dict, Tuple

import numpy as np

from route import VehicleRoute, VehicleType
from support import *


class VRPECSolution:

    def __init__(self,
                 distance_matrix: np.ndarray,
                 depot: int,
                 charging_stations: List[int],
                 customers_robot_only: List[int],
                 customers_both: List[int],
                 van_params: Dict[str, float],
                 robot_params: Dict[str, float],
                 customer_demand: Dict[int, float],
                 time_windows: Dict[int, Tuple[float, float]],
                 service_times: Dict[int, float],
                 ):

        self.distance_matrix: np.ndarray = distance_matrix
        self.depot: int = depot
        self.charging_stations: List[int] = charging_stations

        self.customers_robot_only: List[int] = customers_robot_only
        self.customers_both: List[int] = customers_both
        self.all_customers: List[int] = customers_robot_only + customers_both

        self.van_params: Dict[str, float] = van_params
        self.robot_params: Dict[str, float] = robot_params
        self.customer_demand: Dict[int, float] = customer_demand
        self.time_windows: Dict[int, Tuple[float, float]] = time_windows
        self.service_time: Dict[int, float] = service_times

        self.unassigned_customers: List[int] = self.all_customers
        self.routes: List[VehicleRoute] = []

    def copy(self):
        new_solution = VRPECSolution(
            self.distance_matrix.copy(),
            self.depot,
            self.charging_stations.copy(),
            self.customers_robot_only.copy(),
            self.customers_both.copy(),
            self.van_params.copy(),
            self.robot_params.copy(),
            self.customer_demand.copy(),
            self.time_windows.copy(),
            self.service_time.copy(),
        )

        new_solution.routes = self.routes
        new_solution.unassigned_customers = self.unassigned_customers

        return new_solution

    def objective_value(self) -> float:
        total_cost = 0
        for route in self.routes:
            total_cost += route.get_route_cost()
        return total_cost

    def initial_solution(self):
        while len(self.unassigned_customers) > 0:
            np.random.shuffle(self.unassigned_customers)
            customer = self.unassigned_customers[0]

            # Khởi tạo tuyến cho khách hàng
            route = VehicleRoute(distance_matrix=self.distance_matrix, depot=self.depot, unassigned_stations=self.charging_stations)
            if customer in self.customers_robot_only:
                # Nếu khách hàng chỉ phục vụ bởi robot
                station = station_nearest_customer(self.distance_matrix, self.charging_stations, customer)[0]
                route.open_robot_route(customer, station)
                self.unassigned_customers.remove(customer)
            else: # Nếu khách hàng có thể phục vụ bởi cả xe tải và robot
                p = np.random.rand()
                if p < 0.5:
                    # Mở tuyến xe tải
                    route.open_van_route(customer)
                    self.unassigned_customers.remove(customer)
                else:
                    # Mở tuyến robot
                    station = station_nearest_customer(self.distance_matrix, self.charging_stations, customer)[0]
                    route.open_robot_route(customer, station)
                    self.unassigned_customers.remove(customer)

            # Thêm khách hàng vào tuyến
            np.random.shuffle(self.unassigned_customers)
