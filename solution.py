from typing import List, Dict, Tuple, Set

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
                 k_vehicles: int
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
        self.k_vehicles: int = k_vehicles

        self.unassigned_customers: List[int] = self.all_customers
        self.routes: Dict[int, VehicleRoute] = {}

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
            self.k_vehicles
        )

        new_solution.routes = {k: v for k, v in self.routes.items()}
        new_solution.unassigned_customers = List(self.unassigned_customers)

        return new_solution

    def objective_value(self) -> float:
        total_cost = 0
        for vehicle_id, route in self.routes.items():
            total_cost += route.get_route_cost()
        return total_cost

    def initial_routes(self):
        """
        Initialize routes for each vehicle.
        """
        for i in range(self.k_vehicles):
            self.routes[i] = VehicleRoute(self.distance_matrix, self.depot, self.van_params, self.robot_params)

    def initial_solution(self):
        self.initial_routes()

        np.random.shuffle(self.all_customers)
        for customer in self.all_customers:
            if customer in self.customers_robot_only:
                vehicle_id = np.random.randint(0, self.k_vehicles)
                if len(self.routes[vehicle_id].get_robot_route()) == 2: # Chưa chuyến nào
                    station = np.random.choice(self.charging_stations)
                    self.routes[vehicle_id].open_robot_route(customer, station)
            else:
                vehicle_id = np.random.randint(0, self.k_vehicles)
                if len(self.routes[vehicle_id].get_van_route()) == 2:
                    n = np.random.rand()
                    if n < 0.5: # Mở tuyến robot
                        station = np.random.choice(self.charging_stations)
                        self.routes[vehicle_id].open_robot_route(customer, station)
                    else: # Mở tuyến van
                        self.routes[vehicle_id].open_van_route(customer)
