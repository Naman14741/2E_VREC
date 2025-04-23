from typing import Dict, Tuple

import numpy as np

from route import VehicleRoute
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
        while self.unassigned_customers:
            np.random.shuffle(self.unassigned_customers)
            first_customer = self.unassigned_customers[0]
            self.unassigned_customers.remove(first_customer)

            # Create a new route for the first customer
            route = VehicleRoute(
                distance_matrix=self.distance_matrix,
                depot=self.depot,
                unassigned_stations=self.charging_stations.copy(),
                station_list=self.charging_stations,
                customer_list=self.all_customers,
                customer_demand=self.customer_demand
            )

            if first_customer in self.customers_robot_only:
                # Customer can only be served by robot
                station = \
                station_nearest_customer(self.distance_matrix, route.get_unassigned_stations(), first_customer)[0]
                route.open_robot_route(first_customer, station)
            else:  # Customer can be served by either van or robot
                p = np.random.rand()
                if p < 0.5:
                    # Open van route
                    route.open_van_route(first_customer)
                else:
                    # Open robot route
                    station = \
                    station_nearest_customer(self.distance_matrix, route.get_unassigned_stations(), first_customer)[0]
                    route.open_robot_route(first_customer, station)

            # Create a copy of unassigned customers to iterate over
            customers_to_check = self.unassigned_customers.copy()
            np.random.shuffle(customers_to_check)

            for next_customer in customers_to_check:
                if next_customer in self.customers_robot_only:
                    if route.insert_customer_robot_into_route(next_customer):
                        self.unassigned_customers.remove(next_customer)
                else:
                    p = np.random.rand()
                    if p < 0.5:
                        if route.insert_customer_van_into_route(next_customer):
                            self.unassigned_customers.remove(next_customer)
                    else:
                        if route.insert_customer_robot_into_route(next_customer):
                            self.unassigned_customers.remove(next_customer)

            # Add the route to the solution
            self.routes.append(route)
