import random
import numpy as np
from alns import ALNS, State
from alns.accept import HillClimbing

# Dữ liệu đầu vào
distance_matrix = [
    [0, 50, 80, 80, 50, 80, 80, 82, 84, 86],  # depot 0
    [50, 0, 0, 80, 80, 80, 50, 50, 90, 90],  # station 1
    [80, 70, 0, 50, 80, 80, 80, 80, 80, 80],  # station 2
    [80, 80, 50, 0, 50, 80, 50, 50, 80, 80],  # station 3
    [50, 80, 80, 50, 0, 70, 90, 80, 30, 30],  # station 4
    [80, 80, 80, 80, 70, 0, 80, 80, 80, 80],  # station 5
    [80, 50, 80, 50, 90, 80, 0, 80, 90, 90],  # customer 6
    [82, 50, 80, 50, 80, 80, 80, 0, 90, 90],  # customer 7
    [84, 90, 80, 80, 30, 80, 90, 90, 0, 30],  # customer 8
    [86, 90, 80, 80, 30, 80, 90, 90, 30, 0]  # customer 9
]

nodes = list(range(len(distance_matrix)))
depot = 0
charging_stations = [1, 2, 3, 4, 5]
customers_robot_only = [6, 8, 9]
customers_both = [7]
all_customers = customers_robot_only + customers_both

van_params = {
    "speed": 2.0,
    "battery_capacity": 400.0,
    "capacity": 200.0,
    "travel_cost_rate": 2.0,
    "energy_consumption_rate": 2.0,
    "charge_rate": 10.0
}

robot_params = {
    "speed": 1.0,
    "battery_capacity": 120.0,
    "capacity": 50.0,
    "travel_cost_rate": 1.0,
    "energy_consumption_rate": 1.0,
    "charge_rate": 4.0
}


def initialize_solution():
    van_route = [depot] + random.sample(customers_both, len(customers_both)) + [depot]
    robot_route = random.sample(customers_robot_only, len(customers_robot_only))
    return van_route, robot_route


def compute_cost(van_route, robot_route):
    if not van_route or not robot_route:
        return float("inf")

    total_cost = 0
    battery_van = van_params["battery_capacity"]
    battery_robot = robot_params["battery_capacity"]

    for i in range(len(van_route) - 1):
        d = distance_matrix[van_route[i]][van_route[i + 1]]
        battery_van -= d * van_params["energy_consumption_rate"]
        total_cost += d * van_params["travel_cost_rate"]
        if battery_van <= 0:
            return float("inf")

    for i in range(len(robot_route) - 1):
        d = distance_matrix[robot_route[i]][robot_route[i + 1]]
        battery_robot -= d * robot_params["energy_consumption_rate"]
        total_cost += d * robot_params["travel_cost_rate"]
        if battery_robot <= 0:
            return float("inf")

    return total_cost


class RoutingState(State):
    def __init__(self, van_route, robot_route):
        self.van_route = van_route
        self.robot_route = robot_route

    def copy(self):
        return RoutingState(self.van_route[:], self.robot_route[:])


def destroy_solution(state, removal_rate=0.3):
    if not state.van_route or not state.robot_route:
        return state, set()

    van_route = state.van_route[:]
    robot_route = state.robot_route[:]
    num_remove_van = max(1, int(len(van_route) * removal_rate))
    num_remove_robot = max(1, int(len(robot_route) * removal_rate))
    removed_van = set(random.sample(van_route[1:-1], num_remove_van)) if len(van_route) > 2 else set()
    removed_robot = set(random.sample(robot_route, num_remove_robot)) if robot_route else set()

    return RoutingState([node for node in van_route if node not in removed_van],
                        [node for node in robot_route if node not in removed_robot]), removed_van.union(removed_robot)


def repair_solution(state, removed_customers):
    new_van_route = state.van_route[:]
    new_robot_route = state.robot_route[:]

    for customer in removed_customers:
        best_pos = None
        best_cost = float("inf")
        target_route = new_van_route if customer in customers_both else new_robot_route

        for i in range(1, len(target_route)):
            temp_route = target_route[:i] + [customer] + target_route[i:]
            cost = compute_cost(new_van_route, new_robot_route)
            if cost < best_cost:
                best_cost = cost
                best_pos = i
        if best_pos is not None:
            target_route.insert(best_pos, customer)
    return RoutingState(new_van_route, new_robot_route)


def alns_solver(iterations=1000):
    initial_van_route, initial_robot_route = initialize_solution()
    initial_solution = RoutingState(initial_van_route, initial_robot_route)

    alns = ALNS()
    alns.add_destroy_operator(lambda state: destroy_solution(state))
    alns.add_repair_operator(lambda state, removed: repair_solution(state, removed))

    criterion = HillClimbing()
    result = alns.iterate(initial_solution, lambda s: compute_cost(s.van_route, s.robot_route), criterion, iterations)

    best_van_route = result.best_state.van_route if result.best_state else []
    best_robot_route = result.best_state.robot_route if result.best_state else []
    best_cost = compute_cost(best_van_route, best_robot_route) if result.best_state else float("inf")

    return best_van_route, best_robot_route, best_cost


# Chạy thuật toán ALNS
best_van_route, best_robot_route, best_cost = alns_solver()
print("Best van route:", best_van_route)
print("Best robot route:", best_robot_route)
print("Best cost:", best_cost)