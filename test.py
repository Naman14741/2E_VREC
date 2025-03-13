import copy
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd
import re

from alns import ALNS
from alns.accept import RecordToRecordTravel
from alns.select import RouletteWheel
from alns.stop import MaxIterations


def read_2evrp_data(file_path):
    """
    Read and parse 2E-VRP data from the specified file.
    """
    with open(file_path, 'r') as file:
        lines = file.readlines()

    data = {}
    section = None

    for line in lines:
        line = line.strip()
        if line.startswith("SATELLITES:"):
            satellites = int(line.split(":")[1].strip())
            data["SATELLITES"] = list(range(1, satellites + 1))
        elif line.startswith("CUSTOMERS:"):
            customers = int(line.split(":")[1].strip())
            data['CUSTOMERS'] = list(range(1, customers + 1))
            data["END"] = customers + 1
        elif line.startswith("L1CAPACITY:"):
            data["L1_CAPACITY"] = int(line.split(":")[1].strip())
        elif line.startswith("L2CAPACITY:"):
            data["L2_CAPACITY"] = int(line.split(":")[1].strip())
        elif line.startswith("L1FLEET:"):
            data["L1_FLEET"] = int(line.split(":")[1].strip())
        elif line.startswith("L2FLEET:"):
            data["L2_FLEET"] = int(line.split(":")[1].strip())
        elif line == "NODE_COORD_SECTION":
            section = "NODE_COORDS"
            data[section] = {}
        elif line == "DEMAND_SECTION":
            section = "DEMANDS"
            data[section] = {}
        elif line == "DEPOT_SECTION":
            section = None
        elif line == "EOF":
            break
        elif section:
            parts = list(map(int, re.findall(r'\d+', line)))
            if len(parts) < 2:
                continue
            node_id = parts[0]
            values = tuple(parts[1:]) if section == "NODE_COORDS" or section == "SATELLITES" else parts[1]
            data[section][node_id] = values

    data["DEPOT"] = 0
    data['NODE_COORDS'][data['END']] = data['NODE_COORDS'][0]
    data["DEMANDS"][data["END"]] = 0

    return data


class TwoEVRPState:
    """
    Solution state for the Two-Echelon Vehicle Routing Problem.

    first_echelon: dict of first-level trucks, where each route is:
       [DEPOT, s, ... (customers served by L1) ..., s, END]
       (s appears twice: for drop-off/pickup of L2 vehicles)
    second_echelon: dict of second-level vehicles by satellite, each route:
       [s] + list of customers + [s]
    unassigned: list of customers not yet assigned.
    """

    def __init__(self, first_echelon, second_echelon, unassigned=None):
        self.first_echelon = first_echelon
        self.second_echelon = second_echelon
        self.unassigned = unassigned if unassigned is not None else []

    def copy(self):
        return TwoEVRPState(copy.deepcopy(self.first_echelon),
                            copy.deepcopy(self.second_echelon),
                            self.unassigned.copy())

    def objective(self):
        # Calculate total distance for L1 and L2 vehicles
        cost_L1 = 0
        for route in self.first_echelon.values():
            cost_L1 += sum(euclidean_distance(route[i], route[i + 1]) for i in range(len(route) - 1))
        cost_L2 = 0
        for s, routes in self.second_echelon.items():
            for route in routes:
                if route:
                    cost_L2 += euclidean_distance(s, route[0])
                    cost_L2 += sum(euclidean_distance(route[i], route[i + 1]) for i in range(len(route) - 1))
                    cost_L2 += euclidean_distance(route[-1], s)
        penalty = 1e6 * len(self.unassigned)
        return cost_L1 + cost_L2 + penalty


def euclidean_distance(a, b):
    """Calculate Euclidean distance between two nodes."""
    x1, y1 = NODE_COORDS[a]
    x2, y2 = NODE_COORDS[b]
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def initial_solution():
    """
    Create an initial solution for the 2E-VRP problem.

    - L1 vehicles: Assign each vehicle to a satellite (round-robin).
      Initialize L1 route with structure: [DEPOT, s, s, END]
    - L2 vehicles: Each satellite has an empty route initially; add more if needed.
    - Customers are primarily assigned to L2; add to unassigned if not possible.
    """
    first_echelon = {}
    for veh in range(L1_FLEET):
        assigned_sat = SATELLITES[veh % len(SATELLITES)]
        first_echelon[veh] = [DEPOT, assigned_sat, assigned_sat, END]

    second_echelon = {s: [[]] for s in SATELLITES}
    route_cap = {s: [L2_CAPACITY] for s in SATELLITES}
    unassigned = []

    sorted_customers = sorted(CUSTOMERS, key=lambda c: min(euclidean_distance(c, s) for s in SATELLITES))
    for c in sorted_customers:
        s_best = min(SATELLITES, key=lambda s: euclidean_distance(s, c))
        assigned = False
        for idx, cap in enumerate(route_cap[s_best]):
            if cap >= DEMANDS[c]:
                second_echelon[s_best][idx].append(c)
                route_cap[s_best][idx] -= DEMANDS[c]
                assigned = True
                break
        if not assigned:
            if len(second_echelon[s_best]) < max_routes_sat[s_best]:
                second_echelon[s_best].append([c])
                route_cap[s_best].append(L2_CAPACITY - DEMANDS[c])
                assigned = True
            else:
                unassigned.append(c)

    return TwoEVRPState(first_echelon, second_echelon, unassigned)


def random_removal(state, rng, num_remove=3):
    """
    Destroy operator: Randomly remove customers from L1 and L2 routes.
    Only removes nodes that are CUSTOMERS (not DEPOT, END, or Satellites).
    """
    state = state.copy()
    customers_to_remove = rng.choice(CUSTOMERS, min(num_remove, len(CUSTOMERS)), replace=False)

    # L1: Remove customers from routes, keeping fixed points (DEPOT, END, Satellite)
    for veh, route in state.first_echelon.items():
        new_route = [node for node in route if
                     (node not in customers_to_remove) or (node in [DEPOT, END] or node in SATELLITES)]
        state.first_echelon[veh] = new_route

    # L2: Remove from routes
    for s, routes in state.second_echelon.items():
        for r in routes:
            r[:] = [c for c in r if c not in customers_to_remove]

    state.unassigned.extend(customers_to_remove)
    return state


def used_capacity_L1(route):
    """Calculate used capacity for an L1 route."""
    return sum(DEMANDS[c] for c in route if c in CUSTOMERS)


def greedy_repair(state, rng):
    """
    Repair operator: For each unassigned customer, consider 2 options:

    Option 1: Insert into L1 route (truck) between first and second satellite visits.
              Check all valid positions between index 2 and index (len(route)-2).
              Check capacity and calculate extra cost.
    Option 2: Insert into L2 route of the nearest satellite.

    Then insert the customer at the position with the smallest extra cost.
    """
    state = state.copy()
    rng.shuffle(state.unassigned)

    for c in state.unassigned[:]:  # Use a copy of the list to safely remove items
        best_option = None
        best_extra_cost = float('inf')

        # Option 1: Insert into L1
        for veh, route in state.first_echelon.items():
            # Assume route format: [DEPOT, s, ... , s, END]
            # Valid positions are from index 2 to index = len(route)-2
            s = route[1]  # satellite assigned to this vehicle
            for pos in range(2, len(route) - 1):
                prev_node = route[pos - 1]
                next_node = route[pos]
                extra_cost = euclidean_distance(prev_node, c) + euclidean_distance(c, next_node) - euclidean_distance(
                    prev_node, next_node)
                if used_capacity_L1(route) + DEMANDS[c] <= L1_CAPACITY:
                    if extra_cost < best_extra_cost:
                        best_extra_cost = extra_cost
                        best_option = ('L1', veh, pos)

        # Option 2: Insert into L2 (city vehicles)
        for s in SATELLITES:
            for idx, route2 in enumerate(state.second_echelon[s]):
                # Calculate current route load
                current_load = sum(DEMANDS[customer] for customer in route2)

                # Check if adding this customer would exceed capacity
                if current_load + DEMANDS[c] > L2_CAPACITY:
                    continue

                if len(route2) == 0:
                    extra = euclidean_distance(s, c) + euclidean_distance(c, s)
                    if extra < best_extra_cost:
                        best_extra_cost = extra
                        best_option = ('L2', s, idx, 0)
                else:
                    # Try all possible insertion positions in the route
                    for pos in range(len(route2) + 1):
                        if pos == 0:
                            # Insert at the beginning
                            prev_node = s
                            next_node = route2[0] if route2 else s
                        elif pos == len(route2):
                            # Insert at the end
                            prev_node = route2[-1]
                            next_node = s
                        else:
                            # Insert in the middle
                            prev_node = route2[pos - 1]
                            next_node = route2[pos]

                        extra = euclidean_distance(prev_node, c) + euclidean_distance(c,
                                                                                      next_node) - euclidean_distance(
                            prev_node, next_node)
                        if extra < best_extra_cost:
                            best_extra_cost = extra
                            best_option = ('L2', s, idx, pos)

        if best_option is None:
            # If no feasible option (rare), create a new route at first satellite
            s = SATELLITES[0]
            if len(state.second_echelon[s]) < max_routes_sat[s]:
                state.second_echelon[s].append([c])
                state.unassigned.remove(c)
            # If even this fails, leave customer unassigned
        else:
            if best_option[0] == 'L1':
                _, veh, pos = best_option
                state.first_echelon[veh].insert(pos, c)
                state.unassigned.remove(c)
            else:
                _, s, idx, pos = best_option
                state.second_echelon[s][idx].insert(pos, c)
                state.unassigned.remove(c)

    return state


def plot_solution(best):
    """
    Visualize the 2E-VRP solution showing both first and second echelon routes.
    """
    plt.figure(figsize=(10, 6))

    # Plot nodes: Depot, End, Satellites, Customers
    plt.scatter(NODE_COORDS[DEPOT][0], NODE_COORDS[DEPOT][1], color='red', s=100, label='Depot')
    plt.scatter(NODE_COORDS[END][0], NODE_COORDS[END][1], color='black', s=100, marker='D', label='End')
    for idx, s in enumerate(SATELLITES):
        plt.scatter(NODE_COORDS[s][0], NODE_COORDS[s][1], color='blue', s=100, label='Satellite' if idx == 0 else "")
    for idx, c in enumerate(CUSTOMERS):
        plt.scatter(NODE_COORDS[c][0], NODE_COORDS[c][1], color='green', s=50, label='Customer' if idx == 0 else "")

    # Plot L1 routes (trucks)
    colors_L1 = ['purple', 'orange', 'magenta', 'cyan']
    for veh, route in best.first_echelon.items():
        color = colors_L1[veh % len(colors_L1)]
        xs = [NODE_COORDS[n][0] for n in route]
        ys = [NODE_COORDS[n][1] for n in route]
        plt.plot(xs, ys, color=color, marker='o', label=f'L1 Truck {veh}')

    # Plot L2 routes (city vehicles)
    colors_L2 = ['gray', 'brown', 'yellow', 'pink']
    color_idx = 0
    for s, routes in best.second_echelon.items():
        for i, route in enumerate(routes):
            if route:  # Only plot non-empty routes
                full_route = [s] + route + [s]
                color = colors_L2[color_idx % len(colors_L2)]
                xs = [NODE_COORDS[n][0] for n in full_route]
                ys = [NODE_COORDS[n][1] for n in full_route]
                plt.plot(xs, ys, linestyle='--', marker='x', color=color,
                         label=f'L2 Sat {s} Veh {i}' if i == 0 else "")
                color_idx += 1

    # Annotate all nodes with their ID and demand
    for node, coord in NODE_COORDS.items():
        x, y = coord
        plt.text(x + 1, y + 1, f"{node}:{DEMANDS[node]}", fontsize=8, color='black')

    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.title("Two-Echelon VRP: L1 (Depot -> Satellite -> [Customers] -> Satellite -> End) & L2 Routes")
    plt.legend()
    plt.tight_layout()
    return plt  # Return the plot object for flexibility


def run_alns(file_path='dataset/Set2/E-n22-k4-s06-17.dat', seed=1234, max_iterations=1000):
    """
    Run the ALNS algorithm on the given problem instance.

    Args:
        file_path: Path to the data file
        seed: Random seed for reproducibility
        max_iterations: Maximum number of iterations

    Returns:
        best_state: The best solution found
    """
    global DEPOT, END, CUSTOMERS, L1_CAPACITY, L2_CAPACITY, L1_FLEET, L2_FLEET, SATELLITES, NODE_COORDS, DEMANDS, max_routes_sat

    # Load data
    data = read_2evrp_data(file_path)

    # Set global variables
    DEPOT = data['DEPOT']
    END = data['END']
    CUSTOMERS = data['CUSTOMERS']
    L1_CAPACITY = data['L1_CAPACITY']
    L2_CAPACITY = data['L2_CAPACITY']
    L1_FLEET = data['L1_FLEET']
    L2_FLEET = data['L2_FLEET']
    SATELLITES = data['SATELLITES']
    NODE_COORDS = data['NODE_COORDS']
    DEMANDS = data['DEMANDS']

    # Maximum number of L2 routes per satellite (distribute evenly)
    max_routes_sat = {s: L2_FLEET // len(SATELLITES) for s in SATELLITES}

    # Set random seed for reproducibility
    rng = rnd.default_rng(seed)

    # Create initial solution
    init_state = initial_solution()

    # Setup ALNS
    alns = ALNS(rng)
    alns.add_destroy_operator(random_removal)
    alns.add_repair_operator(greedy_repair)

    # Configure acceptance, selection and stopping criteria
    accept = RecordToRecordTravel.autofit(init_state.objective(), 0.02, 0, 10000)
    select = RouletteWheel([25, 5, 1, 0], 0.8, 1, 1)
    stop = MaxIterations(max_iterations)

    # Run ALNS algorithm
    result = alns.iterate(init_state, select, accept, stop)
    best = result.best_state

    # Print solution details
    print("Best solution cost:", best.objective())
    print("\n--- L1 Routes (Trucks):")
    for veh, route in best.first_echelon.items():
        print(f"Truck {veh}: {route}")
    print("\n--- L2 Routes (Vehicles at Satellite):")
    for s, routes in best.second_echelon.items():
        for idx, r in enumerate(routes):
            if r:  # Only print non-empty routes
                print(f"Satellite {s}, Vehicle {idx + 1}: {[s] + r + [s]}")

    # Visualize the solution
    plt = plot_solution(best)
    plt.show()

    return best


# Main execution
if __name__ == "__main__":
    try:
        run_alns()
    except FileNotFoundError:
        print("Data file not found. Please check the file path.")
        print("Default path is 'dataset/Set2/E-n22-k4-s06-17.dat'")
        file_path = input("Enter the correct file path: ")
        if file_path.strip():
            run_alns(file_path)