import numpy as np
from solution import VRPECSolution

class Destroy:
    """
    Input is a feasible solution
    :return: A solution unfeasible to repair operator for a better solution
    """

    def __init__(self, beta=0.3):
        # Customer destroy rate
        self.beta = beta

    def random_customer_removal(self, solution: VRPECSolution,
                            random_state: np.random.RandomState) -> VRPECSolution:
    """
    Remove a random customer from the solution.
    """
    destroyed = solution.copy()
    assigned_customers = [c for c in destroyed.all_customers
                          if c not in destroyed.unassigned_customers]

    if not assigned_customers:
        return destroyed  # No customers to remove

    customer_to_remove = random_state.choice(assigned_customers)

    for route in destroyed.van_routes:
        if customer_to_remove in route.nodes:
            idx = route.nodes.index(customer_to_remove)
            route.nodes.pop(idx)
            if idx < len(route.robot_onboard):
                route.robot_onboard.pop(idx)

    for route in destroyed.robot_routes:
        if customer_to_remove in route.nodes:
            route.nodes.remove(customer_to_remove)

    destroyed.unassigned_customers.add(customer_to_remove)
    return destroyed

    def greedy_customer_removal(self, solution: VRPECSolution) -> VRPECSolution:
        """
        Remove the customer that causes the least increase in cost when removed.
        """
        destroyed = solution.copy()
        assigned_customers = [c for c in destroyed.all_customers
                              if c not in destroyed.unassigned_customers]
    
        if not assigned_customers:
            return destroyed  # No customers to remove
    
        best_customer = None
        best_cost_increase = float('inf')
    
        for customer in assigned_customers:
            cost_increase = 0.0
    
            for route in destroyed.van_routes:
                if customer in route.nodes:
                    idx = route.nodes.index(customer)
                    prev_node = route.nodes[idx - 1]
                    next_node = route.nodes[idx + 1] if idx < len(route.nodes) - 1 else route.nodes[0]
                    cost_increase += (destroyed.distance_matrix[prev_node][next_node] -
                                      destroyed.distance_matrix[prev_node][customer] -
                                      destroyed.distance_matrix[customer][next_node])* destroyed.van_params["travel_cost_rate"]/ destroyed.van_params["speed"]
    
            for route in destroyed.robot_routes:
                if customer in route.nodes:
                    idx = route.nodes.index(customer)
                    prev_node = route.nodes[idx - 1]
                    next_node = route.nodes[idx + 1] if idx < len(route.nodes) - 1 else route.nodes[0]
                    cost_increase += (destroyed.distance_matrix[prev_node][next_node] -
                                      destroyed.distance_matrix[prev_node][customer] -
                                      destroyed.distance_matrix[customer][next_node]) * destroyed.robot_params["travel_cost_rate"] / destroyed.robot_params["speed"]
    
            if cost_increase < best_cost_increase:
                best_cost_increase = cost_increase
                best_customer = customer
    
        for route in destroyed.van_routes:
            if best_customer in route.nodes:
                idx = route.nodes.index(best_customer)
                route.nodes.pop(idx)
                if idx < len(route.robot_onboard):
                    route.robot_onboard.pop(idx)
    
        for route in destroyed.robot_routes:
            if best_customer in route.nodes:
                route.nodes.remove(best_customer)
    
        destroyed.unassigned_customers.add(best_customer)
        return destroyed
    def customer_removal(solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        rand_num = np.random.rand()
        if rand_num < self.beta:
            destroyed = random_customer_removal(solution, random_state)
        else:
            destroyed = greedy_customer_removal(solution)
        return destroyed
    
    def station_removal(self, solution: VRPECSolution,
                        random_state: np.random.RandomState) -> VRPECSolution:
        """
        Remove a random charging station and its associated robot routes.
        """
        destroyed = solution.copy()
        used_stations = set()
        for route in destroyed.van_routes:
            for node in route.nodes:
                if node in destroyed.charging_stations:
                    used_stations.add(node)
    
        if not used_stations:
            return destroyed  # No stations to remove
    
        station_to_remove = random_state.choice(list(used_stations))
    
        routes_to_remove = []
        for i, route in enumerate(destroyed.robot_routes):
            if station_to_remove in route.nodes:
                routes_to_remove.append(i)
                for node in route.nodes:
                    if node in destroyed.all_customers:
                        destroyed.unassigned_customers.add(node)
    
        for i in sorted(routes_to_remove, reverse=True):
            destroyed.robot_routes.pop(i)
    
        for route in destroyed.van_routes:
            if station_to_remove in route.nodes:
                idx = route.nodes.index(station_to_remove)
                if 0 < idx < len(route.nodes) - 1:
                    route.nodes.pop(idx)
                    if idx < len(route.robot_onboard):
                        route.robot_onboard.pop(idx)
    
        return destroyed
    
    def random_route_closure(self, solution: VRPECSolution,
                             random_state: np.random.RandomState) -> VRPECSolution:
        """
        remove all customers from a random route (van or robot) and mark them as unassigned.
        """
        destroyed = solution.copy()
        if not destroyed.van_routes and not destroyed.robot_routes:
            return destroyed  # No routes to destroy
        
        route_to_closure = []
        for route in destroyed.van_routes + destroyed.robot_routes:
            if route.nodes:
                route_to_closure.append(route)
    
        route_removed = random_state.choice(route_to_closure)
        for node in route_removed.nodes:
            if node in destroyed.all_customers:
                destroyed.unassigned_customers.add(node)
    
        if route_removed.is_van_route:
            destroyed.van_routes.remove(route_removed)
        else:
            destroyed.robot_routes.remove(route_removed)
        
        return destroyed
    
    def greedy_route_closure(self, solution: VRPECSolution) -> VRPECSolution:
        """
        Remove the route that causes the least increase in cost when removed.
        """
        destroyed = solution.copy()
    
        if not destroyed.van_routes and not destroyed.robot_routes:
            return destroyed
        
        candidate_routes = destroyed.van_routes + destroyed.robot_routes
        best_route = None
        best_cost = float('inf')
        
        for route in candidate_routes:
            if route in destroyed.van_routes:
                cost_rate = destroyed.van_params["travel_cost_rate"]
                speed = destroyed.van_params["speed"]
            else:
                cost_rate = destroyed.robot_params["travel_cost_rate"]
                speed = destroyed.robot_params["speed"]
    
            cost = 0.0
            for i in range(len(route.nodes) - 1):
                node1, node2 = route.nodes[i], route.nodes[i + 1]
                distance = destroyed.distance_matrix[node1][node2]
    
            cost += (cost_rate * distance / speed)
    
            if cost > best_cost:
                best_cost = cost
                best_route = route
    
        if best_route:
            for node in best_route.nodes:
                if node in destroyed.all_customers:
                    destroyed.unassigned_customers.add(node)
    
            if best_route.is_van_route:
                destroyed.van_routes.remove(best_route)
            else:
                destroyed.robot_routes.remove(best_route)
    
        return destroyed
    
    def route_closure(self, solution: VRPECSolution, random_state: np.random.RandomState -> VRPECSolution:
        rand_num = np.random.rand()
        if rand_num < self.beta:
            destroyed = random_route_closure(solution, random_state)
        else:
            destroyed = greedy_route_closure(solution)
        return destroyed
    
    
    def route_destruction(self, solution: VRPECSolution,
                          random_state: np.random.RandomState) -> VRPECSolution:
        """
        Destroy a random van route and its associated robot routes.
        """
        destroyed = solution.copy()
        if not destroyed.van_routes:
            return destroyed  # Nocustomer_demand routes to destroy
    
        van_route_idx = random_state.randint(0, len(destroyed.van_routes))
    
        if van_route_idx < len(destroyed.van_routes):
            van_route = destroyed.van_routes[van_route_idx]
    
            for node in van_route.nodes:
                if node in destroyed.all_customers:
                    destroyed.unassigned_customers.add(node)
    
            stations_in_route = [node for node in van_route.nodes
                                 if node in destroyed.charging_stations]
    
            routes_to_remove = []
            for i, route in enumerate(destroyed.robot_routes):
                if any(station in route.nodes for station in stations_in_route):
                    routes_to_remove.append(i)
                    for node in route.nodes:
                        if node in destroyed.all_customers:
                            destroyed.unassigned_customers.add(node)
    
            for i in sorted(routes_to_remove, reverse=True):
                destroyed.robot_routes.pop(i)
    
            destroyed.van_routes.pop(van_route_idx)
    
        return destroye


    def station_removal_redundant(self):
        """
        Redundant charging station removal removes a redundant charging station if one exists.
        """
        pass

    def route_closure_random(self):
        """
        Random van/robot route closure removes all customers from a randomly chosen van/robot route.
        """
        pass

    def route_closure_greedy(self):
        """
        Greedy van/robot route closure removes the route that can yield the largest cost reduction.
        """
        pass

    def route_destruction_random(self):
        """
        Random route destruction randomly chooses a 2E-VREC route in the solution to destroy.
        """
        pass

    def route_destruction_greedy(self):
        """
        Greedy route destruction chooses a 2E-VREC route, with a minimum number of customer nodes to destroy.
        """
        pass
