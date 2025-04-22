import numpy as np
import random
from typing import List, Tuple, Set, Optional

from solution import VRPECSolution
from route import VehicleRoute, VehicleType

class Destroy:

    def __init__(self, beta=0.3, gamma=0.3, delta=0.3, epsilon=0.3):
        """
        Args:
            beta (float): Xác suất chọn xóa khách hàng ngẫu nhiên (so với tham lam).
            gamma (float): Xác suất chọn đóng/phá hủy route ngẫu nhiên (so với tham lam cost/count).
            delta (float): Xác suất chọn phá hủy route ngẫu nhiên (so với tham lam theo số khách hàng).
                           (Thường gamma và delta có thể giống nhau nếu chỉ có 1 loại route destruction).
            epsilon (float): Xác suất chọn xóa trạm (visit) ngẫu nhiên (so với tham lam).
        """
        self.beta = beta
        self.gamma = gamma
        self.epsilon = epsilon

    def _get_assigned_customers(self, solution: VRPECSolution) -> Set[int]:
        """Lấy tập hợp các khách hàng duy nhất đang được phục vụ."""
        assigned_customers = set()
        if not hasattr(solution, 'routes') or not solution.routes: 
            return assigned_customers
        for route in solution.routes.values():
            # Use hasattr for robustness if route structure might vary
            if hasattr(route, 'get_van_route'):
                 for n, vt in route.get_van_route():
                    if n in solution.all_customers: 
                        assigned_customers.add(n)
            if hasattr(route, 'get_robot_route'):
                for n, vt in route.get_robot_route():
                    if n in solution.all_customers: 
                        assigned_customers.add(n)
        assigned_customers.discard(solution.depot)
        return assigned_customers

    def _find_route_for_customer(self, solution: VRPECSolution, customer_id: int) -> Optional[Tuple[int, VehicleRoute]]:
        """Tìm vehicle_id và route chứa khách hàng (giả định khách hàng chỉ thuộc 1 route)."""
        if not hasattr(solution, 'routes'): 
            return None
        for vid, route in solution.routes.items():
             # Check both paths for the customer
             if hasattr(route, 'get_van_route') and any(n == customer_id for n, vt in route.get_van_route()): 
                 return vid, route
             if hasattr(route, 'get_robot_route') and any(n == customer_id for n, vt in route.get_robot_route()): 
                 return vid, route
        return None 

    def _calculate_removal_cost_delta(self, solution: VRPECSolution, route: VehicleRoute, customer: int) -> float:
        
        original_cost = route.get_route_cost()
        # Create copies to simulate removal
        temp_van_route = list(route.get_van_route()) if hasattr(route, 'get_van_route') else []
        temp_robot_route = list(route.get_robot_route()) if hasattr(route, 'get_robot_route') else []
        removed = False

        # Simulate removal from copies
        indices_van = [i for i, (n, vt) in enumerate(temp_van_route) if n == customer]
        for i in sorted(indices_van, reverse=True):
            if 0 <= i < len(temp_van_route):
                del temp_van_route[i]; removed = True
        indices_robot = [i for i, (n, vt) in enumerate(temp_robot_route) if n == customer]
        for i in sorted(indices_robot, reverse=True):
             if 0 <= i < len(temp_robot_route): 
                del temp_robot_route[i]
                removed = True

        if not removed: return 0.0 # C

        # Calculate cost of the modified temporary route
        van_params=getattr(solution,'van_params',{})
        robot_params=getattr(solution,'robot_params',{})
        temp_route = VehicleRoute(solution.distance_matrix, solution.depot, van_params, robot_params)
        temp_route.set_van_route(temp_van_route)
        temp_route.set_robot_route(temp_robot_route)
        try:
            new_cost = temp_route.get_route_cost()
            return new_cost - original_cost
        except Exception as e:
             print(f"Error calculating delta cost: {e}")
             return 0.0 # Return 0 delta if cost calculation fails

    def _calculate_single_station_visit_removal_delta(
        self, solution: VRPECSolution, vehicle_id: int, path_type: str, index_to_remove: int
    ) -> Optional[float]:
        """Tính delta chi phí khi xóa 1 lần ghé thăm trạm cụ thể."""
        if vehicle_id not in solution.routes: return None

        route = solution.routes[vehicle_id]
        original_cost = route.get_route_cost()

        temp_path = []
        original_other_path = [] # The path that is NOT being modified

        # Create copies and identify paths
        try:
            if path_type == 'van' and hasattr(route, 'get_van_route'):
                temp_path = list(route.get_van_route())
                original_other_path = list(route.get_robot_route()) if hasattr(route, 'get_robot_route') else []
            elif path_type == 'robot' and hasattr(route, 'get_robot_route'):
                temp_path = list(route.get_robot_route())
                original_other_path = list(route.get_van_route()) if hasattr(route, 'get_van_route') else []
            else: return None
        except Exception as e: 
            return None

        # Simulate removal
        if 0 <= index_to_remove < len(temp_path):
            del temp_path[index_to_remove]
        else: 
            return None # Invalid index

        # Clean the modified path
        cleaned_path = self._clean_route(temp_path, solution.depot)

        # Create temporary route and calculate new cost
        van_params=getattr(solution,'van_params',{})
        robot_params=getattr(solution,'robot_params',{})
        temp_route = VehicleRoute(solution.distance_matrix, solution.depot, van_params, robot_params)
        if path_type == 'van':
            temp_route.set_van_route(cleaned_path)
            temp_route.set_robot_route(original_other_path)
        else: # robot
            temp_route.set_van_route(original_other_path)
            temp_route.set_robot_route(cleaned_path)

        try:
            new_cost = temp_route.get_route_cost()
            return new_cost - original_cost
        except Exception as e: 
            print(f"Error calculating single station visit removal delta: {e}")
            return None


    def _clean_route(self, route_tuples: List[Tuple[int, VehicleType]], depot: int) -> List[Tuple[int, VehicleType]]:
        """Loại bỏ các nút lặp lại liên tiếp và đảm bảo depot ở đầu/cuối."""
        if not route_tuples:
            return [(depot, VehicleType.VAN_CARRY_ROBOT), (depot, VehicleType.VAN_CARRY_ROBOT)]
        cleaned = []
        # Handle start node
        if route_tuples[0][0] != depot:
            cleaned.append((depot, VehicleType.VAN_CARRY_ROBOT))
            cleaned.append(route_tuples[0])
        else:
            cleaned.append(route_tuples[0])
        # Remove consecutive duplicates
        for i in range(1, len(route_tuples)):
            if route_tuples[i][0] != cleaned[-1][0]:
                cleaned.append(route_tuples[i])
        # Handle end node
        if not cleaned or cleaned[-1][0] != depot:
            cleaned.append((depot, VehicleType.VAN_CARRY_ROBOT))
        elif cleaned[-1][0] == depot and cleaned[-1][1] == VehicleType.ROBOT_ONLY:
            cleaned[-1] = (depot, VehicleType.VAN_CARRY_ROBOT) # Correct type
        # Ensure minimum length
        if len(cleaned) < 2:
            return [(depot, VehicleType.VAN_CARRY_ROBOT), (depot, VehicleType.VAN_CARRY_ROBOT)]
        return cleaned

    def _get_customers_on_route(self, solution: VRPECSolution, route: VehicleRoute) -> Set[int]:
        """Helper để lấy tập khách hàng duy nhất trên một route cụ thể."""
        customers = set()
        if hasattr(route, 'get_van_route'):
            for n, vt in route.get_van_route():
                if n in solution.all_customers: 
                    customers.add(n)
        if hasattr(route, 'get_robot_route'):
             for n, vt in route.get_robot_route():
                 if n in solution.all_customers: 
                    customers.add(n)
        return customers

    def random_customer_removal(self, solution: VRPECSolution,
                                random_state: np.random.RandomState) -> VRPECSolution:
        destroyed = solution.copy()
        assigned_customers = list(self._get_assigned_customers(destroyed))
        if not assigned_customers: 
            return destroyed

        customer_to_remove = random_state.choice(assigned_customers)
        route_info = self._find_route_for_customer(destroyed, customer_to_remove)
        if route_info is None: 
            return destroyed 

        vehicle_id, route_to_modify = route_info
        van_indices=[]; robot_indices=[]
        if hasattr(route_to_modify, 'get_van_route'): 
            van_indices = [i for i, (n, vt) in enumerate(route_to_modify.get_van_route()) if n == customer_to_remove]
        if hasattr(route_to_modify, 'get_robot_route'): 
            robot_indices = [i for i, (n, vt) in enumerate(route_to_modify.get_robot_route()) if n == customer_to_remove]
        all_instances = [('van', i) for i in van_indices] + [('robot', i) for i in robot_indices]
        if not all_instances: 
            return destroyed 

        # Choose one instance randomly
        path_type, index_to_remove = random_state.choice(all_instances)
        removed = False
        try:
            original_path = []; node_id = -1; v_type = None
            if path_type == 'van' and hasattr(route_to_modify, 'get_van_route'): 
                original_path = route_to_modify.get_van_route()
            elif path_type == 'robot' and hasattr(route_to_modify, 'get_robot_route'): 
                original_path = route_to_modify.get_robot_route()

            if original_path and 0 <= index_to_remove < len(original_path):
                    node_id, v_type = original_path[index_to_remove]
                    route_to_modify.remove_customer(node_id, v_type) # Use route's method
                    removed = True
        except (ValueError, IndexError) as e: 
            print(f"Error removing customer instance: {e}")

        # Add to unassigned only if actually removed
        if removed and customer_to_remove not in destroyed.unassigned_customers:
            destroyed.unassigned_customers.append(customer_to_remove)
        return destroyed

    def greedy_customer_removal(self, solution: VRPECSolution) -> VRPECSolution:
        """Tìm khách hàng tốt nhất (heuristic), xóa 1 lần xuất hiện ngẫu nhiên của nó."""
        destroyed = solution.copy()
        assigned_customers = list(self._get_assigned_customers(destroyed))
        if not assigned_customers: 
            return destroyed

        customer_deltas = {}
        for customer in assigned_customers:
            route_info = self._find_route_for_customer(destroyed, customer)
            if route_info:
                delta = self._calculate_removal_cost_delta(destroyed, route_info[1], customer)
                customer_deltas[customer] = delta
        if not customer_deltas: 
            return destroyed

        best_customer_to_remove = min(customer_deltas, key=customer_deltas.get)

        # Find the route and instances for the chosen customer
        route_info = self._find_route_for_customer(destroyed, best_customer_to_remove)
        if route_info is None: 
            return destroyed
        vehicle_id, route_to_modify = route_info
        van_indices = []; robot_indices = []
        if hasattr(route_to_modify, 'get_van_route'): 
            van_indices = [i for i, (n, vt) in enumerate(route_to_modify.get_van_route()) if n == best_customer_to_remove]
        if hasattr(route_to_modify, 'get_robot_route'): 
            robot_indices = [i for i, (n, vt) in enumerate(route_to_modify.get_robot_route()) if n == best_customer_to_remove]
        all_instances = [('van', i) for i in van_indices] + [('robot', i) for i in robot_indices]
        if not all_instances: 
            return destroyed

        # Choose one instance randomly to remove
        path_type, index_to_remove = random.choice(all_instances) # Use standard random here
        removed = False
        try:
            original_path = []; node_id = -1; v_type = None
            if path_type == 'van' and hasattr(route_to_modify, 'get_van_route'): 
                original_path = route_to_modify.get_van_route()
            elif path_type == 'robot' and hasattr(route_to_modify, 'get_robot_route'): 
                original_path = route_to_modify.get_robot_route()

            if original_path and 0 <= index_to_remove < len(original_path):
                    node_id, v_type = original_path[index_to_remove]
                    route_to_modify.remove_customer(node_id, v_type)
                    removed = True
        except (ValueError, IndexError) as e: 
            print(f"Error removing greedy customer instance: {e}")

        if removed and best_customer_to_remove not in destroyed.unassigned_customers:
            destroyed.unassigned_customers.append(best_customer_to_remove)
        return destroyed

    def customer_removal(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Chọn giữa xóa khách hàng ngẫu nhiên hoặc tham lam (xóa 1 instance)."""
        if random_state.rand() < self.beta:
            return self.random_customer_removal(solution, random_state)
        else:
            # Greedy removal's instance choice uses standard random, not passed state
            return self.greedy_customer_removal(solution)

    # --- Route Closure/Destruction Operators (Remove 1 entire route) ---

    def random_route_closure(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Đóng (xóa) 1 route ngẫu nhiên."""
        destroyed = solution.copy()
        active_vehicle_ids = [vid for vid, route in destroyed.routes.items()
                              if (hasattr(route,'get_van_route') and len(route.get_van_route()) > 2) or \
                                 (hasattr(route,'get_robot_route') and len(route.get_robot_route()) > 2)]
        if not active_vehicle_ids: 
            return destroyed
        vehicle_id_to_remove = random_state.choice(active_vehicle_ids)
        if vehicle_id_to_remove in destroyed.routes:
            route_to_remove = destroyed.routes[vehicle_id_to_remove]
            customers_on_route = self._get_customers_on_route(destroyed, route_to_remove)
            for customer in customers_on_route:
                if customer not in destroyed.unassigned_customers: 
                    destroyed.unassigned_customers.append(customer)
            del destroyed.routes[vehicle_id_to_remove]
        return destroyed

    def greedy_route_closure(self, solution: VRPECSolution) -> VRPECSolution:
        """Đóng (xóa) 1 route có chi phí cao nhất."""
        destroyed = solution.copy()
        active_routes = {vid: route for vid, route in destroyed.routes.items()
                         if (hasattr(route,'get_van_route') and len(route.get_van_route()) > 2) or \
                            (hasattr(route,'get_robot_route') and len(route.get_robot_route()) > 2)}
        if not active_routes: 
            return destroyed
        best_vehicle_id = -1; max_cost = -float('inf') # Initialize max_cost correctly
        for vehicle_id, route in active_routes.items():
            try: 
                cost = route.get_route_cost()
            except Exception: 
                continue
            if cost > max_cost: 
                max_cost = cost; best_vehicle_id = vehicle_id
        if best_vehicle_id != -1 and best_vehicle_id in destroyed.routes:
            route_to_remove = destroyed.routes[best_vehicle_id]
            customers_on_route = self._get_customers_on_route(destroyed, route_to_remove)
            for customer in customers_on_route:
                 if customer not in destroyed.unassigned_customers: destroyed.unassigned_customers.append(customer)
            del destroyed.routes[best_vehicle_id]
        return destroyed

    def route_closure(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Chọn giữa đóng route ngẫu nhiên hoặc tham lam (theo chi phí, đóng 1)."""
        if random_state.rand() < self.gamma:
            return self.random_route_closure(solution, random_state)
        else:
            return self.greedy_route_closure(solution)

    def random_station_removal(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Xóa 1 lần ghé thăm ngẫu nhiên đến 1 trạm ngẫu nhiên khỏi 1 route ngẫu nhiên."""
        destroyed = solution.copy()
        all_station_visits = [] # List of (vehicle_id, route_obj, path_type, index, station_id)

        if not hasattr(solution, 'routes') or not solution.routes: return destroyed
        for vid, route in destroyed.routes.items():
            if hasattr(route, 'get_van_route'):
                for idx, (n, vt) in enumerate(route.get_van_route()):
                    if n in destroyed.charging_stations: all_station_visits.append((vid, route, 'van', idx, n))
            if hasattr(route, 'get_robot_route'):
                for idx, (n, vt) in enumerate(route.get_robot_route()):
                     if n in destroyed.charging_stations: all_station_visits.append((vid, route, 'robot', idx, n))

        if not all_station_visits: return destroyed

        # Choose one visit randomly
        chosen_visit_idx = random_state.choice(len(all_station_visits))
        vehicle_id, route_to_modify, path_type, index_to_remove, station_id = all_station_visits[chosen_visit_idx]

        # Remove the chosen visit by index and clean
        removed = False; temp_path = []
        try:
            if path_type == 'van' and hasattr(route_to_modify, 'get_van_route'):
                temp_path = list(route_to_modify.get_van_route())
                if 0 <= index_to_remove < len(temp_path): del temp_path[index_to_remove]; removed = True
                cleaned_path = self._clean_route(temp_path, destroyed.depot)
                route_to_modify.set_van_route(cleaned_path)
            elif path_type == 'robot' and hasattr(route_to_modify, 'get_robot_route'):
                 temp_path = list(route_to_modify.get_robot_route())
                 if 0 <= index_to_remove < len(temp_path): del temp_path[index_to_remove]; removed = True
                 cleaned_path = self._clean_route(temp_path, destroyed.depot)
                 route_to_modify.set_robot_route(cleaned_path)
        except IndexError as e: print(f"Error removing random station visit: {e}")

        return destroyed

    def greedy_station_removal(self, solution: VRPECSolution) -> VRPECSolution:
        """Xóa 1 lần ghé thăm trạm mang lại lợi ích chi phí lớn nhất."""
        destroyed = solution.copy()
        all_station_visits = [] # List of (vid, path_type, index, station_id)
        visit_deltas = {}       # Dict mapping index in all_station_visits to delta cost

        # Find all station visits and calculate their removal delta
        if not hasattr(solution, 'routes') or not solution.routes: return destroyed
        current_visit_list_idx = 0
        for vid, route in destroyed.routes.items():
            if hasattr(route, 'get_van_route'):
                for idx, (n, vt) in enumerate(route.get_van_route()):
                    if n in destroyed.charging_stations:
                        visit_info = (vid, 'van', idx, n)
                        all_station_visits.append(visit_info)
                        delta = self._calculate_single_station_visit_removal_delta(destroyed, vid, 'van', idx)
                        if delta is not None: visit_deltas[current_visit_list_idx] = delta
                        current_visit_list_idx += 1
            if hasattr(route, 'get_robot_route'):
                for idx, (n, vt) in enumerate(route.get_robot_route()):
                     if n in destroyed.charging_stations:
                         visit_info = (vid, 'robot', idx, n)
                         all_station_visits.append(visit_info)
                         delta = self._calculate_single_station_visit_removal_delta(destroyed, vid, 'robot', idx)
                         if delta is not None: visit_deltas[current_visit_list_idx] = delta
                         current_visit_list_idx += 1

        if not visit_deltas: return destroyed # No valid visits or all deltas failed

        # Find the visit with the minimum delta cost (best saving or least increase)
        best_visit_list_idx = min(visit_deltas, key=visit_deltas.get)
        vehicle_id, path_type, index_to_remove, station_id = all_station_visits[best_visit_list_idx]

        # Perform the removal for the best visit found
        if vehicle_id in destroyed.routes:
            route_to_modify = destroyed.routes[vehicle_id]
            removed = False; temp_path = []
            try:
                if path_type == 'van' and hasattr(route_to_modify, 'get_van_route'):
                    temp_path = list(route_to_modify.get_van_route())
                    if 0 <= index_to_remove < len(temp_path): del temp_path[index_to_remove]; removed = True
                    cleaned_path = self._clean_route(temp_path, destroyed.depot)
                    route_to_modify.set_van_route(cleaned_path)
                elif path_type == 'robot' and hasattr(route_to_modify, 'get_robot_route'):
                     temp_path = list(route_to_modify.get_robot_route())
                     if 0 <= index_to_remove < len(temp_path): del temp_path[index_to_remove]; removed = True
                     cleaned_path = self._clean_route(temp_path, destroyed.depot)
                     route_to_modify.set_robot_route(cleaned_path)
            except IndexError as e: print(f"Error removing greedy station visit: {e}")

        return destroyed

    def station_removal(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Chọn giữa xóa trạm (visit) ngẫu nhiên hoặc tham lam."""
        if random_state.rand() < self.epsilon:
            return self.random_station_removal(solution, random_state)
        else:
            return self.greedy_station_removal(solution)

    def route_destruction_greedy(self, solution: VRPECSolution) -> VRPECSolution:
        """Phá hủy (xóa) 1 route có số lượng khách hàng ít nhất."""
        destroyed = solution.copy()
        active_routes = {vid: route for vid, route in destroyed.routes.items()
                         if (hasattr(route,'get_van_route') and len(route.get_van_route()) > 2) or \
                            (hasattr(route,'get_robot_route') and len(route.get_robot_route()) > 2)}
        if not active_routes:
            return destroyed
        best_vehicle_id = -1
        min_customer_count = float('inf')
        for vehicle_id, route in active_routes.items():
            count = len(self._get_customers_on_route(destroyed, route))
            if count < min_customer_count:
                min_customer_count = count
                best_vehicle_id = vehicle_id
        if best_vehicle_id != -1 and best_vehicle_id in destroyed.routes:
            route_to_remove = destroyed.routes[best_vehicle_id]
            customers_to_unassign = self._get_customers_on_route(destroyed, route_to_remove)
            for customer in customers_to_unassign:
                if customer not in destroyed.unassigned_customers:
                    destroyed.unassigned_customers.append(customer)
            del destroyed.routes[best_vehicle_id]
        return destroyed
