from typing import Optional, Tuple

from solution import VRPECSolution
from route import VehicleRoute, VehicleType


class HeuristicFeasible:
    
    def __init__(self, solution: VRPECSolution):
        self.solution: VRPECSolution = solution

    def calculate_charge_time(self, energy_needed: float, rate: float) -> float:
        """Calculate time needed to charge a specific amount of energy."""
        if rate <= 0:
            return float('inf') if energy_needed > 1e-9 else 0.0
        return energy_needed / rate if energy_needed > 1e-9 else 0.0

    def get_next_parking_station(self, route: VehicleRoute, current_van_node_idx: int) -> Optional[int]:
        """Find the next parking station/depot node in the van path."""
        van_path = route.get_van_route()
        for i in range(current_van_node_idx + 1, len(van_path)):
            node_id, _ = van_path[i]
            if node_id in self.solution.charging_stations or node_id == self.solution.depot:
                return node_id
        if van_path and van_path[-1][0] == self.solution.depot:
            return self.solution.depot
        return None

    def get_robot_independent_energy_needs(self, route: VehicleRoute, start_node_in_robot_route: int) -> Tuple[float, float]:
        """Estimate energy needs for a robot's independent trip (ROBOT_ONLY)."""
        robot_path = route.get_robot_route()
        low_level_energy = 0.0
        high_level_energy = 0.0
        
        if len(robot_path) < 2:
            return 0.0, 0.0
            
        in_independent_trip = False
        first_segment_found = False
        start_idx_in_robot_path = -1
        
        for idx, (node_id, v_type) in enumerate(robot_path):
            if (node_id == start_node_in_robot_route and 
                idx + 1 < len(robot_path) and 
                robot_path[idx+1][1] == VehicleType.ROBOT_ONLY):
                start_idx_in_robot_path = idx
                break
                
        if start_idx_in_robot_path == -1:
            return 0.0, 0.0
            
        # Calculate energy for ROBOT_ONLY segments starting from the identified point
        for i in range(start_idx_in_robot_path, len(robot_path) - 1):
            node1_id, v_type1 = robot_path[i]
            node2_id, v_type2 = robot_path[i+1]
            
            if v_type1 == VehicleType.ROBOT_ONLY or v_type2 == VehicleType.ROBOT_ONLY:
                if node1_id != start_node_in_robot_route and not in_independent_trip:
                    continue
                    
                in_independent_trip = True
                dist = self.solution.distance_matrix[node1_id][node2_id]
                energy_segment = dist * self.solution.robot_params["energy_consumption_rate"]
                
                if not first_segment_found and v_type2 == VehicleType.ROBOT_ONLY:
                    low_level_energy = energy_segment
                    first_segment_found = True
                    
                high_level_energy += energy_segment
                
                # Stop calculation when robot trip ends (reaches station/depot and is carried next)
                is_end_node = node2_id in self.solution.charging_stations or node2_id == self.solution.depot
                next_node_is_carried = (i + 2 < len(robot_path) and robot_path[i+2][1] != VehicleType.ROBOT_ONLY)
                
                if is_end_node and next_node_is_carried:
                    break
                    
        high_level_energy = min(high_level_energy, self.solution.robot_params["battery_capacity"])
        low_level_energy = min(low_level_energy, high_level_energy)
        
        return low_level_energy, high_level_energy

    def calculate_current_time_warp(self, route: VehicleRoute, current_van_node_idx: int, 
                                   current_van_arrival_time: float) -> float:
        """Calculate time warp (maximum waiting time) at the current van node."""
        van_path = route.get_van_route()
        
        if current_van_node_idx >= len(van_path) - 1:
            return 0.0
            
        current_node_id, _ = van_path[current_van_node_idx]
        next_node_id, _ = van_path[current_van_node_idx + 1]
        
        current_service_time = self.solution.service_time.get(current_node_id, 0.0)
        earliest_start_current, _ = self.solution.time_windows.get(current_node_id, (0.0, float('inf')))
        earliest_departure = max(current_van_arrival_time, earliest_start_current) + current_service_time
        
        travel_time = self.solution.distance_matrix[current_node_id][next_node_id] / self.solution.van_params['speed']
        _, latest_arrival_next = self.solution.time_windows.get(next_node_id, (0.0, float('inf')))
        latest_departure_allowed = latest_arrival_next - travel_time
        
        time_warp = latest_departure_allowed - earliest_departure
        return max(0.0, time_warp)

    def apply_four_step_heuristic_at_node(self, vehicle_id: int, van_node_idx: int, 
                                         current_van_arrival_time: float, 
                                         current_van_battery: float, 
                                         current_robot_battery: float) -> Tuple[bool, float, float, float, float]:
        """
        Apply the 4-step charging heuristic at a station/depot.
        Returns: (success, van_charge, robot_charge, en_route_charge, charge_time)
        """
        route = self.solution.routes[vehicle_id]
        van_path = route.get_van_route()
        current_node_id, current_vtype = van_path[van_node_idx]
        
        # Only apply at charging locations
        is_charging_location = (current_node_id in self.solution.charging_stations or 
                               current_node_id == self.solution.depot)
                               
        if not is_charging_location or van_node_idx >= len(van_path) - 1:
            return True, 0.0, 0.0, 0.0, 0.0
            
        # Initialize parameters
        time_warp = self.calculate_current_time_warp(route, van_node_idx, current_van_arrival_time)
        remaining_time_warp = time_warp
        
        van_charge_rate = self.solution.van_params["recharging_rate"]
        robot_charge_rate = self.solution.robot_params["recharging_rate"]
        
        van_max_capacity = self.solution.van_params["battery_capacity"]
        robot_max_capacity = self.solution.robot_params["battery_capacity"]
        
        robot_energy_low, robot_energy_high = self.get_robot_independent_energy_needs(route, current_node_id)
        is_robot_starting_trip = robot_energy_high > 0
        
        next_node_id, next_vtype = van_path[van_node_idx + 1]
        is_robot_onboard_next_segment = (next_vtype == VehicleType.VAN_CARRY_ROBOT)
        
        van_target_charge_amount = 0.0
        robot_target_charge_amount = 0.0
        en_route_charge_next_segment = 0.0
        total_charge_time_spent = 0.0
        
        temp_current_van_battery = current_van_battery
        temp_current_robot_battery = current_robot_battery
        
        # Step 1: Van low charge
        van_charge_needed_low = 0.0
        next_station_id = self.get_next_parking_station(route, van_node_idx)
        
        if next_station_id is not None:
            distance_to_next_station = 0
            for i in range(van_node_idx, len(van_path) - 1):
                n1, _ = van_path[i]
                n2, _ = van_path[i+1]
                distance_to_next_station += self.solution.distance_matrix[n1][n2]
                if n2 == next_station_id:
                    break
            else:
                distance_to_next_station = self.solution.distance_matrix[current_node_id][self.solution.depot]
                
            energy_to_next_station = distance_to_next_station * self.solution.van_params["energy_consumption_rate"]
            van_charge_needed_low = max(0.0, energy_to_next_station - temp_current_van_battery)
            van_charge_needed_low = min(van_charge_needed_low, van_max_capacity - temp_current_van_battery)
            
        van_charge_time_step1 = self.calculate_charge_time(van_charge_needed_low, van_charge_rate)
        
        if van_charge_time_step1 > remaining_time_warp + 1e-6:
            return False, 0.0, 0.0, 0.0, 0.0
            
        van_target_charge_amount += van_charge_needed_low
        time_spent_step1 = van_charge_time_step1
        remaining_time_warp -= time_spent_step1
        total_charge_time_spent = max(total_charge_time_spent, time_spent_step1)
        temp_current_van_battery += van_charge_needed_low
        
        # Step 2: Robot charge
        robot_charge_time_step2 = 0.0
        robot_target_charge_step2 = 0.0
        
        if is_robot_starting_trip:
            robot_charge_needed_high = max(0.0, robot_energy_high - temp_current_robot_battery)
            robot_charge_needed_high = min(robot_charge_needed_high, robot_max_capacity - temp_current_robot_battery)
            robot_charge_time_high = self.calculate_charge_time(robot_charge_needed_high, robot_charge_rate)
            
            if robot_charge_time_high <= remaining_time_warp + 1e-6:
                robot_target_charge_step2 = robot_charge_needed_high
                robot_charge_time_step2 = robot_charge_time_high
            else:
                robot_charge_needed_low = max(0.0, robot_energy_low - temp_current_robot_battery)
                robot_charge_needed_low = min(robot_charge_needed_low, robot_max_capacity - temp_current_robot_battery)
                robot_charge_time_low = self.calculate_charge_time(robot_charge_needed_low, robot_charge_rate)
                
                if robot_charge_time_low <= remaining_time_warp + 1e-6:
                    robot_target_charge_step2 = robot_charge_needed_low
                    robot_charge_time_step2 = robot_charge_time_low
                    
                    # Calculate en-route charging if needed
                    if is_robot_onboard_next_segment and robot_energy_high > robot_energy_low:
                        needed_en_route = max(0.0, robot_energy_high - (temp_current_robot_battery + robot_target_charge_step2))
                        travel_time_next_segment = (self.solution.distance_matrix[current_node_id][next_node_id] / 
                                                  self.solution.van_params["speed"])
                        max_en_route_possible_time = travel_time_next_segment * robot_charge_rate
                        
                        van_energy_after_own_needs = (temp_current_van_battery - 
                                                    (self.solution.distance_matrix[current_node_id][next_node_id] * 
                                                     self.solution.van_params["energy_consumption_rate"]))
                        max_en_route_possible_van = max(0, van_energy_after_own_needs) * self.solution.van_params.get("charging_efficiency", 1.0)
                        
                        en_route_charge_next_segment = min(needed_en_route, max_en_route_possible_time, max_en_route_possible_van)
                else:
                    return False, 0.0, 0.0, 0.0, 0.0
                    
            robot_target_charge_amount += robot_target_charge_step2
            time_spent_step2 = robot_charge_time_step2
            remaining_time_warp -= time_spent_step2
            total_charge_time_spent = max(total_charge_time_spent, time_spent_step2)
            temp_current_robot_battery += robot_target_charge_step2
        
        # Step 3: Van top-up
        van_charge_needed_to_max = max(0.0, van_max_capacity - temp_current_van_battery)
        van_charge_time_to_max = self.calculate_charge_time(van_charge_needed_to_max, van_charge_rate)
        actual_van_charge_time_step3 = min(van_charge_time_to_max, remaining_time_warp)
        additional_van_charge_step3 = actual_van_charge_time_step3 * van_charge_rate
        
        van_target_charge_amount += additional_van_charge_step3
        time_spent_step3 = actual_van_charge_time_step3
        remaining_time_warp -= time_spent_step3
        total_charge_time_spent = max(time_spent_step1 + time_spent_step3, total_charge_time_spent)
        temp_current_van_battery += additional_van_charge_step3
        
        # Step 4: Robot top-up
        if remaining_time_warp > 1e-6:
            robot_charge_needed_to_max = max(0.0, robot_max_capacity - temp_current_robot_battery)
            robot_charge_time_to_max = self.calculate_charge_time(robot_charge_needed_to_max, robot_charge_rate)
            actual_robot_charge_time_step4 = min(robot_charge_time_to_max, remaining_time_warp)
            additional_robot_charge_step4 = actual_robot_charge_time_step4 * robot_charge_rate
            
            robot_target_charge_amount += additional_robot_charge_step4
            time_spent_step4 = actual_robot_charge_time_step4
            remaining_time_warp -= time_spent_step4
            total_charge_time_spent = max(time_spent_step2 + time_spent_step4, total_charge_time_spent)
            temp_current_robot_battery += additional_robot_charge_step4
            
        return True, van_target_charge_amount, robot_target_charge_amount, en_route_charge_next_segment, total_charge_time_spent

    # --- Main Feasibility Check ---

    def is_feasible(self, check_time_windows=True, check_capacity=True, 
                   check_energy=True, check_single_vehicle_visit=True) -> bool:
        """Check overall feasibility of the solution."""
        # 1. Check unassigned customers
        if self.solution.unassigned_customers:
            return False

        # 2. Check single vehicle visit constraint
        if check_single_vehicle_visit:
            customer_visit_map = {}  # customer_id -> vehicle_id
            if hasattr(self.solution, 'routes'):
                for vehicle_id, route in self.solution.routes.items():
                    customers_in_this_route = set()  # Optimization
                    # Check van path
                    if hasattr(route, 'get_van_route'):
                        for node_id, _ in route.get_van_route():
                            if node_id in self.solution.all_customers and node_id not in customers_in_this_route:
                                if node_id in customer_visit_map:
                                    if customer_visit_map[node_id] != vehicle_id:
                                        return False  # Violation
                                else:
                                    customer_visit_map[node_id] = vehicle_id
                                customers_in_this_route.add(node_id)
                    # Check robot path (only for customers not seen in van path of this route)
                    if hasattr(route, 'get_robot_route'):
                        for node_id, _ in route.get_robot_route():
                            if node_id in self.solution.all_customers and node_id not in customers_in_this_route:
                                if node_id in customer_visit_map:
                                    if customer_visit_map[node_id] != vehicle_id:
                                        return False  # Violation
                                else:
                                    customer_visit_map[node_id] = vehicle_id
                                # No need to add to customers_in_this_route here
                                
            # Check that all customers are visited
            visited_customers = set(customer_visit_map.keys())
            all_customers_set = set(self.solution.all_customers)
            if visited_customers != all_customers_set:
                return False  # Not all customers are visited

        # 3. Check feasibility of individual routes
        if hasattr(self.solution, 'routes'):
            for vehicle_id in self.solution.routes.keys():
                if not self._is_vehicle_route_feasible(vehicle_id, check_time_windows, check_capacity, check_energy):
                    return False  # One infeasible route makes the solution infeasible

        return True  # All checks passed

    def _is_vehicle_route_feasible(self, vehicle_id: int, check_time_windows=True, 
                                  check_capacity=True, check_energy=True) -> bool:
        """Check feasibility of a single route (TW, Energy, Capacity)."""
        if vehicle_id not in self.solution.routes:
            return True  # Non-existent route is considered feasible?
            
        route = self.solution.routes[vehicle_id]
        if not hasattr(route, 'get_van_route'):
            return True  # Route without van path feasible?

        van_path = route.get_van_route()
        # Basic structure checks
        if not van_path or len(van_path) < 2:
            return True if len(van_path) <= 2 else False
            
        if van_path[0][0] != self.solution.depot or van_path[-1][0] != self.solution.depot:
            return False

        # Initialize state
        current_time = self.solution.time_windows.get(self.solution.depot, (0.0, 0.0))[0]
        current_van_battery = self.solution.van_params["battery_capacity"]
        current_robot_battery = self.solution.robot_params["battery_capacity"]

        # Simulate van traversal
        for i in range(len(van_path) - 1):
            node1_id, vtype1 = van_path[i]
            node2_id, vtype2 = van_path[i+1]
            distance = self.solution.distance_matrix[node1_id][node2_id]

            # 1. Apply charging heuristic at node1
            van_charge_plan = 0.0
            robot_charge_plan = 0.0
            en_route_charge_plan = 0.0
            charge_time_at_node1 = 0.0
            
            if check_energy:
                success, van_charge_plan, robot_charge_plan, en_route_charge_plan, charge_time_at_node1 = \
                    self.apply_four_step_heuristic_at_node(
                        vehicle_id, i, current_time, current_van_battery, current_robot_battery
                    )
                    
                if not success:
                    return False  # Heuristic failed -> infeasible energy
                    
                # Update battery levels after charging at node1
                current_van_battery = min(
                    current_van_battery + van_charge_plan, 
                    self.solution.van_params["battery_capacity"]
                )
                current_robot_battery = min(
                    current_robot_battery + robot_charge_plan, 
                    self.solution.robot_params["battery_capacity"]
                )

            # 2. Check Time Windows for arrival at node2
            service_time1 = self.solution.service_time.get(node1_id, 0.0)
            earliest_start1, _ = self.solution.time_windows.get(node1_id, (0.0, float('inf')))
            departure_time1 = max(current_time, earliest_start1) + service_time1 + charge_time_at_node1
            travel_time = distance / self.solution.van_params['speed']
            arrival_time_at_node2 = departure_time1 + travel_time
            earliest_arrival2, latest_arrival2 = self.solution.time_windows.get(node2_id, (0.0, float('inf')))
            
            if check_time_windows and arrival_time_at_node2 > latest_arrival2 + 1e-6:
                return False  # TW violation
                
            current_time = max(arrival_time_at_node2, earliest_arrival2)  # Update time for next iteration

            # 3. Update Energy after travel segment
            if check_energy:
                van_energy_consumed = distance * self.solution.van_params["energy_consumption_rate"]
                van_energy_for_enroute = en_route_charge_plan / self.solution.van_params.get("charging_efficiency", 1.0)
                current_van_battery -= (van_energy_consumed + van_energy_for_enroute)
                
                if current_van_battery < -1e-6:
                    return False  # Van ran out of battery

                # Update robot battery if it was on the van
                if vtype2 == VehicleType.VAN_CARRY_ROBOT:
                    robot_energy_consumed = distance * self.solution.robot_params["energy_consumption_rate"]
                    current_robot_battery -= robot_energy_consumed
                    current_robot_battery += en_route_charge_plan
                    current_robot_battery = min(current_robot_battery, self.solution.robot_params["battery_capacity"])
                    
                    if current_robot_battery < -1e-6:
                        return False  # Robot ran out of battery (on van)

        # 4. Simplified Capacity Check (total demand vs capacity)
        if check_capacity:
            total_demand_on_route = 0
            customer_nodes_in_van_path = {
                node_id for node_id, vtype in van_path 
                if node_id in self.solution.all_customers
            }
            
            for cust_id in customer_nodes_in_van_path:
                total_demand_on_route += self.solution.customer_demand.get(cust_id, 0.0)
                
            if total_demand_on_route > self.solution.van_params["capacity"] + 1e-6:
                return False  # Capacity violation

        return True
