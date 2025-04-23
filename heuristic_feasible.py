from typing import Optional, Tuple, Set
from solution import VRPECSolution
from route import VehicleRoute, VehicleType

class HeuristicFeasible:

    def __init__(self, solution: VRPECSolution):
        """
        Khởi tạo bộ kiểm tra tính khả thi.
        Args:
            solution (VRPECSolution): Giải pháp cần kiểm tra. Giả định solution.routes là List[VehicleRoute].
        """
        self.solution: VRPECSolution = solution
        # Lưu trữ các tham số thường dùng
        self.depot = solution.depot
        self.charging_stations = solution.charging_stations
        self.all_customers = solution.all_customers
        self.distance_matrix = solution.distance_matrix
        self.van_params = solution.van_params
        self.robot_params = solution.robot_params
        self.service_time = solution.service_time
        self.time_windows = solution.time_windows
        self.customer_demand = solution.customer_demand

    def calculate_charge_time(self, energy_needed: float, rate: float) -> float:
        """Tính thời gian cần để sạc một lượng năng lượng."""
        if rate <= 1e-9:
            return float('inf') if energy_needed > 1e-9 else 0.0
        return energy_needed / rate if energy_needed > 1e-9 else 0.0

    def get_next_parking_station(self, route: VehicleRoute, current_van_node_idx: int) -> Optional[int]:
        """Tìm trạm đỗ/depot tiếp theo trong đường đi của van."""
        if not hasattr(route, 'get_van_route'): return None
        van_path = route.get_van_route()
        if not van_path: return None

        for i in range(current_van_node_idx + 1, len(van_path)):
            node_id, _ = van_path[i]
            if node_id in self.charging_stations or node_id == self.depot:
                return node_id
        if van_path[-1][0] == self.depot:
            return self.depot
        return None

    def get_robot_independent_energy_needs(self, route: VehicleRoute, start_node_in_robot_route: int) -> Tuple[float, float]:
        """Ước tính nhu cầu năng lượng cho chuyến đi độc lập của robot (ROBOT_ONLY)."""
        if not hasattr(route, 'get_robot_route'): 
            return 0.0, 0.0
        robot_path = route.get_robot_route()
        low_level_energy = 0.0 # Năng lượng tối thiểu cần (đoạn đầu tiên)
        high_level_energy = 0.0 # Tổng năng lượng cần

        if len(robot_path) < 2: 
            return 0.0, 0.0

        start_idx_in_robot_path = -1
        # Tìm điểm bắt đầu chuyến đi ROBOT_ONLY
        for idx, (node_id, v_type) in enumerate(robot_path):
            if node_id == start_node_in_robot_route and \
               idx + 1 < len(robot_path) and \
               robot_path[idx+1][1] == VehicleType.ROBOT_ONLY:
                start_idx_in_robot_path = idx + 1
                break

        if start_idx_in_robot_path == -1: return 0.0, 0.0

        # Tính năng lượng cho các đoạn ROBOT_ONLY
        first_segment_energy = 0.0
        first_segment_found = False
        robot_trip_ended = False
        max_robot_battery = self.robot_params.get("battery_capacity", float('inf'))
        robot_energy_rate = self.robot_params.get("energy_consumption_rate", float('inf'))
        if self.distance_matrix is None: return float('inf'), float('inf')

        for i in range(start_idx_in_robot_path, len(robot_path)):
             node1_id, v_type1 = robot_path[i-1]
             node2_id, v_type2 = robot_path[i]

             if v_type2 == VehicleType.ROBOT_ONLY:
                 dist = self.distance_matrix[node1_id][node2_id]
                 energy_segment = dist * robot_energy_rate
                 high_level_energy += energy_segment

                 if not first_segment_found:
                     first_segment_energy = energy_segment
                     first_segment_found = True

                 # Kiểm tra kết thúc chuyến đi
                 is_end_station_node = node2_id in self.charging_stations or node2_id == self.depot
                 next_node_is_carried = (i + 1 < len(robot_path) and robot_path[i+1][1] != VehicleType.ROBOT_ONLY)
                 if is_end_station_node and next_node_is_carried:
                     robot_trip_ended = True
                     break
             elif v_type1 == VehicleType.ROBOT_ONLY and v_type2 != VehicleType.ROBOT_ONLY:
                 robot_trip_ended = True
                 break

        if not robot_trip_ended and len(robot_path)>0 and robot_path[-1][1] == VehicleType.ROBOT_ONLY:
             robot_trip_ended = True

        high_level_energy = min(high_level_energy, max_robot_battery)
        low_level_energy = min(first_segment_energy, high_level_energy)

        return low_level_energy, high_level_energy

    def calculate_current_time_warp(self, route: VehicleRoute, current_van_node_idx: int,
                                   current_van_arrival_time: float) -> float:
        """Tính time warp (thời gian chờ tối đa cho phép) tại nút hiện tại của van."""
        if not hasattr(route, 'get_van_route'): 
            return 0.0
        van_path = route.get_van_route()

        if not van_path or current_van_node_idx >= len(van_path) - 1: 
            return 0.0

        current_node_id, _ = van_path[current_van_node_idx]
        next_node_id, _ = van_path[current_van_node_idx + 1]
        if self.distance_matrix is None: 
            return 0.0
        van_speed = self.van_params.get('speed', float('inf'))
        if van_speed <= 1e-9: return 0.0

        current_service_time = self.service_time.get(current_node_id, 0.0)
        earliest_start_current, _ = self.time_windows.get(current_node_id, (0.0, float('inf')))
        start_service_time = max(current_van_arrival_time, earliest_start_current)
        earliest_departure = start_service_time + current_service_time

        travel_time = self.distance_matrix[current_node_id][next_node_id] / van_speed
        _, latest_arrival_next = self.time_windows.get(next_node_id, (0.0, float('inf')))
        latest_departure_allowed = latest_arrival_next - travel_time

        time_warp = latest_departure_allowed - earliest_departure
        return max(0.0, time_warp)

    def apply_four_step_heuristic_at_node(
        self, 
        vehicle_idx: int, 
        van_node_idx: int,
        current_van_arrival_time: float,
        current_van_battery: float,
        current_robot_battery: float
    ) -> Tuple[bool, float, float, float, float]:
        """
        Áp dụng heuristic sạc 4 bước tại trạm/depot.
        
        Trả về: (thành công, lượng sạc van, lượng sạc robot, lượng sạc trên đường, thời gian sạc)
        """
        # Check if solution and route are valid
        if not hasattr(self.solution, 'routes') or not (0 <= vehicle_idx < len(self.solution.routes)):
            return False, 0.0, 0.0, 0.0, 0.0
            
        route = self.solution.routes[vehicle_idx]
        
        if not hasattr(route, 'get_van_route'):
            return False, 0.0, 0.0, 0.0, 0.0
            
        van_path = route.get_van_route()
        
        if not (0 <= van_node_idx < len(van_path)):
            return False, 0.0, 0.0, 0.0, 0.0

        current_node_id, _ = van_path[van_node_idx]
        is_charging_location = (current_node_id in self.charging_stations or 
                               current_node_id == self.depot)
                               
        if not is_charging_location or van_node_idx >= len(van_path) - 1:
            return True, 0.0, 0.0, 0.0, 0.0

        # --- Khởi tạo tham số ---
        time_warp = self.calculate_current_time_warp(
            route, van_node_idx, current_van_arrival_time
        )
        remaining_time_warp = time_warp
        van_charge_rate = self.van_params.get("recharging_rate", 0.0)
        robot_charge_rate = self.robot_params.get("recharging_rate", 0.0)
        van_max_capacity = self.van_params.get("battery_capacity", 0.0)
        robot_max_capacity = self.robot_params.get("battery_capacity", 0.0)
        van_energy_rate = self.van_params.get("energy_consumption_rate", float('inf'))
        van_efficiency = self.van_params.get("charge_rate", 1.0)
        
        robot_energy_low, robot_energy_high = self.get_robot_independent_energy_needs(
            route, current_node_id
        )
        
        is_robot_starting_trip = robot_energy_high > 1e-9
        next_node_id, next_vtype = van_path[van_node_idx + 1]
        is_robot_onboard_next_segment = (next_vtype == VehicleType.VAN_CARRY_ROBOT)

        van_target_charge_amount = 0.0
        robot_target_charge_amount = 0.0
        en_route_charge_next_segment = 0.0
        total_charge_time_spent = 0.0
        temp_current_van_battery = current_van_battery
        temp_current_robot_battery = current_robot_battery
        
        if self.distance_matrix is None:
            return False, 0.0, 0.0, 0.0, 0.0

        # --- Step 1: Van low charge ---
        van_charge_needed_low = 0.0
        next_station_id = self.get_next_parking_station(route, van_node_idx)
        
        if next_station_id is not None:
            distance_to_next_station = 0.0
            found_next = False
            
            for i in range(van_node_idx, len(van_path) - 1):
                n1, _ = van_path[i]
                n2, _ = van_path[i+1]
                distance_to_next_station += self.distance_matrix[n1][n2]
                
                if n2 == next_station_id:
                    found_next = True
                    break
                    
            if found_next:
                energy_to_next_station = distance_to_next_station * van_energy_rate
                van_charge_needed_low = max(0.0, energy_to_next_station - temp_current_van_battery)
                van_charge_needed_low = min(
                    van_charge_needed_low, 
                    van_max_capacity - temp_current_van_battery
                )

        van_charge_time_step1 = self.calculate_charge_time(van_charge_needed_low, van_charge_rate)
        
        if van_charge_time_step1 > remaining_time_warp + 1e-6:
            return False, 0.0, 0.0, 0.0, 0.0

        van_target_charge_amount += van_charge_needed_low
        time_spent_step1 = van_charge_time_step1
        remaining_time_warp -= time_spent_step1
        total_charge_time_spent = max(total_charge_time_spent, time_spent_step1)
        temp_current_van_battery += van_charge_needed_low

        # --- Step 2: Robot charge ---
        robot_charge_time_step2 = 0.0
        robot_target_charge_step2 = 0.0
        
        if is_robot_starting_trip:
            robot_charge_needed_high = max(
                0.0, robot_energy_high - temp_current_robot_battery
            )
            robot_charge_needed_high = min(
                robot_charge_needed_high,
                robot_max_capacity - temp_current_robot_battery
            )
            robot_charge_time_high = self.calculate_charge_time(
                robot_charge_needed_high, robot_charge_rate
            )

            if robot_charge_time_high <= remaining_time_warp + 1e-6:
                robot_target_charge_step2 = robot_charge_needed_high
                robot_charge_time_step2 = robot_charge_time_high
            else:
                robot_charge_needed_low = max(
                    0.0, robot_energy_low - temp_current_robot_battery
                )
                robot_charge_needed_low = min(
                    robot_charge_needed_low,
                    robot_max_capacity - temp_current_robot_battery
                )
                robot_charge_time_low = self.calculate_charge_time(
                    robot_charge_needed_low, robot_charge_rate
                )

                if robot_charge_time_low <= remaining_time_warp + 1e-6:
                    robot_target_charge_step2 = robot_charge_needed_low
                    robot_charge_time_step2 = robot_charge_time_low
                    
                    # Calculate en-route charging if needed
                    if (is_robot_onboard_next_segment and 
                            robot_energy_high > robot_energy_low + 1e-6):
                        needed_en_route = max(
                            0.0, 
                            robot_energy_high - (
                                temp_current_robot_battery + robot_target_charge_step2
                            )
                        )
                        van_speed = self.van_params.get('speed', float('inf'))
                        
                        if van_speed > 1e-9:
                            travel_time_next_segment = (
                                self.distance_matrix[current_node_id][next_node_id] / van_speed
                            )
                        else:
                            travel_time_next_segment = float('inf')
                            
                        max_en_route_possible_time = travel_time_next_segment * robot_charge_rate
                        van_energy_needed_next = (
                            self.distance_matrix[current_node_id][next_node_id] * van_energy_rate
                        )
                        van_energy_available = max(
                            0, temp_current_van_battery - van_energy_needed_next
                        )
                        max_en_route_possible_van = van_energy_available * van_efficiency
                        
                        en_route_charge_next_segment = min(
                            needed_en_route,
                            max_en_route_possible_time,
                            max_en_route_possible_van
                        )
                else:
                    # Not feasible
                    return False, 0.0, 0.0, 0.0, 0.0

            robot_target_charge_amount += robot_target_charge_step2
            time_spent_step2 = robot_charge_time_step2
            remaining_time_warp -= time_spent_step2
            total_charge_time_spent = max(total_charge_time_spent, time_spent_step2)
            temp_current_robot_battery += robot_target_charge_step2

        # --- Step 3: Van top-up ---
        van_charge_needed_to_max = max(0.0, van_max_capacity - temp_current_van_battery)
        van_charge_time_to_max = self.calculate_charge_time(
            van_charge_needed_to_max, van_charge_rate
        )
        actual_van_charge_time_step3 = min(van_charge_time_to_max, remaining_time_warp)
        additional_van_charge_step3 = actual_van_charge_time_step3 * van_charge_rate

        van_target_charge_amount += additional_van_charge_step3
        time_spent_step3 = actual_van_charge_time_step3
        remaining_time_warp -= time_spent_step3
        total_charge_time_spent = max(
            time_spent_step1 + time_spent_step3, total_charge_time_spent
        )
        temp_current_van_battery += additional_van_charge_step3

        # --- Step 4: Robot top-up ---
        if remaining_time_warp > 1e-6:
            robot_charge_needed_to_max = max(
                0.0, robot_max_capacity - temp_current_robot_battery
            )
            robot_charge_time_to_max = self.calculate_charge_time(
                robot_charge_needed_to_max, robot_charge_rate
            )
            actual_robot_charge_time_step4 = min(
                robot_charge_time_to_max, remaining_time_warp
            )
            additional_robot_charge_step4 = actual_robot_charge_time_step4 * robot_charge_rate

            robot_target_charge_amount += additional_robot_charge_step4
            time_spent_step4 = actual_robot_charge_time_step4
            total_charge_time_spent = max(
                time_spent_step2 + time_spent_step4, total_charge_time_spent
            )
            temp_current_robot_battery += additional_robot_charge_step4

        return (
            True, 
            van_target_charge_amount, 
            robot_target_charge_amount, 
            en_route_charge_next_segment, 
            total_charge_time_spent
        )

    def is_feasible(self, check_time_windows=True, check_capacity=True,
                   check_energy=True, check_single_vehicle_visit=True) -> bool:
        """Kiểm tra tính khả thi tổng thể của giải pháp."""
        if self.depot is None or self.distance_matrix is None: 
            return False

        # 1. Check unassigned customers
        if hasattr(self.solution, 'unassigned_customers') and self.solution.unassigned_customers: 
            return False

        # 2. Check single vehicle visit constraint
        if check_single_vehicle_visit:
            customer_visit_count = {cust_id: 0 for cust_id in self.all_customers}
            if hasattr(self.solution, 'routes'):
                for route in self.solution.routes:
                    customers_served_by_route = self._get_customers_on_route(route)
                    for cust_id in customers_served_by_route:
                        if cust_id in customer_visit_count: 
                            customer_visit_count[cust_id] += 1

            for count in customer_visit_count.values():
                if count != 1: 
                    return False # Must be visited exactly once

        # 3. Check feasibility of individual routes
        if hasattr(self.solution, 'routes'):
            for vehicle_idx in range(len(self.solution.routes)):
                if not self._is_vehicle_route_feasible(vehicle_idx, check_time_windows, check_capacity, check_energy):
                    return False

        return True

    def _get_customers_on_route(self, route: VehicleRoute) -> Set[int]:
        """Helper lấy tập khách hàng duy nhất trên một route."""
        # Sao chép từ lớp Destroy để tránh phụ thuộc vòng
        customers = set()
        if hasattr(route, 'get_van_route'):
            van_nodes = {n for n, vt in route.get_van_route() if n in self.all_customers}
            customers.update(van_nodes)
        if hasattr(route, 'get_robot_route'):
            robot_only_nodes = {n for n, vt in route.get_robot_route() if n in self.all_customers and vt == VehicleType.ROBOT_ONLY}
            customers.update(robot_only_nodes)
        customers.discard(self.depot)
        return customers


    def _is_vehicle_route_feasible(self, vehicle_idx: int, check_time_windows=True,
                                  check_capacity=True, check_energy=True) -> bool:
        """Kiểm tra tính khả thi của một route đơn lẻ."""
        if not hasattr(self.solution, 'routes') or not (0 <= vehicle_idx < len(self.solution.routes)): 
            return False
        route = self.solution.routes[vehicle_idx]
        if not hasattr(route, 'get_van_route'): 
            return True # Or False if empty route is invalid
        van_path = route.get_van_route()
        if not van_path: 
            return True
        if len(van_path) < 2: 
            return len(van_path) == 0
        if van_path[0][0] != self.depot or van_path[-1][0] != self.depot: 
            return False
        if self.distance_matrix is None: 
            return False # Requirement

        # --- Khởi tạo trạng thái ---
        depot_start_time, _ = self.time_windows.get(self.depot, (0.0, float('inf')))
        current_time = depot_start_time
        current_van_battery = self.van_params.get("battery_capacity", 0.0)
        current_robot_battery = self.robot_params.get("battery_capacity", 0.0)
        van_speed = self.van_params.get('speed', float('inf'))
        van_energy_rate = self.van_params.get("energy_consumption_rate", float('inf'))
        van_efficiency = self.van_params.get("charge_rate", 1.0)
        if van_speed <= 1e-9: 
            return False

        # --- Mô phỏng hành trình ---
        for i in range(len(van_path) - 1):
            node1_id, vtype1 = van_path[i]
            node2_id, vtype2 = van_path[i+1]
            distance = self.distance_matrix[node1_id][node2_id]

            # --- 1. Sạc tại node1 ---
            van_charge_plan, robot_charge_plan, en_route_charge_plan, charge_time_at_node1 = 0.0, 0.0, 0.0, 0.0
            if check_energy and (node1_id in self.charging_stations or node1_id == self.depot):
                success, van_charge_plan, robot_charge_plan, en_route_charge_plan, charge_time_at_node1 = \
                    self.apply_four_step_heuristic_at_node(vehicle_idx, i, current_time, current_van_battery, current_robot_battery)
                if not success:
                    return False
                current_van_battery = min(current_van_battery + van_charge_plan, self.van_params.get("battery_capacity", 0.0))
                current_robot_battery = min(current_robot_battery + robot_charge_plan, self.robot_params.get("battery_capacity", 0.0))

            # --- 2. Tính toán thời gian & kiểm tra TW tại node2 ---
            service_time1 = self.service_time.get(node1_id, 0.0)
            earliest_start1, _ = self.time_windows.get(node1_id, (0.0, float('inf')))
            actual_start_time1 = max(current_time, earliest_start1)
            departure_time1 = actual_start_time1 + service_time1 + charge_time_at_node1
            travel_time = distance / van_speed
            arrival_time_at_node2 = departure_time1 + travel_time
            earliest_arrival2, latest_arrival2 = self.time_windows.get(node2_id, (0.0, float('inf')))

            if check_time_windows and arrival_time_at_node2 > latest_arrival2 + 1e-6: 
                return False
            current_time = max(arrival_time_at_node2, earliest_arrival2)

            # --- 3. Cập nhật Năng lượng sau di chuyển ---
            if check_energy:
                van_energy_consumed_travel = distance * van_energy_rate
                van_energy_for_enroute = en_route_charge_plan / van_efficiency if van_efficiency > 1e-9 else float('inf')
                current_van_battery -= (van_energy_consumed_travel + van_energy_for_enroute)
                if current_van_battery < -1e-6: 
                    return False

                if vtype2 == VehicleType.VAN_CARRY_ROBOT:
                     current_robot_battery += en_route_charge_plan # Sạc trên đường
                     current_robot_battery = min(current_robot_battery, self.robot_params.get("battery_capacity", 0.0))

            # --- 4. Kiểm tra Capacity (đơn giản) ---
            if check_capacity and i == len(van_path) - 2: # Kiểm tra 1 lần ở cuối
                customers_served = self._get_customers_on_route(route)
                total_demand = sum(self.customer_demand.get(cust_id, 0.0) for cust_id in customers_served)
                van_capacity = self.van_params.get("capacity", 0.0)
                if total_demand > van_capacity + 1e-6: return False

        # --- Kiểm tra năng lượng cuối cùng ---
        if check_energy and current_van_battery < -1e-6: return False

        return True