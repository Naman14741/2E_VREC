from typing import Optional, Tuple, List, Dict

from route import VehicleRoute

class HeuristicFeasible:
    def __init__(self, van_routes: List[VehicleRoute], robot_routes: List[VehicleRoute],
                 distance_matrix: Dict[int, Dict[int, float]], time_windows: Dict[int, Tuple[float, float]],
                 service_times: Dict[int, float], van_params: Dict[str, float], robot_params: Dict[str, float],
                 depot: int, parking_stations: List[int], customers_robot_only: List[int], all_customers: List[int],
                 customer_demand: Dict[int, float]):
        self.van_routes = van_routes
        self.robot_routes = robot_routes
        self.distance_matrix = distance_matrix
        self.time_windows = time_windows
        self.service_times = service_times
        self.van_params = van_params
        self.robot_params = robot_params
        self.depot = depot
        self.parking_stations = parking_stations
        self.customers_robot_only = customers_robot_only
        self.all_customers = all_customers
        self.customer_demand = customer_demand
        self.unassigned_customers = []  # Danh sách khách hàng chưa được phục vụ (nếu có)

        # Bản đồ từ van_route_idx đến list robot_route_idx
        self.van_robot_map = {}  # Cần được cập nhật dựa trên giải pháp

    def calculate_charge_time(self, energy_needed: float, rate: float) -> float:
        """Tính thời gian cần để nạp một lượng năng lượng nhất định."""
        if rate <= 0:
            return float('inf') if energy_needed > 0 else 0.0
        return energy_needed / rate

    def get_next_parking_station(self, route: VehicleRoute, current_node_idx: int) -> Optional[int]:
        """Tìm nút trạm đỗ tiếp theo trong tuyến đường sau chỉ số hiện tại."""
        for i in range(current_node_idx + 1, len(route.nodes)):
            if route.nodes[i] in self.parking_stations or route.nodes[i] == self.depot:
                return route.nodes[i]
        return None  # Nên luôn kết thúc tại depot

    def get_robot_route_energy_needs(self, robot_route: VehicleRoute) -> Tuple[float, float]:
        """
        Ước tính nhu cầu năng lượng mức thấp (đoạn tiếp theo) và mức cao cho tuyến đường robot.
        Trả về (low_level_energy, high_level_energy).
        """
        if len(robot_route.nodes) < 2:
            return 0.0, 0.0

        # Mức thấp: Năng lượng cho đoạn đầu tiên
        node1, node2 = robot_route.nodes[0], robot_route.nodes[1]
        distance_first_segment = self.distance_matrix[node1][node2]
        low_level_energy = distance_first_segment * self.robot_params["energy_consumption_rate"]

        # Mức cao: Năng lượng cho toàn bộ tuyến đường (đơn giản hóa)
        high_level_energy = 0.0
        for i in range(len(robot_route.nodes) - 1):
            n1, n2 = robot_route.nodes[i], robot_route.nodes[i + 1]
            dist = self.distance_matrix[n1][n2]
            high_level_energy += dist * self.robot_params["energy_consumption_rate"]

        # Giới hạn mức cao tại dung lượng pin
        high_level_energy = min(high_level_energy, self.robot_params["battery_capacity"])

        return low_level_energy, high_level_energy

    def calculate_current_time_warp(self, route: VehicleRoute, current_node_idx: int, current_arrival_time: float) -> float:
        """
        Tính toán time warp (thời gian dư) hiện tại tại nút.
        Đây là sự khác biệt giữa thời gian khởi hành muộn nhất có thể và
        thời gian khởi hành sớm nhất có thể mà không vi phạm cửa sổ thời gian của nút tiếp theo.
        """
        if current_node_idx >= len(route.nodes) - 1:
            return 0.0  # Không có nút tiếp theo

        current_node = route.nodes[current_node_idx]
        next_node = route.nodes[current_node_idx + 1]

        # Thời gian khởi hành sớm nhất từ nút hiện tại
        current_service_time = self.service_times.get(current_node, 0.0)
        earliest_start_current, _ = self.time_windows.get(current_node, (0.0, float('inf')))
        earliest_departure = max(current_arrival_time + current_service_time, earliest_start_current)

        # Thời gian khởi hành muộn nhất từ nút hiện tại để đáp ứng deadline của nút tiếp theo
        travel_time = self.distance_matrix[current_node][next_node] / self.van_params["speed"]  # Giả sử tuyến van
        _, latest_arrival_next = self.time_windows.get(next_node, (0.0, float('inf')))
        latest_departure = latest_arrival_next - travel_time

        # Tính time warp
        time_warp = latest_departure - earliest_departure

        return max(0.0, time_warp)  # Time warp không thể âm

    def apply_four_step_heuristic_at_node(self, van_route_idx: int, node_idx: int, current_van_arrival_time: float, current_van_battery: float, current_robot_battery: float) -> Tuple[bool, float, float, float]:
        """
        Áp dụng heuristic nạp điện bốn bước tại một nút cụ thể trong tuyến đường van.
        Trả về (success, van_target_charge, robot_target_charge, en_route_charge_next).
        """
        van_route = self.van_routes[van_route_idx]
        current_node = van_route.nodes[node_idx]

        # Heuristic chỉ áp dụng tại trạm đỗ hoặc depot
        if current_node not in self.parking_stations and current_node != self.depot:
            return True, 0.0, 0.0, 0.0  # Không cần hành động nạp điện

        if node_idx >= len(van_route.nodes) - 1:
            return True, 0.0, 0.0, 0.0  # Tại depot cuối, không có đoạn tiếp theo

        # --- Tính toán ban đầu ---
        time_warp = self.calculate_current_time_warp(van_route, node_idx, current_van_arrival_time)
        remaining_time_warp = time_warp

        van_charge_rate = self.van_params["recharging_rate"]
        robot_charge_rate = self.robot_params["recharging_rate"]

        van_max_capacity = self.van_params["battery_capacity"]
        robot_max_capacity = self.robot_params["battery_capacity"]

        # Tìm tuyến robot liên quan
        associated_robot_routes = []
        if van_route_idx in self.van_robot_map:
            for robot_route_idx in self.van_robot_map[van_route_idx]:
                if self.robot_routes[robot_route_idx].nodes and self.robot_routes[robot_route_idx].nodes[0] == current_node:
                    associated_robot_routes.append(self.robot_routes[robot_route_idx])
        robot_route = associated_robot_routes[0] if associated_robot_routes else None

        is_robot_drop_off = robot_route is not None and robot_route.nodes[0] == current_node
        robot_onboard_next = (node_idx < len(van_route.robot_onboard) and van_route.robot_onboard[node_idx])

        van_target_charge_amount = 0.0
        robot_target_charge_amount = 0.0
        en_route_charge_next_segment = 0.0

        # --- Bước 1: Nạp van đến mức thấp ---
        van_charge_needed_low = 0.0
        next_station = self.get_next_parking_station(van_route, node_idx)
        if next_station is not None:
            dist_to_next_station = self.distance_matrix[current_node][next_station]
            energy_to_next_station = dist_to_next_station * self.van_params["energy_consumption_rate"]
            van_charge_needed_low = max(0.0, energy_to_next_station - current_van_battery)

        van_charge_time_step1 = self.calculate_charge_time(van_charge_needed_low, van_charge_rate)

        if van_charge_time_step1 > remaining_time_warp:
            print(f"Heuristic Step 1 FAIL: Van needs {van_charge_time_step1:.2f}s, warp is {remaining_time_warp:.2f}s")
            return False, 0.0, 0.0, 0.0

        van_target_charge_amount = van_charge_needed_low
        remaining_time_warp -= van_charge_time_step1
        current_van_battery += van_target_charge_amount

        # --- Bước 2: Nạp robot đến mức cao ---
        robot_charge_time_step2 = 0.0
        robot_target_charge_step2 = 0.0
        if robot_route:
            robot_energy_low, robot_energy_high = self.get_robot_route_energy_needs(robot_route)
            robot_charge_needed_high = max(0.0, robot_energy_high - current_robot_battery)
            robot_charge_time_high = self.calculate_charge_time(robot_charge_needed_high, robot_charge_rate)

            if robot_charge_time_high <= remaining_time_warp:
                robot_target_charge_step2 = robot_charge_needed_high
                robot_charge_time_step2 = robot_charge_time_high
            else:
                robot_charge_needed_low = max(0.0, robot_energy_low - current_robot_battery)
                robot_charge_time_low = self.calculate_charge_time(robot_charge_needed_low, robot_charge_rate)

                if is_robot_drop_off:
                    # Kiểm tra xem thời gian nạp đến mức thấp có làm van đến muộn tại nút tiếp theo không
                    total_time_at_node = max(current_van_arrival_time + self.service_times.get(current_node, 0.0), self.time_windows.get(current_node, (0.0, float('inf')))[0]) + robot_charge_time_low
                    travel_time = self.distance_matrix[current_node][van_route.nodes[node_idx + 1]] / self.van_params["speed"]
                    arrival_next = total_time_at_node + travel_time
                    if arrival_next > self.time_windows[van_route.nodes[node_idx + 1]][1]:
                        print(f"Heuristic Step 2 FAIL: Charging robot to low level would delay van at next node.")
                        return False, 0.0, 0.0, 0.0
                    robot_target_charge_step2 = robot_charge_needed_low
                    robot_charge_time_step2 = robot_charge_time_low
                    remaining_time_warp = 0  # Tiêu thụ hết time warp
                else:
                    if robot_charge_time_low <= remaining_time_warp:
                        robot_target_charge_step2 = robot_charge_needed_low
                        robot_charge_time_step2 = robot_charge_time_low
                        if robot_onboard_next:
                            needed_en_route = max(0.0, robot_energy_high - (current_robot_battery + robot_target_charge_step2))
                            next_node = van_route.nodes[node_idx + 1]
                            travel_time_next = self.distance_matrix[current_node][next_node] / self.van_params["speed"]
                            max_en_route_possible_time = travel_time_next * robot_charge_rate
                            van_energy_after_own_needs = current_van_battery - (self.distance_matrix[current_node][next_node] * self.van_params["energy_consumption_rate"])
                            max_en_route_possible_van = max(0, van_energy_after_own_needs)
                            en_route_charge_next_segment = min(needed_en_route, max_en_route_possible_time, max_en_route_possible_van)
                    else:
                        robot_target_charge_step2 = 0
                        robot_charge_time_step2 = 0
                        if robot_onboard_next:
                            needed_en_route = max(0.0, robot_energy_low - current_robot_battery)
                            next_node = van_route.nodes[node_idx + 1]
                            travel_time_next = self.distance_matrix[current_node][next_node] / self.van_params["speed"]
                            max_en_route_possible_time = travel_time_next * robot_charge_rate
                            van_energy_after_own_needs = current_van_battery - (self.distance_matrix[current_node][next_node] * self.van_params["energy_consumption_rate"])
                            max_en_route_possible_van = max(0, van_energy_after_own_needs)
                            en_route_charge_next_segment = min(needed_en_route, max_en_route_possible_time, max_en_route_possible_van)

            robot_target_charge_amount = robot_target_charge_step2
            remaining_time_warp = max(0.0, remaining_time_warp - robot_charge_time_step2)
            current_robot_battery += robot_target_charge_amount

        # --- Bước 3: Nạp van đến mức tối đa ---
        van_charge_needed_to_max = max(0.0, van_max_capacity - current_van_battery)
        van_charge_time_to_max = self.calculate_charge_time(van_charge_needed_to_max, van_charge_rate)
        actual_van_charge_time_step3 = min(van_charge_time_to_max, remaining_time_warp)
        additional_van_charge_step3 = actual_van_charge_time_step3 * van_charge_rate
        van_target_charge_amount += additional_van_charge_step3
        remaining_time_warp -= actual_van_charge_time_step3
        current_van_battery += additional_van_charge_step3

        # --- Bước 4: Nạp robot đến mức tối đa ---
        if robot_route:
            robot_charge_needed_to_max = max(0.0, robot_max_capacity - current_robot_battery)
            robot_charge_time_to_max = self.calculate_charge_time(robot_charge_needed_to_max, robot_charge_rate)
            actual_robot_charge_time_step4 = min(robot_charge_time_to_max, remaining_time_warp)
            additional_robot_charge_step4 = actual_robot_charge_time_step4 * robot_charge_rate
            robot_target_charge_amount += additional_robot_charge_step4

        return True, van_target_charge_amount, robot_target_charge_amount, en_route_charge_next_segment

    def is_feasible(self, check_time_windows=True, check_capacity=True, check_energy=True) -> bool:
        """
        Kiểm tra xem trạng thái giải pháp hiện tại có khả thi hay không.
        """
        if self.unassigned_customers:
            return False

        for van_idx, van_route in enumerate(self.van_routes):
            if not self._is_van_route_feasible(van_idx, check_time_windows, check_capacity, check_energy):
                return False

        return True

    def _is_van_route_feasible(self, van_route_idx: int, check_time_windows=True) -> bool:
        """Kiểm tra xem một tuyến đường van đơn lẻ có khả thi hay không."""
        van_route = self.van_routes[van_route_idx]
        if not van_route.nodes or len(van_route.nodes) < 2:
            return True
        if van_route.nodes[0] != self.depot or van_route.nodes[-1] != self.depot:
            return False

        current_time = self.time_windows.get(self.depot, (0.0, 0.0))[0]
        current_van_battery = self.van_params["battery_capacity"]
        current_robot_battery = 0.0  # Giả định ban đầu

        for i in range(len(van_route.nodes) - 1):
            node1, node2 = van_route.nodes[i], van_route.nodes[i + 1]
            distance = self.distance_matrix[node1][node2]
            travel_time = distance / self.van_params["speed"]

            # Áp dụng heuristic tại node1
            success, van_charge, robot_charge, en_route_charge = self.apply_four_step_heuristic_at_node(
                van_route_idx, i, current_time, current_van_battery, current_robot_battery
            )
            if not success:
                return False

            # Tính thời gian nạp điện
            van_charge_time = van_charge / self.van_params["recharging_rate"] if van_charge > 0 else 0.0
            robot_charge_time = robot_charge / self.robot_params["recharging_rate"] if robot_charge > 0 else 0.0
            total_charge_time = max(van_charge_time, robot_charge_time)

            # Cập nhật thời gian khởi hành từ node1
            service_time = self.service_times.get(node1, 0.0)
            earliest_start, _ = self.time_windows.get(node1, (0.0, float('inf')))
            departure_time = max(current_time + service_time, earliest_start) + total_charge_time

            # Cập nhật thời gian đến node2
            current_time = departure_time + travel_time
            earliest_arrival, latest_arrival = self.time_windows.get(node2, (0.0, float('inf')))
            if check_time_windows and current_time > latest_arrival + 1e-6:
                return False

            # Cập nhật pin van và robot
            current_van_battery += van_charge - (distance * self.van_params["energy_consumption_rate"])
            if current_van_battery < 0:
                return False
            if van_route.robot_onboard[i]:
                current_robot_battery += en_route_charge - (distance * self.robot_params["energy_consumption_rate"])
                if current_robot_battery < 0:
                    return False

        return True