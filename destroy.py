import numpy as np
import random
from typing import List, Tuple, Set, Optional, Dict
import copy  # Quan trọng cho deep copy

from solution import VRPECSolution
from route import VehicleRoute, VehicleType


class Destroy:

    def __init__(self, solution: VRPECSolution, beta=0.3, gamma=0.3, epsilon=0.3):
        """
        Khởi tạo các toán tử hủy diệt.

        Args:
            solution (VRPECSolution): Một mẫu giải pháp để truy cập thông tin chung.
                                      QUAN TRỌNG: solution.copy() PHẢI thực hiện deepcopy routes.
            beta (float): Xác suất chọn xóa khách hàng ngẫu nhiên (so với tham lam).
            gamma (float): Xác suất chọn đóng route ngẫu nhiên (so với tham lam cost).
            epsilon (float): Xác suất chọn xóa trạm (visit) ngẫu nhiên (so với tham lam).
        """
        self.beta = beta
        self.gamma = gamma
        self.epsilon = epsilon
        # Lưu trữ thông tin chung từ giải pháp mẫu
        self.all_customers = solution.all_customers
        self.depot = solution.depot
        self.charging_stations = solution.charging_stations
        self.distance_matrix = solution.distance_matrix
        self.van_params = solution.van_params
        self.robot_params = solution.robot_params
        self.customer_demand = solution.customer_demand

    # --- Helpers ---

    def _get_assigned_customers(self, solution: VRPECSolution) -> Set[int]:
        """Lấy tập hợp các khách hàng duy nhất đang được phục vụ trong giải pháp."""
        assigned_customers = set()
        if not hasattr(solution, 'routes') or not solution.routes:
            return assigned_customers
        for route in solution.routes:  # Duyệt qua List[VehicleRoute]
            van_nodes = {n for n, vt in route.get_van_route() if n in self.all_customers}
            robot_only_nodes = {n for n, vt in route.get_robot_route() if n in self.all_customers and vt == VehicleType.ROBOT_ONLY}
            assigned_customers.update(van_nodes)
            assigned_customers.update(robot_only_nodes)
        assigned_customers.discard(self.depot)
        return assigned_customers

    def _find_route_for_customer(self, solution: VRPECSolution, customer_id: int) -> Optional[Tuple[int, VehicleRoute]]:
        """Tìm chỉ số (index) và đối tượng route chứa khách hàng."""
        if not hasattr(solution, 'routes') or not solution.routes:
            return None
        for idx, route in enumerate(solution.routes):  # Duyệt qua List[VehicleRoute]
            is_on_van = any(node == customer_id for node, _ in route.get_van_route())
            is_served_by_robot = any(node == customer_id and vt == VehicleType.ROBOT_ONLY for node, vt in route.get_robot_route())

            if is_served_by_robot:  # Ưu tiên nếu được robot phục vụ
                return idx, route
            elif is_on_van:  # Nếu không được robot phục vụ nhưng có trên van
                return idx, route
        return None

    def _calculate_removal_cost_delta(self, route: VehicleRoute, customer: int) -> float:
        """Tính delta chi phí khi xóa TẤT CẢ lần xuất hiện của khách hàng (mô phỏng)."""
        try:
            original_cost = route.get_route_cost()
        except Exception as e:
            # print(f"Warning: Could not get original cost for delta calculation: {e}")
            return float('inf')  # Không thể tính, trả về giá trị xấu

        # Tạo bản sao và xóa khách hàng (trừ depot)
        temp_van_route = [(n, vt) for n, vt in route.get_van_route() if n != customer or n == self.depot]
        temp_robot_route = [(n, vt) for n, vt in route.get_robot_route() if n != customer or n == self.depot]

        # Chỉ tính toán thêm nếu khách hàng thực sự đã bị xóa (ít nhất một lần)
        original_van_len = len([n for n, vt in route.get_van_route() if n == customer and n != self.depot])
        original_robot_len = len([n for n, vt in route.get_robot_route() if n == customer and n != self.depot])

        if original_van_len == 0 and original_robot_len == 0:
            return 0.0  # Khách hàng không có trên route hoặc chỉ là depot

        # Làm sạch các tuyến đường tạm thời sau khi xóa
        cleaned_van_route = self._clean_route(temp_van_route, self.depot)
        cleaned_robot_route = self._clean_route(temp_robot_route, self.depot)

        try:
            # Tạo một đối tượng VehicleRoute tạm thời để tính chi phí mới
            temp_route = VehicleRoute(
                distance_matrix=self.distance_matrix, depot=self.depot,
                unassigned_stations=[], customer_list=self.all_customers,
                station_list=self.charging_stations, customer_demand=self.customer_demand,
                van_params=self.van_params, robot_params=self.robot_params
            )
            temp_route.set_van_route(cleaned_van_route)
            temp_route.set_robot_route(cleaned_robot_route)
            new_cost = temp_route.get_route_cost()
            return new_cost - original_cost
        except Exception as e:
            # print(f"Error calculating delta cost for customer {customer}: {e}")
            return float('inf')  # Lỗi khi tính, trả về giá trị xấu

    def _calculate_single_station_visit_removal_delta(
        self, route: VehicleRoute, path_type: str, index_to_remove: int
    ) -> Optional[float]:
        """Tính delta chi phí khi xóa 1 lần ghé thăm trạm cụ thể (mô phỏng)."""
        try:
            original_cost = route.get_route_cost()
        except Exception:
            return None

        temp_path = []
        original_other_path = []
        original_path = []

        try:
            if path_type == 'van':
                original_path = route.get_van_route()
                temp_path = list(original_path)
                original_other_path = list(route.get_robot_route())
            elif path_type == 'robot':
                original_path = route.get_robot_route()
                temp_path = list(original_path)
                original_other_path = list(route.get_van_route())
            else:
                return None
        except Exception:
            return None

        if 0 <= index_to_remove < len(temp_path):
            node_id, vt = temp_path[index_to_remove]
            if node_id in self.charging_stations:  # Chỉ xóa nếu đúng là trạm
                del temp_path[index_to_remove]
            else:
                return 0.0  # Không phải trạm, không xóa, delta = 0
        else:
            return None  # Index lỗi

        cleaned_path = self._clean_route(temp_path, self.depot)

        try:
            temp_route = VehicleRoute(
                distance_matrix=self.distance_matrix, depot=self.depot,
                unassigned_stations=[], customer_list=self.all_customers,
                station_list=self.charging_stations, customer_demand=self.customer_demand,
                van_params=self.van_params, robot_params=self.robot_params
            )
            if path_type == 'van':
                temp_route.set_van_route(cleaned_path)
                temp_route.set_robot_route(original_other_path)
            else:
                temp_route.set_van_route(original_other_path)
                temp_route.set_robot_route(cleaned_path)
            new_cost = temp_route.get_route_cost()
            return new_cost - original_cost
        except Exception as e:
            # print(f"Error calculating single station visit removal delta: {e}")
            return float('inf')  # Lỗi khi tính, trả về giá trị xấu

    def _clean_route(self, route_tuples: List[Tuple[int, VehicleType]], depot: int) -> List[Tuple[int, VehicleType]]:
        """Loại bỏ các nút lặp lại liên tiếp và đảm bảo depot ở đầu/cuối."""
        if not route_tuples:
            return [(depot, VehicleType.VAN_CARRY_ROBOT), (depot, VehicleType.VAN_CARRY_ROBOT)]
        cleaned = []
        # Xử lý nút đầu
        start_node, start_type = route_tuples[0]
        cleaned.append((depot, VehicleType.VAN_CARRY_ROBOT))  # Luôn bắt đầu bằng depot
        if start_node != depot:
            cleaned.append((start_node, start_type))

        # Xử lý các nút giữa
        last_added_node, _ = cleaned[-1]
        for i in range(1, len(route_tuples)):
            current_node, current_type = route_tuples[i]
            # Chỉ thêm nếu khác nút cuối cùng VÀ không phải depot (trừ khi là nút cuối cùng của list gốc)
            if current_node != last_added_node:
                if current_node != depot or i == len(route_tuples) - 1:
                    cleaned.append((current_node, current_type))
                    last_added_node = current_node
            pass

        # Xử lý nút cuối
        if not cleaned or cleaned[-1][0] != depot:
            cleaned.append((depot, VehicleType.VAN_CARRY_ROBOT))
        else:  # Nếu nút cuối đã là depot, đảm bảo type đúng
            cleaned[-1] = (depot, VehicleType.VAN_CARRY_ROBOT)

        # Đảm bảo tối thiểu depot -> depot
        if len(cleaned) < 2:
            return [(depot, VehicleType.VAN_CARRY_ROBOT), (depot, VehicleType.VAN_CARRY_ROBOT)]

        # Loại bỏ trùng lặp liên tiếp lần cuối (có thể xảy ra do logic trên)
        final_cleaned = []
        if cleaned:
            final_cleaned.append(cleaned[0])
            for i in range(1, len(cleaned)):
                # Thêm nếu khác nút trước đó HOẶC nếu nút hiện tại là depot VÀ nút trước đó cũng là depot (chỉ để cập nhật type)
                if cleaned[i][0] != final_cleaned[-1][0]:
                    final_cleaned.append(cleaned[i])
                elif cleaned[i][0] == depot and final_cleaned[-1][0] == depot:
                    final_cleaned[-1] = cleaned[i]  # Cập nhật type của depot cuối cùng

        # Đảm bảo tối thiểu depot -> depot lần nữa sau khi final clean
        if len(final_cleaned) < 2:
            return [(depot, VehicleType.VAN_CARRY_ROBOT), (depot, VehicleType.VAN_CARRY_ROBOT)]

        return final_cleaned

    def _get_customers_on_route(self, route: VehicleRoute) -> Set[int]:
        """Helper lấy tập khách hàng duy nhất trên một route."""
        customers = set()
        van_nodes = {n for n, vt in route.get_van_route() if n in self.all_customers}
        robot_only_nodes = {n for n, vt in route.get_robot_route() if n in self.all_customers and vt == VehicleType.ROBOT_ONLY}
        customers.update(van_nodes)
        customers.update(robot_only_nodes)
        customers.discard(self.depot)
        return customers

    # --- Customer Removal Operators (Manual Removal) ---

    def random_customer_removal(self, solution: VRPECSolution,
                                random_state: np.random.RandomState) -> VRPECSolution:
        """Xóa một lần xuất hiện ngẫu nhiên của một khách hàng ngẫu nhiên (thủ công)."""
        destroyed = copy.deepcopy(solution)  
        assigned_customers = list(self._get_assigned_customers(destroyed))
        if not assigned_customers:
            return destroyed

        customer_to_remove = random_state.choice(assigned_customers)
        route_info = self._find_route_for_customer(destroyed, customer_to_remove)
        if route_info is None:
            return destroyed

        route_idx, route_to_modify = route_info

        # Tìm tất cả các instance của khách hàng trên route đó
        all_instances = []
        for i, (node, vt) in enumerate(route_to_modify.get_van_route()):
            if node == customer_to_remove:
                all_instances.append({'path': 'van', 'index': i, 'type': vt})
        for i, (node, vt) in enumerate(route_to_modify.get_robot_route()):
            if node == customer_to_remove:
                all_instances.append({'path': 'robot', 'index': i, 'type': vt})

        if not all_instances:
            return destroyed

        # Chọn ngẫu nhiên MỘT instance để xóa
        instance_to_remove = random_state.choice(all_instances)
        path_type = instance_to_remove['path']
        index_to_remove = instance_to_remove['index']  # Chỉ số gốc

        removed = False
        try:
            if path_type == 'van':
                temp_path = list(route_to_modify.get_van_route())
                # Kiểm tra index hợp lệ trước khi xóa
                if 0 <= index_to_remove < len(temp_path) and temp_path[index_to_remove][0] == customer_to_remove:
                    del temp_path[index_to_remove]
                    removed = True
                    cleaned_path = self._clean_route(temp_path, self.depot)
                    route_to_modify.set_van_route(cleaned_path)  # Cập nhật route trong bản sao
            elif path_type == 'robot':
                temp_path = list(route_to_modify.get_robot_route())
                # Kiểm tra index hợp lệ trước khi xóa
                if 0 <= index_to_remove < len(temp_path) and temp_path[index_to_remove][0] == customer_to_remove:
                    del temp_path[index_to_remove]
                    removed = True
                    cleaned_path = self._clean_route(temp_path, self.depot)
                    route_to_modify.set_robot_route(cleaned_path)  # Cập nhật route trong bản sao

        except (IndexError, Exception) as e:
            print(f"Error in random_customer_removal (manual) for {customer_to_remove}: {e}")

        if removed and customer_to_remove not in destroyed.unassigned_customers:
            destroyed.unassigned_customers.append(customer_to_remove)
        return destroyed

    def greedy_customer_removal(self, solution: VRPECSolution) -> VRPECSolution:
        """Xóa một lần xuất hiện ngẫu nhiên của khách hàng có delta chi phí xóa tốt nhất (thủ công)."""
        destroyed = copy.deepcopy(solution)
        assigned_customers = list(self._get_assigned_customers(destroyed))
        if not assigned_customers:
            return destroyed

        customer_deltas = {}  # {customer_id: (best_delta, route_idx, route_obj)}
        for customer in assigned_customers:
            route_info = self._find_route_for_customer(destroyed, customer)
            if route_info:
                route_idx, route_obj = route_info
                delta = self._calculate_removal_cost_delta(route_obj, customer)  # Delta mô phỏng xóa tất cả
                if delta != float('inf'):
                    if customer not in customer_deltas or delta < customer_deltas[customer][0]:
                        customer_deltas[customer] = (delta, route_idx, route_obj)

        if not customer_deltas:
            return destroyed

        best_customer_to_remove = min(customer_deltas, key=lambda c: customer_deltas[c][0])
        best_delta, route_idx, route_to_modify = customer_deltas[best_customer_to_remove]

        # Tìm lại tất cả instance của khách hàng tốt nhất trên route đó
        all_instances = []
        for i, (node, vt) in enumerate(route_to_modify.get_van_route()):
            if node == best_customer_to_remove:
                all_instances.append({'path': 'van', 'index': i, 'type': vt})
        for i, (node, vt) in enumerate(route_to_modify.get_robot_route()):
            if node == best_customer_to_remove:
                all_instances.append({'path': 'robot', 'index': i, 'type': vt})

        if not all_instances:
            return destroyed

        # Chọn ngẫu nhiên MỘT instance để xóa (dùng random chuẩn)
        instance_to_remove = random.choice(all_instances)
        path_type = instance_to_remove['path']
        index_to_remove = instance_to_remove['index']  # Chỉ số gốc

        removed = False
        try:
            if path_type == 'van':
                temp_path = list(route_to_modify.get_van_route())
                if 0 <= index_to_remove < len(temp_path) and temp_path[index_to_remove][0] == best_customer_to_remove:
                    del temp_path[index_to_remove]
                    removed = True
                    cleaned_path = self._clean_route(temp_path, self.depot)
                    route_to_modify.set_van_route(cleaned_path)
            elif path_type == 'robot':
                temp_path = list(route_to_modify.get_robot_route())
                if 0 <= index_to_remove < len(temp_path) and temp_path[index_to_remove][0] == best_customer_to_remove:
                    del temp_path[index_to_remove]
                    removed = True
                    cleaned_path = self._clean_route(temp_path, self.depot)
                    route_to_modify.set_robot_route(cleaned_path)

        except (IndexError, Exception) as e:
            print(f"Error in greedy_customer_removal (manual) for {best_customer_to_remove}: {e}")

        if removed and best_customer_to_remove not in destroyed.unassigned_customers:
            destroyed.unassigned_customers.append(best_customer_to_remove)
        return destroyed

    def customer_removal(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Chọn giữa xóa khách hàng ngẫu nhiên hoặc tham lam (thủ công)."""
        if random_state.rand() < self.beta:
            return self.random_customer_removal(solution, random_state)
        else:
            return self.greedy_customer_removal(solution)

    # --- Route Closure/Destruction Operators ---

    def random_route_closure(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Đóng (xóa) 1 route ngẫu nhiên."""
        destroyed = copy.deepcopy(solution)
        active_route_indices = [idx for idx, route in enumerate(destroyed.routes)
                                if len(route.get_van_route()) > 2 or len(route.get_robot_route()) > 2]
        if not active_route_indices:
            return destroyed

        route_idx_to_remove = random_state.choice(active_route_indices)
        # Kiểm tra lại chỉ số hợp lệ sau khi xóa (mặc dù list index thường ổn định)
        if 0 <= route_idx_to_remove < len(destroyed.routes):
            route_to_remove = destroyed.routes[route_idx_to_remove]
            customers_on_route = self._get_customers_on_route(route_to_remove)
            for customer in customers_on_route:
                if customer not in destroyed.unassigned_customers:
                    destroyed.unassigned_customers.append(customer)
            del destroyed.routes[route_idx_to_remove]  # Xóa khỏi list theo index
        return destroyed

    def greedy_route_closure(self, solution: VRPECSolution) -> VRPECSolution:
        """Đóng (xóa) 1 route có chi phí cao nhất."""
        destroyed = copy.deepcopy(solution)
        active_routes_info = {}  # {idx: cost}
        for idx, route in enumerate(destroyed.routes):
            if len(route.get_van_route()) > 2 or len(route.get_robot_route()) > 2:
                try:
                    active_routes_info[idx] = route.get_route_cost()
                except Exception:
                    continue
        if not active_routes_info:
            return destroyed

        best_route_idx = max(active_routes_info, key=active_routes_info.get)

        if 0 <= best_route_idx < len(destroyed.routes):  # Kiểm tra index trước khi truy cập
            route_to_remove = destroyed.routes[best_route_idx]
            customers_on_route = self._get_customers_on_route(route_to_remove)
            for customer in customers_on_route:
                if customer not in destroyed.unassigned_customers:
                    destroyed.unassigned_customers.append(customer)
            del destroyed.routes[best_route_idx]  # Xóa khỏi list theo index
        return destroyed

    def route_closure(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Chọn giữa đóng route ngẫu nhiên hoặc tham lam (theo chi phí)."""
        if random_state.rand() < self.gamma:
            return self.random_route_closure(solution, random_state)
        else:
            return self.greedy_route_closure(solution)

    # --- Station Visit Removal ---

    def random_station_removal(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Xóa 1 lần ghé thăm ngẫu nhiên đến 1 trạm (thủ công)."""
        destroyed = copy.deepcopy(solution)
        all_station_visits = []  # (route_idx, path_type, visit_idx, station_id)
        for route_idx, route in enumerate(destroyed.routes):
            # Kiểm tra station trên van (không phải depot đầu/cuối)
            van_path = route.get_van_route()
            for visit_idx, (node, vt) in enumerate(van_path):
                if node in self.charging_stations and visit_idx != 0 and visit_idx != len(van_path) - 1:
                    all_station_visits.append((route_idx, 'van', visit_idx, node))
            # Kiểm tra station trên robot (không phải depot đầu/cuối)
            robot_path = route.get_robot_route()
            for visit_idx, (node, vt) in enumerate(robot_path):
                if node in self.charging_stations and visit_idx != 0 and visit_idx != len(robot_path) - 1:
                    all_station_visits.append((route_idx, 'robot', visit_idx, node))

        if not all_station_visits:
            return destroyed

        chosen_visit_idx = random_state.choice(len(all_station_visits))
        route_idx, path_type, index_to_remove, station_id = all_station_visits[chosen_visit_idx]

        # Kiểm tra index hợp lệ trước khi truy cập route
        if not (0 <= route_idx < len(destroyed.routes)):
            return destroyed
        route_to_modify = destroyed.routes[route_idx]

        removed = False
        try:
            if path_type == 'van':
                temp_path = list(route_to_modify.get_van_route())
                # Kiểm tra index và node ID trước khi xóa
                if 0 <= index_to_remove < len(temp_path) and temp_path[index_to_remove][0] == station_id:
                    del temp_path[index_to_remove]
                    removed = True
                    route_to_modify.set_van_route(self._clean_route(temp_path, self.depot))
            elif path_type == 'robot':
                temp_path = list(route_to_modify.get_robot_route())
                # Kiểm tra index và node ID trước khi xóa
                if 0 <= index_to_remove < len(temp_path) and temp_path[index_to_remove][0] == station_id:
                    del temp_path[index_to_remove]
                    removed = True
                    route_to_modify.set_robot_route(self._clean_route(temp_path, self.depot))
        except (IndexError, Exception) as e:
            print(f"Error removing random station visit: {e}")
        return destroyed

    def greedy_station_removal(self, solution: VRPECSolution) -> VRPECSolution:
        """Xóa 1 lần ghé thăm trạm mang lại lợi ích chi phí lớn nhất (thủ công)."""
        destroyed = copy.deepcopy(solution)
        all_station_visits = []  # (route_idx, path_type, visit_idx, station_id)
        visit_deltas = {}  # {visit_list_idx: delta}
        current_visit_list_idx = 0
        for route_idx, route in enumerate(destroyed.routes):
            van_path = route.get_van_route()
            for visit_idx, (node, vt) in enumerate(van_path):
                if node in self.charging_stations and visit_idx != 0 and visit_idx != len(van_path) - 1:
                    visit_info = (route_idx, 'van', visit_idx, node)
                    all_station_visits.append(visit_info)
                    delta = self._calculate_single_station_visit_removal_delta(route, 'van', visit_idx)
                    if delta is not None and delta != float('inf'):
                        visit_deltas[current_visit_list_idx] = delta
                    current_visit_list_idx += 1
            robot_path = route.get_robot_route()
            for visit_idx, (node, vt) in enumerate(robot_path):
                if node in self.charging_stations and visit_idx != 0 and visit_idx != len(robot_path) - 1:
                    visit_info = (route_idx, 'robot', visit_idx, node)
                    all_station_visits.append(visit_info)
                    delta = self._calculate_single_station_visit_removal_delta(route, 'robot', visit_idx)
                    if delta is not None and delta != float('inf'):
                        visit_deltas[current_visit_list_idx] = delta
                    current_visit_list_idx += 1

        if not visit_deltas:
            return destroyed

        best_visit_list_idx = min(visit_deltas, key=visit_deltas.get)
        route_idx, path_type, index_to_remove, station_id = all_station_visits[best_visit_list_idx]

        # Kiểm tra index hợp lệ trước khi truy cập route
        if not (0 <= route_idx < len(destroyed.routes)):
            return destroyed
        route_to_modify = destroyed.routes[route_idx]

        removed = False
        try:
            if path_type == 'van':
                temp_path = list(route_to_modify.get_van_route())
                if 0 <= index_to_remove < len(temp_path) and temp_path[index_to_remove][0] == station_id:
                    del temp_path[index_to_remove]
                    removed = True
                    route_to_modify.set_van_route(self._clean_route(temp_path, self.depot))
            elif path_type == 'robot':
                temp_path = list(route_to_modify.get_robot_route())
                if 0 <= index_to_remove < len(temp_path) and temp_path[index_to_remove][0] == station_id:
                    del temp_path[index_to_remove]
                    removed = True
                    route_to_modify.set_robot_route(self._clean_route(temp_path, self.depot))
        except (IndexError, Exception) as e:
            print(f"Error removing greedy station visit: {e}")
        return destroyed

    def station_removal(self, solution: VRPECSolution, random_state: np.random.RandomState) -> VRPECSolution:
        """Chọn giữa xóa trạm (visit) ngẫu nhiên hoặc tham lam (thủ công)."""
        if random_state.rand() < self.epsilon:
            return self.random_station_removal(solution, random_state)
        else:
            return self.greedy_station_removal(solution)

    # --- Route Destruction by Customer Count ---

    def route_destruction_greedy_count(self, solution: VRPECSolution) -> VRPECSolution:
        """Phá hủy (xóa) 1 route có số lượng khách hàng ít nhất."""
        destroyed = copy.deepcopy(solution)
        active_routes_info = {}  # {idx: customer_count}
        for idx, route in enumerate(destroyed.routes):
            # Chỉ xét route có khách hàng thực sự
            customers_in_this_route = self._get_customers_on_route(route)
            if customers_in_this_route:  # Đảm bảo route có khách hàng
                active_routes_info[idx] = len(customers_in_this_route)
            # Hoặc có thể dùng: if len(route.get_van_route()) > 2 or len(route.get_robot_route()) > 2:

        if not active_routes_info:
            return destroyed

        best_route_idx = min(active_routes_info, key=active_routes_info.get)

        if 0 <= best_route_idx < len(destroyed.routes):  # Kiểm tra index
            route_to_remove = destroyed.routes[best_route_idx]
            customers_on_route = self._get_customers_on_route(route_to_remove)
            for customer in customers_on_route:
                if customer not in destroyed.unassigned_customers:
                    destroyed.unassigned_customers.append(customer)
            del destroyed.routes[best_route_idx]  # Xóa khỏi list theo index
        return destroyed
