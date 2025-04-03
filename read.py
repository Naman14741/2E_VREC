import ast
import numpy as np


class Read:

    @staticmethod
    def parameters(path='data/vehicle-parameters.txt'):
        van_params = {}
        robot_params = {}

        with open(path, "r") as file:
            lines = file.readlines()

        current_section = None
        for line in lines:
            line = line.strip()
            if not line:
                continue

            if line.startswith("Van:"):
                current_section = van_params
            elif line.startswith("Robot:"):
                current_section = robot_params
            elif current_section is not None:
                parts = line.split(":")
                if len(parts) == 2:
                    key = parts[0].strip().lower().replace(" ", "_")
                    try:
                        value = float(parts[1].split()[0])
                        current_section[key] = value
                    except ValueError:
                        continue  # Bỏ qua dòng không hợp lệ

        return van_params, robot_params

    @staticmethod
    def read_testcase(instances='tiny', test=0):
        paths = {
            'large': 'data/large-scale instances.txt',
            'medium': 'data/medium-scale instances.txt',
            'small': 'data/small-scale instances.txt',
            'tiny': 'data/tiny-scale instances.txt'
        }

        path = paths.get(instances, 'data/tiny-scale instances.txt')

        data = {}
        try:
            with open(path, 'r', encoding='utf-8') as file:
                for line in file:
                    line = line.strip()
                    if line and line.startswith('##'):
                        key_value = line.lstrip('#').split('=', 1)
                        if len(key_value) == 2:
                            key, value = key_value[0].strip(), key_value[1].strip()
                            try:
                                data[key] = ast.literal_eval(value)
                            except (ValueError, SyntaxError):
                                data[key] = value
        except FileNotFoundError:
            raise FileNotFoundError(f"File {path} not found.")

        output = []
        for i in range(min(20, len(data.get('parking_nodes_set', [])))):
            parking_nodes = data.get('parking_nodes_set', [[]])
            customers = data.get('customers_set', [[]])
            time_windows = data.get('time_windows_set', [[]])

            parking_nodes_tuples = list(zip(parking_nodes[i][0], parking_nodes[i][1])) if len(
                parking_nodes) > i and len(parking_nodes[i]) >= 2 else []
            customers_tuples = list(zip(customers[i][0], customers[i][1])) if len(customers) > i and len(
                customers[i]) >= 2 else []
            time_windows_tuples = list(zip(time_windows[i][0], time_windows[i][1])) if len(time_windows) > i and len(
                time_windows[i]) >= 2 else []

            output.append({
                'NCust': data.get('NCust', 0),
                'NSate': data.get('NSate', 0),
                'depot': tuple(data.get('depot', ())),
                'parking_nodes_set': parking_nodes_tuples,
                'customers_set': customers_tuples,
                'time_windows_set': time_windows_tuples,
                'demands_set': data.get('demand_set', [[]])[i] if len(data.get('demand_set', [])) > i else []
            })

        test = max(0, min(test, len(output) - 1))  # Đảm bảo test nằm trong phạm vi hợp lệ
        return output[test]

    @staticmethod
    def old_input(instance='tiny', testcase=0):
        data = Read().read_testcase(instance, testcase)

        def manhattan_distance(p1, p2):
            return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

        def create_distance_matrix(data):
            # Put depot at index 0, followed by other locations
            depot_location = tuple(data['depot'])
            other_locations = data['parking_nodes_set'] + data['customers_set']
            locations = [depot_location] + other_locations
            n = len(locations)

            distance_matrix = np.zeros((n, n))
            
            # First, calculate depot distances (first row and column)
            for j in range(1, n):
                depot_to_location = manhattan_distance(depot_location, locations[j])
                distance_matrix[0][j] = depot_to_location  # Depot to location j
                distance_matrix[j][0] = depot_to_location  # Location j to depot
            
            # Then calculate distances between other locations
            for i in range(1, n):
                for j in range(1, n):
                    if i != j:
                        distance_matrix[i][j] = manhattan_distance(locations[i], locations[j])

            return distance_matrix

        distance_matrix = np.array(create_distance_matrix(data))
        depot = 0
        charging_stations = list(range(1, 1 + data['NSate']))
        customer = list(range(1 + data['NSate'], 1 + data['NSate'] + data['NCust']))
        customers_robot_only = customer[round(len(customer) * 2 / 3):]
        customers_both = customer[:round(len(customer) * 2 / 3)]

        customer_demand = {
            customer[i]: data['demands_set'][i] if i < len(data['demands_set']) else 0
            for i in range(len(customer))
        }

        time_windows = {0: (0, 8)}
        time_windows.update({idx: (0, 8) for idx in charging_stations})
        time_windows.update({customer[i]: data['time_windows_set'][i] for i in range(len(customer))})

        service_time = {0: 0}
        service_time.update({idx: 0 for idx in charging_stations})
        service_time.update({idx: 0.1 for idx in customer})

        return distance_matrix, depot, charging_stations, customers_robot_only, customers_both, customer_demand, time_windows, service_time