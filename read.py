import ast


class Read:


    @staticmethod
    def parameters(path):
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
                    value = float(parts[1].split()[0])
                    current_section[key] = value

        return van_params, robot_params

    @staticmethod
    def testcase(instances='tiny', test=0):
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
        for i in range(20):
            parking_nodes = data.get('parking_nodes_set', [[]])[i]
            customers = data.get('customers_set', [[]])[i]
            time_windows = data.get('time_windows_set', [[]])[i]

            # Chuyển đổi danh sách thành danh sách tuple có hai phần tử từ hai danh sách con
            parking_nodes_tuples = list(zip(parking_nodes[0], parking_nodes[1])) if len(parking_nodes) >= 2 else []
            customers_tuples = list(zip(customers[0], customers[1])) if len(customers) >= 2 else []
            time_windows_tuples = list(zip(time_windows[0], time_windows[1])) if len(time_windows) >= 2 else []

            output.append({
                'NCust': data.get('NCust', 0),
                'NSate': data.get('NSate', 0),
                'depot': data.get('depot', []),
                'parking_nodes_set': parking_nodes_tuples,
                'customers_set': customers_tuples,
                'time_windows_set': time_windows_tuples,
                'demands_set': data.get('demands_set', [[]])[i]
            })

        test = max(test, 0)
        test = min(test, 19)
        return output[test]