from enum import Enum
from typing import List, Tuple, Dict, Any


class VehicleType(Enum):
    """Enum defining the different vehicle types/modes in a VRPEC route."""
    VAN_CARRY_ROBOT = 1  # Van carrying robot
    ROBOT_CARRIED = 2    # Robot being carried by van
    ROBOT_ONLY = 3       # Robot traveling independently


class VehicleRoute:
    """
    Class representing a route for a van and robot in a VRPEC solution.
    Stores both the van's route and robot's route.
    """

    def __init__(self, 
                distance_matrix: List[List[float]],
                depot: int,
                unassigned_stations: List[int],
                customer_list: List[int],
                station_list: List[int],
                customer_demand: Dict[int, float],
                van_params: Dict[str, Any],
                robot_params: Dict[str, Any]):
        """
        Initialize a vehicle route.

        Args:
            distance_matrix: 2D array of distances between nodes
            depot: Depot node ID
            unassigned_stations: List of charging station node IDs
            customer_list: List of all customer node IDs
            station_list: List of all charging station node IDs
            customer_demand: Dictionary mapping customer ID to demand
            van_params: Dictionary of van parameters
            robot_params: Dictionary of robot parameters
        """
        self.distance_matrix = distance_matrix
        self.depot = depot
        self.unassigned_stations = unassigned_stations
        self.customer_list = customer_list
        self.station_list = station_list
        self.customer_demand = customer_demand
        self.van_params = van_params
        self.robot_params = robot_params
        
        # Initialize routes as depot-to-depot only
        self.van_route: List[Tuple[int, VehicleType]] = [
            (depot, VehicleType.VAN_CARRY_ROBOT), 
            (depot, VehicleType.VAN_CARRY_ROBOT)
        ]
        self.robot_route: List[Tuple[int, VehicleType]] = [
            (depot, VehicleType.ROBOT_CARRIED),
            (depot, VehicleType.ROBOT_CARRIED)
        ]
        
        # Route statistics
        self._cost = None
        self._distance = None

    def get_van_route(self) -> List[Tuple[int, VehicleType]]:
        """Get the van's route as a list of (node, vehicle_type) tuples."""
        return self.van_route

    def set_van_route(self, route: List[Tuple[int, VehicleType]]) -> None:
        """
        Set the van's route.
        
        Args:
            route: List of (node, vehicle_type) tuples
        """
        self.van_route = route
        self._cost = None  # Reset cached cost
        self._distance = None  # Reset cached distance

    def get_robot_route(self) -> List[Tuple[int, VehicleType]]:
        """Get the robot's route as a list of (node, vehicle_type) tuples."""
        return self.robot_route

    def set_robot_route(self, route: List[Tuple[int, VehicleType]]) -> None:
        """
        Set the robot's route.
        
        Args:
            route: List of (node, vehicle_type) tuples
        """
        self.robot_route = route
        self._cost = None  # Reset cached cost
        self._distance = None  # Reset cached distance
        
    def calculate_total_distance(self) -> float:
        """
        Calculate the total distance traveled by both van and robot.
        
        Returns:
            Total distance traveled
        """
        if self._distance is not None:
            return self._distance
            
        total_distance = 0.0
        
        # Calculate van distance
        for i in range(1, len(self.van_route)):
            prev_node = self.van_route[i-1][0]
            curr_node = self.van_route[i][0]
            if prev_node < len(self.distance_matrix) and curr_node < len(self.distance_matrix):
                total_distance += self.distance_matrix[prev_node][curr_node]
            else:
                return float('inf')  # Invalid node indexes
        
        # Calculate robot-only distance (when not carried by van)
        robot_only_segments = []
        start_idx = None
        
        for i, (node, vtype) in enumerate(self.robot_route):
            if vtype == VehicleType.ROBOT_ONLY:
                if start_idx is None:
                    # Find the starting index of this segment
                    if i > 0:
                        start_idx = i - 1
                    else:
                        start_idx = 0
            elif start_idx is not None:
                # End of a ROBOT_ONLY segment
                robot_only_segments.append((start_idx, i))
                start_idx = None
                
        # Handle case where the last segment is ROBOT_ONLY
        if start_idx is not None:
            robot_only_segments.append((start_idx, len(self.robot_route) - 1))
            
        # Calculate distances for each robot-only segment
        for start, end in robot_only_segments:
            for i in range(start + 1, end + 1):
                prev_node = self.robot_route[i-1][0]
                curr_node = self.robot_route[i][0]
                if prev_node < len(self.distance_matrix) and curr_node < len(self.distance_matrix):
                    total_distance += self.distance_matrix[prev_node][curr_node]
                else:
                    return float('inf')  # Invalid node indexes
        
        # Cache the result
        self._distance = total_distance
        return total_distance

    def get_route_cost(self) -> float:
        """
        Calculate the total cost of the route based on distance and other factors.
        
        Returns:
            Total route cost
        """
        if self._cost is not None:
            return self._cost
            
        total_distance = self.calculate_total_distance()
        if total_distance == float('inf'):
            return float('inf')
            
        # Basic cost calculation based on distance
        van_cost_rate = self.van_params.get('cost_per_distance', 1.0)
        robot_cost_rate = self.robot_params.get('cost_per_distance', 0.5)
        
        # Calculate van distance
        van_distance = 0.0
        for i in range(1, len(self.van_route)):
            prev_node = self.van_route[i-1][0]
            curr_node = self.van_route[i][0]
            if prev_node < len(self.distance_matrix) and curr_node < len(self.distance_matrix):
                van_distance += self.distance_matrix[prev_node][curr_node]
                
        # Calculate robot-only distance
        robot_only_distance = total_distance - van_distance
        
        # Calculate total cost
        total_cost = (van_distance * van_cost_rate) + (robot_only_distance * robot_cost_rate)
        
        # Additional cost factors can be added here
        # For example, time window violations, capacity constraints, etc.
        
        # Cache the result
        self._cost = total_cost
        return total_cost