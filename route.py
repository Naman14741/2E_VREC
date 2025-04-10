from typing import List, Dict

class Route:
    """
    Represents a van and robot route of vehicle number k in 2E-VREC problem
    """

    def __init__(self):
        # List of node for van k
        self._van_route: List[int] = []
        # The power charged for van k at node i
        self._van_charged: Dict[int, float] = {}

        # List of node for robot k
        self._robot_route: List[int] = []
        # The power charged for robot k at node i
        self._robot_charged: Dict[int, float] = {}
