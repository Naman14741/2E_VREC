class Destroy:
    """
    Input is a feasible solution
    :return: A solution unfeasible to repair operator for a better solution
    """

    def __init__(self, beta=0.3):
        # Customer destroy rate
        self.beta = beta

    def customer_removal_random(self):
        """
        Random customer removal randomly removes a customer node.
        """
        pass

    def customer_removal_greedy(self):
        """
        Greedy customer removal removes the customer that can yield the largest cost reduction for a given route.
        """
        pass

    def station_removal_random(self):
        """
        Charging station removal randomly removes a charging station.
        """
        pass

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