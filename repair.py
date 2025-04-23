import numpy as np

from route import VehicleRoute
from solution import VRPECSolution
from support import *


class Repair:
    # Repair method 1
    @staticmethod
    def route_reconstruction(solution: VRPECSolution):
        """
        Ensures that the solution is feasible, the same method as initial solution
        """
        repaired = solution.copy()
        while repaired.unassigned_customers:
            np.random.shuffle(repaired.unassigned_customers)
            first_customer = repaired.unassigned_customers[0]
            repaired.unassigned_customers.remove(first_customer)

            # Create a new route for the first customer
            route = VehicleRoute(
                distance_matrix=repaired.distance_matrix,
                depot=repaired.depot,
                unassigned_stations=repaired.charging_stations.copy(),
                station_list=repaired.charging_stations,
                customer_list=repaired.all_customers,
                customer_demand=repaired.customer_demand
            )

            if first_customer in repaired.customers_robot_only:
                # Customer can only be served by robot
                station = \
                    station_nearest_customer(repaired.distance_matrix, route.get_unassigned_stations(), first_customer)[0]
                route.open_robot_route(first_customer, station)
            else:  # Customer can be served by either van or robot
                p = np.random.rand()
                if p < 0.5:
                    # Open van route
                    route.open_van_route(first_customer)
                else:
                    # Open robot route
                    station = \
                        station_nearest_customer(repaired.distance_matrix, route.get_unassigned_stations(), first_customer)[
                            0]
                    route.open_robot_route(first_customer, station)

            # Create a copy of unassigned customers to iterate over
            customers_to_check = repaired.unassigned_customers.copy()
            np.random.shuffle(customers_to_check)

            for next_customer in customers_to_check:
                if next_customer in repaired.customers_robot_only:
                    if route.insert_customer_robot_into_route(next_customer):
                        repaired.unassigned_customers.remove(next_customer)
                else:
                    p = np.random.rand()
                    if p < 0.5:
                        if route.insert_customer_van_into_route(next_customer):
                            repaired.unassigned_customers.remove(next_customer)
                    else:
                        if route.insert_customer_robot_into_route(next_customer):
                            repaired.unassigned_customers.remove(next_customer)

            # Add the route to the solution
            repaired.routes.append(route)

    # Support for repair method 2: Customer insertion
    @staticmethod
    def _customer_insertion_random(self, solution: VRPECSolution):
        """
        Sort un-served customers list into feasible position of 2E-VREC route until all customers have been tried.
        """
        repaired = solution.copy()
        while repaired.unassigned_customers:
            np.random.shuffle(repaired.unassigned_customers)
            # Choose a random route to insert
            vehicle_id = np.random.randint(0, len(repaired.routes) -1)


    # Support for repair method 2: Customer insertion
    def _customer_insertion_greedy(self):
        """
        Sort un-served customers list into feasible position and choose a customer to insert into the 2E-VREC route
        with the least cost increases.
        """
        pass

    # Support for repair method 2: Customer insertion
    def _nearest_station_insertion(self):
        """
        Nearest station insertion is used after we have inserted a customer node and found the route is unfeasible:
        we try to insert a station before or after the inserted customer position.
        """
        pass

    # Repair method 2: Customer insertion
    def customer_insertion(self):
        """
        Use random/greedy customer insertion moves to insert customer nodes.
        If the route is unfeasible due to battery constraints, use nearest station insertion.
        If there are still un-served customers, use Repair Method 1: Route Reconstruction.
        """
        pass

    # Support for repair method 3.1: Route structure change ﬁrst/ customer insert second operation
    def open_van_route(self):
        """
        Open a van route begin from a connected station to customers and finish in ame one or a different one
        """
        pass

    # Support for repair method 3.1: Route structure change ﬁrst/ customer insert second operation
    def open_robot_route(self):
        """
        Open a robot route begin from a connected station to customers and finish in ame one or a different one
        """
        pass

    # Repair method 3.1: Route structure change ﬁrst/ customer insert second operation
    def open_route_customer_insert(self):
        """
        Open van/robot route then Customer insertion operator
        """
        pass

    # Support for repair method 3.2: Route structure change ﬁrst/ customer insert second operation
    def station_insertion_random(self):
        """
        Chooses a position in the 2E-VREC route to insert a station.
        """
        pass

    # Support for repair method 3.2: Route structure change ﬁrst/ customer insert second operation
    def station_insertion_greedy(self):
        """
        Chooses the best insertion position with feasible and least total travel cost increases to insert a station.
        """
        pass

    # Repair method 3.2: Route structure change ﬁrst/ customer insert second operation
    def station_insert_customer_insert(self):
        """
        Insert a station then insert into a 2E-VREC route then Customer insertion operator
        """
        pass