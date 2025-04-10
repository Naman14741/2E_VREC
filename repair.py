class Repair:


    def __init__(self, unserved_customer):
        self.unserved_customer = unserved_customer

    # Repair method 1
    def route_reconstruction(self):
        """
        Ensures that the solution is feasible, the same method as initial solution
        """
        pass

    # Support for repair method 2: Customer insertion
    def _customer_insertion_random(self):
        """
        Sort un-served customers list into feasible position of 2E-VREC route until all customers have been tried.
        """
        pass

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