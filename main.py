from alns import ALNS
from alns.accept import SimulatedAnnealing
from alns.select import RouletteWheel
from alns.stop import MaxIterations
from solution import VRPECSolution
from repair import Repair
from destroy import Destroy
from read import Read
import numpy.random as rnd
import numpy as np

SEED = 1234

van_params, robot_params = Read().parameters()
distance_matrix, depot, charging_stations, customers_robot_only, customers_both, customer_demand, time_windows, service_time = Read().old_input(instance='large', testcase=2)

problem = VRPECSolution(
    distance_matrix=distance_matrix,
    depot=depot,
    charging_stations=charging_stations,
    customers_robot_only=customers_robot_only,
    customers_both=customers_both,
    van_params=van_params,
    robot_params=robot_params,
    customer_demand=customer_demand,
    time_windows=time_windows,
    service_times=service_time
)

initial_solution = problem.initial_solution()

# Create the ALNS algorithm
alns = ALNS(rnd.default_rng(SEED))

destroy = Destroy(problem)

# Add the destroy operator
alns.add_destroy_operator(destroy.customer_removal)

# Add the repair operator
alns.add_repair_operator(Repair.route_reconstruction)

# Selection and acceptance criteria
select = RouletteWheel([27, 7, 4, 0], 0.99, 1, 1)

accept = SimulatedAnnealing(
    start_temperature=500,
    end_temperature=1,
    step=0.99,
    method="exponential"
)

# Stop criteria
stop = MaxIterations(10000)

# Run the ALNS algorithm
result = alns.iterate(initial_solution, select, accept, stop)

best_solution = result.best_state

print("Best solution found:", best_solution.objective())