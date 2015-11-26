package template;

//the list of imports
import java.util.*;
import logist.LogistSettings; 
import logist.behavior.CentralizedBehavior;
import logist.agent.Agent;
import logist.config.Parsers;
import logist.simulation.Vehicle;
import logist.plan.Plan;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

/**
 * A very simple auction agent that assigns all tasks to its first vehicle and
 * handles them sequentially.
 *
 */
@SuppressWarnings("unused")
public class CentralizedTemplate implements CentralizedBehavior {

	private Topology topology;
	private TaskDistribution distribution;
	private Agent agent;
	private long timeout_setup;
	private long timeout_plan;

	@Override
	public void setup(Topology topology, TaskDistribution distribution,
			Agent agent) {

		// this code is used to get the timeouts
		LogistSettings ls = null;
		try {
			ls = Parsers.parseSettings("config/settings_default.xml");
		} catch (Exception exc) {
			System.out.println("There was a problem loading the configuration file.");
		}

		// the setup method cannot last more than timeout_setup milliseconds
		timeout_setup = ls.get(LogistSettings.TimeoutKey.SETUP);
		// the plan method cannot execute more than timeout_plan milliseconds
		timeout_plan = ls.get(LogistSettings.TimeoutKey.PLAN);

		this.topology = topology;
		this.distribution = distribution;
		this.agent = agent;
	}

	@Override
	public List<Plan> plan(List<Vehicle> vehicles, TaskSet tasks) {
		long time_start = System.currentTimeMillis();

		PlanState plan = new PlanState(vehicles, tasks);

		List<Plan> plans = centralizedPlan(vehicles, tasks, plan);

		long time_end = System.currentTimeMillis();
		long duration = time_end - time_start;
		System.out.println("The plan was generated in " + duration + " milliseconds.");

		return plans;
	}

	/**
	 * Computes the plan for all vehicles
	 * @param vehicles set of all vehicles in world
	 * @param tasks set of all tasks to be distributed to vehicles
	 * @param plan initial plan
	 * @return a set of logist plans for all the vehicles in world
	 */
	private List<Plan> centralizedPlan(List<Vehicle> vehicles, final TaskSet tasks, PlanState plan) {
		initSolution(vehicles, tasks, plan);
		long time_start = System.currentTimeMillis();
		double min = Integer.MAX_VALUE;
		PlanState bestPlan = new PlanState(plan);
		int counter = 0;
		double planCostPrev;
		double planCost;
		int ind;
		List<PlanState> neighbours = new ArrayList<PlanState>();
		//cost and load initialized
		for(Vehicle v: vehicles) {
			updateLoad(plan, v);
		}
		
		for (int i = 0; i < 500; i++) {
			planCostPrev = plan.getCost();
			neighbours = ChooseNeighbours(plan, tasks, vehicles);
			if(new Random().nextInt(100) <= 80) {
				plan = localChoice(neighbours);
			}
			planCost = plan.getCost();
			if(planCostPrev == planCost){
				counter ++;
				if(counter == 3) {
					ind = new Random().nextInt(neighbours.size());
					plan = neighbours.get(ind);
					counter = 0;
				}
			}
			
			if(System.currentTimeMillis()-time_start > this.timeout_plan) {
				System.out.println("time out centralized plan");
				break;
			}
			planCost = plan.getCost();
			if(planCost < min) {
				bestPlan = new PlanState(plan);
				min = planCost;
			}
			System.out.println(planCost);
		}
		
		System.out.println("Cost: "+bestPlan.getCost());
		for(Vehicle v: vehicles)System.out.println(bestPlan.getVTasks(v).size());
		List<Plan> vplans = new ArrayList<Plan>();

		Helper helper = new Helper() {
			@Override
			public Plan buildPlan(PlanState state, Vehicle v) {
				Integer next = state.getFirstPickup()[v.id()];
				if(next != null) {
					Task t = getTask(tasks, next);
					City current = v.homeCity();
					Plan p = new Plan(current);
					Integer[] times = new Integer[state.getTimeP().length + state.getTimeD().length];
					System.arraycopy(state.getTimeP(), 0, times, 0, state.getTimeP().length);
					System.arraycopy(state.getTimeD(), 0, times, state.getTimeP().length, state.getTimeD().length);
					ArrayIterator it = new ArrayIterator(times, state.getVTasks().get(v.id()));

					while(it.hasNext()) {
						int time = it.next();
						int pickupIndex = findIndex(v, state, state.getTimeP(), time);
						int deliverIndex = findIndex(v, state, state.getTimeD(), time);
						
						if(pickupIndex != -1) {
							for (City c: current.pathTo(getTask(tasks, pickupIndex).pickupCity)){
								p.appendMove(c);
							}
							current = getTask(tasks, pickupIndex).pickupCity;
							p.appendPickup(getTask(tasks, pickupIndex));

						} else {
							for (City c: current.pathTo(getTask(tasks, deliverIndex).deliveryCity)){
								p.appendMove(c);
							}
							current = getTask(tasks, deliverIndex).deliveryCity;
							p.appendDelivery(getTask(tasks, deliverIndex));
					
						}
					}
					return p;
				}
				else {
					return Plan.EMPTY;
				}
			}
		};

		for (Vehicle v: vehicles) {
			vplans.add(v.id(), helper.buildPlan(bestPlan, v));
		}
		return vplans;
	}

	/**
	 * Provided in template
	 * @param vehicle single vehicle
	 * @param tasks tasks
	 * @return a plan with all tasks for a single vehicle
	 */
	private Plan naivePlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity)) {
				plan.appendMove(city);
			}

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path()) {
				plan.appendMove(city);
			}

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}
		return plan;
	}

	/**
	 * Initialises the plan distributing tasks among the vehicles
	 * @param vehicles Set of vehicles
	 * @param tasks set of tasks
	 * @param plan empty plan
	 */
	private void initSolution(List<Vehicle> vehicles, TaskSet tasks, PlanState plan) {
		int tasksPerVehicule = (int) Math.floor(tasks.size()/vehicles.size());
		int lastVehicule = tasksPerVehicule + tasks.size()%vehicles.size();
		
		for (Vehicle v : vehicles) {
			int t = 0;
			int beginTask;
			int lastTask;
			if(v.id() != vehicles.size()-1 && v.id() != 0){
				beginTask = v.id()*tasksPerVehicule;
				lastTask = (v.id()+1)*tasksPerVehicule;
			}
			else if(v.id()==0){
				beginTask = 0;
				lastTask = tasksPerVehicule;
			}
			else{
				beginTask = v.id()*tasksPerVehicule;
				lastTask = v.id()*tasksPerVehicule + lastVehicule;
			}
			plan.getFirstPickup()[v.id()] = beginTask;
			for(int i = beginTask; i < lastTask; i++){
				plan.addVTasks(v.id(), i);
				plan.getTimeP()[i] = t;
				plan.getTimeD()[i] = t+1;
				t+=2;
			}
			updateLoad(plan, v);
		}
	}

	/**
	 * Chooses best option (lowest cost) in list of neighbours
	 * @param neighbours list of neighbours from current state
	 * @return Best plan option of this iteration
	 */
	private PlanState localChoice(List<PlanState> neighbours) {
		PlanState bestPlan = null;
		double min = Double.MAX_VALUE;
		double cost;
		for(PlanState neighbour: neighbours) {
			cost = neighbour.getCost();
			if(cost < min){
				bestPlan = neighbour;
				min = cost;
			}
		}
		return bestPlan;
	}

	/**
	 * Computes all PlanState neighbours from given PlanState by exchanging tasks between vehicles, and permuting within a vehicle
	 * @param plan current plan
	 * @param tasks set of all tasks
	 * @param vehicles set of all vehicles
	 * @return a list of PlanStates that are neighbours of the current plan
	 */
	private List<PlanState> ChooseNeighbours(PlanState plan, TaskSet tasks, List<Vehicle> vehicles){
		List<PlanState> neighbours = new ArrayList<PlanState>();
		Vehicle v;

		//Pick random vehicle
		int nTasks;
		do {
			v = vehicles.get(new Random().nextInt(vehicles.size()));
			nTasks = plan.getVTasks(v).size();
		} while(nTasks < 1);

		// Change first task with all other vehicles
		Integer task = plan.getFirstPickup()[v.id()];
		for(Vehicle v2: vehicles) {
			if(v.id() != v2.id() && getTask(tasks, task).weight <= v2.capacity()){
				neighbours.addAll(changeVehicle(v, v2, task, plan));
			}		
		}
//		int vTasks;
//		int t;
//		for(Vehicle v1: vehicles){
//			if(plan.getVTasks(v1).size() > 0) {
//				for(Vehicle v2: vehicles) {
//					t = plan.getFirstPickup()[v1.id()];
//					if(v1.id() != v2.id() && getTask(tasks, t).weight <= v2.capacity()){
//						neighbours.addAll(changeVehicle(v1, v2, t, plan));
//					}		
//				}
//			}
//			neighbours.addAll(changeTaskOrder(v1, plan));
//		}
		
		// Changing task order 
		neighbours.addAll(changeTaskOrder(v, plan));
		return neighbours;
	}

	/**
	 * Creates neighbours with permuted task orders for a given vehicle
	 * @param v1 given vehicle
	 * @param plan current plan
	 * @return neighbours of the current plan with permutations over task set of vehicle v1
	 */
	private List<PlanState> changeTaskOrder(Vehicle v1, PlanState plan) {
		List<PlanState> neighbours = new ArrayList<PlanState>();
		int arraySize =  plan.getVTasks(v1).size();
		//sort the hash set to a list
		ArrayList<Integer> tasks = new ArrayList<Integer>(new TreeSet<Integer>(plan.getVTasks(v1)));

		for(int i = 0; i < arraySize*2-1; i++) {
			for(int j = i+1; j < arraySize*2; j++) {
				PlanState neighbour = new PlanState(plan);
				// exchange two pickups
				if(i < arraySize && j < arraySize) {
					// update first pickup
					if(neighbour.getTimeP()[tasks.get(i)] == 0) {
						neighbour.getFirstPickup()[v1.id()] = tasks.get(j);
					}
					else if(neighbour.getTimeP()[tasks.get(j)] == 0) {
						neighbour.getFirstPickup()[v1.id()] = tasks.get(i);
					}
					int t1 = neighbour.getTimeP()[tasks.get(i)];
					neighbour.getTimeP()[tasks.get(i)] = neighbour.getTimeP()[tasks.get(j)];
					neighbour.getTimeP()[tasks.get(j)] = t1;
				}
				// exchange a pickup and a deliver
				else if(i < arraySize && j >= arraySize) {
					//update next pickup
					if(neighbour.getTimeP()[tasks.get(i)] == 0){
						neighbour.getFirstPickup()[v1.id()] = tasks.get(j-arraySize);
					}
					int t1 = neighbour.getTimeP()[tasks.get(i)];
					neighbour.getTimeP()[tasks.get(i)] = neighbour.getTimeD()[tasks.get(j-arraySize)];
					neighbour.getTimeD()[tasks.get(j-arraySize)] = t1;
				}
				// exchange two pickups
				else {
					int t1 = neighbour.getTimeD()[tasks.get(i-arraySize)];
					neighbour.getTimeD()[tasks.get(i-arraySize)] = neighbour.getTimeD()[tasks.get(j-arraySize)];
					neighbour.getTimeD()[tasks.get(j-arraySize)] = t1;
				}
				// check that the time constraint and load constraint are satisfied
				if(checkTimes(neighbour.getTimeP(), neighbour.getTimeD(), plan.getVTasks(v1))
						&& updateLoad(neighbour, v1)){
					neighbours.add(neighbour);
				}
			}
		}
		// let's shuffle
		int counter = 0;
		PlanState neighbour = new PlanState(plan);
		ArrayList<Integer> shuffleTimes = new ArrayList<Integer>();
		for(int i = 0; i < arraySize; i++) {
			shuffleTimes.add(neighbour.getTimeP()[tasks.get(i)]);
			shuffleTimes.add(neighbour.getTimeD()[tasks.get(i)]);
		}
		while(counter < 3) {
			neighbour = new PlanState(plan);
			Collections.shuffle(shuffleTimes);
			for(int i = 0; i < arraySize; i++) {
				neighbour.getTimeP()[tasks.get(i)] = shuffleTimes.get(i);
				neighbour.getTimeD()[tasks.get(i)] = shuffleTimes.get(i+arraySize);
			}
			if(checkTimes(neighbour.getTimeP(), neighbour.getTimeD(), plan.getVTasks(v1))
					&& updateLoad(neighbour, v1)){
				boolean found = false;
				for (Integer i: neighbour.getVTasks(v1)){  // Update nextPickup for v1 to the task after the one just removed
		        	if (neighbour.getTimeP()[i] == 0){
		                neighbour.getFirstPickup()[v1.id()] = i;
		                found = true;
		            }
		        }
		        if(!found) {
		        	neighbour.getFirstPickup()[v1.id()] = null ;
		        }
				neighbours.add(neighbour);
				counter++;
			}
		}
		//System.out.println(counter);
		return neighbours;
	}

	/**
	 * Checks the validity of pickup and delivery time arrays
	 * @param pickup array for pickup times of tasks, with each id = task_id and pickup[id] the time at which it is picked up
	 * @param delivery array for delivery times of tasks, idem pickup
	 * @param tasks task ids to be to be checked, i.e. the tasks belonging to a specific vehicle's route
	 * @return true if all pickups of tasks happen before their delivery, else false
	 */
	private boolean checkTimes(Integer[] pickup, Integer[] delivery, HashSet<Integer> tasks) {
		for(Integer t: tasks) {
			if(pickup[t] > delivery[t]) return false;
		}
		return true;
	}

	/**
	 * Updates the load values of a plan for a given vehicle
	 * @param plan to be updated
	 * @param vehicle for which the task-permutation has changed
	 * @return true if load is legal, else false
	 */
	private boolean updateLoad(PlanState plan, Vehicle vehicle) {
		int vID = vehicle.id();
		int cost = 0;
		Integer next = plan.getFirstPickup()[vID];
		if(next != null) {
			Integer[] times = new Integer[plan.getTimeP().length + plan.getTimeD().length];
			System.arraycopy(plan.getTimeP(), 0, times, 0, plan.getTimeP().length);
			System.arraycopy(plan.getTimeD(), 0, times, plan.getTimeP().length, plan.getTimeD().length);
			ArrayIterator it = new ArrayIterator(times, plan.getVTasks().get(vehicle.id()));
	
			for(int i = 0; i < 2*plan.tasks.size(); i++) {
				plan.getLoad()[vID][i] = 0;
			}
			
			int load = 0;
			City current = vehicle.homeCity();
			while(it.hasNext()) {
				int time = it.next();
				int pickupIndex = findIndex(vehicle, plan, plan.getTimeP(), time);
				int deliverIndex = findIndex(vehicle, plan, plan.getTimeD(), time);
				
				if(pickupIndex != -1) {
					load += getTask(plan.tasks, pickupIndex).weight;
					plan.getLoad()[vID][time] = load;
					cost += current.distanceTo(getTask(plan.tasks, pickupIndex).pickupCity)*vehicle.costPerKm();
					current = getTask(plan.tasks, pickupIndex).pickupCity;
					if(load > vehicle.capacity()) return false;
	
				} else {
					load -= getTask(plan.tasks, deliverIndex).weight;
					plan.getLoad()[vID][time] = load;
					cost += current.distanceTo(getTask(plan.tasks, deliverIndex).deliveryCity)*vehicle.costPerKm();
					current = getTask(plan.tasks, deliverIndex).deliveryCity;
				}
			}
		}
		plan.cost[vehicle.id()] = cost;
		return true;

	}

	private Integer findIndex(Vehicle v, PlanState plan, Integer[] times, Integer t) {
		for(int i = 0; i < times.length; i++) {
			if(times[i] == t && plan.getVTasks().get(v.id()).contains(i)) return i;
		}
		return -1;
	}

	/**
	 * Moves first task in set of v1 to first task in set of v2 and makes necessary updates
	 * @param v1 first vehicle
	 * @param v2 second vehicle
	 * @param task task id
	 * @param plan current plan
	 * @return List of neighbours with task changed and all possible delivery times
	 */
	private List<PlanState> changeVehicle(Vehicle v1, Vehicle v2, Integer task, PlanState plan) {
		List<PlanState> neighbours = new ArrayList<PlanState>();
		PlanState neighbour = new PlanState(plan);
		neighbour.removeVTasks(v1.id(), task);
		neighbour.addVTasks(v2.id(), task);
		
		int deliverTask = neighbour.getTimeD()[task];

		// update times v1 after removing first task
		for(Integer i: neighbour.getVTasks().get(v1.id())) {
			if(neighbour.getTimeP()[i]>deliverTask) neighbour.getTimeP()[i] -= 1;
			if(neighbour.getTimeD()[i]>deliverTask) neighbour.getTimeD()[i] -= 1;
			neighbour.getTimeP()[i] -= 1;
			neighbour.getTimeD()[i] -= 1;
		}
		
		// update time of moved task
		neighbour.getTimeP()[task] = 0;
		
		// update the next pickup
		neighbour.getFirstPickup()[v2.id()] = task;
		
		boolean found = false;
		for (Integer i: neighbour.getVTasks(v1)){  // Update nextPickup for v1 to the task after the one just removed
        	if (neighbour.getTimeP()[i] == 0){
                neighbour.getFirstPickup()[v1.id()] = i;
                found = true;
            }
        }
        if(!found) {
        	neighbour.getFirstPickup()[v1.id()] = null ;
        }
		
		// try to add a deliver until it is not possible anymore
		int t = 1;
		boolean canTake = true;
		
		while(t < 2*neighbour.getVTasks().get(v2.id()).size() && canTake){
			if(neighbour.getLoad()[v2.id()][t-1]+getTask(neighbour.tasks, task).weight > v2.capacity()) canTake = false;
			PlanState newNeighbour = new PlanState(neighbour);
			// update times v2 after adding the task
			for(Integer i: neighbour.getVTasks().get(v2.id())) {
				if(i != task){
					newNeighbour.getTimeP()[i] += 1;
					newNeighbour.getTimeD()[i] += 1;
					if(newNeighbour.getTimeP()[i] >= t)newNeighbour.getTimeP()[i] += 1;
					if(newNeighbour.getTimeD()[i] >= t)newNeighbour.getTimeD()[i] += 1;
				}
			}
			newNeighbour.getTimeD()[task] = t;
			newNeighbour.getTimeP()[task] = 0;
			
	        if(newNeighbour.getVTasks().get(v1.id()).size() == 0){
		    	if(updateLoad(newNeighbour, v2)) {
		    		neighbours.add(newNeighbour);
		    	}
	        }
        
	        else if(updateLoad(newNeighbour, v1) && updateLoad(newNeighbour, v2)){	 
	        	neighbours.add(newNeighbour);
	        }
			t++;
		}
		return neighbours;
	}

	/**
	 * Gets the Task from set with task_id id
	 * @param tasks set
	 * @param id id
	 * @return task with id
	 */
	private Task getTask(TaskSet tasks, int id) {
		for (Task t : tasks) {
			if (t.id == id) {
				return t;
			}
		}
		return null;
	}

	/**
	 * State implementation
	 */
	public class PlanState {

		private Integer[] firstPickup; 
		private Integer[] timeP; // [p0, p1, ..., pn]
		private Integer[] timeD; // [d0, d1, ..., dn]
		private int[][] load;
		private List<Vehicle> vehicles;
		private TaskSet tasks;
		private Map<Integer, HashSet<Integer>> vTasks = new HashMap<Integer, HashSet<Integer>>(); // Map from vehicle_id to Set of tasks in vehicle's track
		//private double cost;
		private double[] cost;
		
		/**
		 * Constructor for empty PlanState
		 * @param vehicles
		 * @param tasks
		 */
		public PlanState(List<Vehicle> vehicles, TaskSet tasks) {
			cost = new double[vehicles.size()];
			firstPickup = new Integer[vehicles.size()];
			timeP = new Integer[tasks.size()];
			for(int i = 0; i < tasks.size(); i++)timeP[i] = 0;
			timeD = new Integer[tasks.size()];
			for(int i = 0; i < tasks.size(); i++)timeD[i] = 0;
			load = new int[vehicles.size()][2 * tasks.size()];
			this.vehicles = vehicles;
			this.tasks = tasks;
			for(Vehicle v: vehicles) vTasks.put(v.id(), new HashSet<Integer>());
		}

		/**
		 * Constructor that copies a plan
		 * @param p plan to be copied to new plan
		 */
		public PlanState(PlanState p) {
			this.cost = p.cost.clone();
			firstPickup = p.getFirstPickup().clone();
			timeP = p.getTimeP().clone();
			timeD = p.getTimeD().clone();
			this.load = new int[p.vehicles.size()][2 * p.tasks.size()];
			for(int i = 0; i < p.vehicles.size(); i++) {
				for(int j = 0; j < p.tasks.size()*2; j++) {
					this.load[i][j] = p.getLoad()[i][j];
				}
			}
			this.tasks = p.tasks;
			this.vehicles = p.vehicles;
			//Copy of HashSet values in Map
			for(Vehicle v: vehicles) vTasks.put(v.id(), new HashSet<Integer>());
			for(Map.Entry<Integer, HashSet<Integer>> e: p.getVTasks().entrySet()) {
				for(Integer i: e.getValue()) {
					vTasks.get(e.getKey()).add(new Integer(i));
				}
			}
		}

		public Integer[] getFirstPickup() {
			return firstPickup;
		}

		public Integer[] getTimeP() {
			return timeP;
		}

		public Integer[] getTimeD() {
			return timeD;
		}

		public int[][] getLoad() {
			return load;
		}

		/**
		 * @return Map of sets for all vehicles
		 */
		public Map<Integer, HashSet<Integer>> getVTasks() {
			return vTasks;
		}

		/**
		 * @param v vehicle
		 * @return set for given vehicle v
		 */
		public HashSet<Integer> getVTasks(Vehicle v) {
			return vTasks.get(v.id());
		}

		public void addVTasks(Integer v, Integer t) {
			vTasks.get(v).add(t);
		}

		public void removeVTasks(Integer v, Integer t) {
			vTasks.get(v).remove(t);
		}
		
		public double getCost() {
			double costTot = 0;
			for(int i = 0; i < vehicles.size(); i++) {
				costTot += cost[i];
			}
			return costTot;
		}

	}

	/**
	 * Iterates over an int array from min to max given the tasks of a vehicle
	 */
	class ArrayIterator implements Iterator<Object> {

		private List<Integer> l;
		private HashSet<Integer> tasks = new HashSet<Integer>();

		ArrayIterator(Integer[] time, HashSet<Integer> t) {
			l = new ArrayList<Integer>(Collections.nCopies(time.length, -1));
			for (Integer i: t) {
				tasks.add(new Integer(i));
			}
			for(Integer ti: tasks) {
				l.set(ti, time[ti]);
				l.set(ti + time.length/2, time[ti + time.length/2]);
			}
		}

		@Override
		public boolean hasNext() {
			for (Integer i: tasks) {
				if(l.get(i) != -1 || l.get(i+l.size()/2) != -1) return true;
			}
			return false;
		}

		@Override
		public Integer next() {
			int min = Integer.MAX_VALUE;
			Integer index = -1;
			
			for(Integer t: tasks) {
				if(l.get(t) != -1 && l.get(t) < min) {
					min = l.get(t);
					index = t;
				}
				if(l.get(t+l.size()/2) != -1 && l.get(t+l.size()/2) < min) {
					 min = l.get(t+l.size()/2);
					 index = t+l.size()/2;
				}
			}

			l.set(index, -1); //remove
			return min;
		}

		@Override
		public void remove() {

		}
	}

	interface Helper {
		/**
		 * Builds the plan for a given vehicle from the state achieved through optimisation
		 * @param state achieved through n iterations
		 * @param v a vehicle from the list
		 * @return plan for vehicle v
		 */
		Plan buildPlan(PlanState state, Vehicle v);
	}
}


