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

//		System.out.println("Agent " + agent.id() + " has tasks " + tasks);
        PlanState plan = new PlanState(vehicles, tasks);

        List<Plan> plans = centralizedPlan(vehicles, tasks, plan);

        //Plan planVehicle1 = naivePlan(vehicles.get(0), tasks);

//        List<Plan> plans = new ArrayList<Plan>();
//        plans.add(planVehicle1);
//        while (plans.size() < vehicles.size()) {
//            plans.add(Plan.EMPTY);
//        }

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
        for (int i = 0; i < 10000; i++) {

            if(new Random().nextInt(100) < 40) {
        		 List<PlanState> neighbours = ChooseNeighbours(plan, tasks, vehicles);
                 plan = localChoice(neighbours);
        	}  
        }

        List<Plan> vplans = new ArrayList<Plan>();

        Helper helper = new Helper() {
            @Override
            public Plan buildPlan(PlanState state, Vehicle v) {
                Integer next = state.getNextPickup()[v.id()];
                Task t = getTask(tasks, next);
                City current = v.homeCity();
                Plan p = new Plan(current);
                ArrayIterator itP = new ArrayIterator(state.getTimeP(), state.getVTasks().get(v.id()));
                ArrayIterator itD = new ArrayIterator(state.getTimeD(), state.getVTasks().get(v.id()));
                while(itP.hasNext() || itD.hasNext()) {
                    int pickupTime = Integer.MAX_VALUE;
                    int pickupIndex = -1;
                    int deliverTime = Integer.MAX_VALUE;
                    int deliverIndex = -1;
                    if(itP.hasNext()) {
                        pickupTime = itP.next();
                        pickupIndex = itP.indexOf(pickupTime);
                    }
                    if(itD.hasNext()) {
                        deliverTime = itD.next();
                        deliverIndex = itD.indexOf(deliverTime);
                    }
                    if(pickupTime < deliverTime) {
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
        };

        for (Vehicle v: vehicles) {
            vplans.add(v.id(), helper.buildPlan(plan, v));
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
     * Initialises the plan with all tasks given to the vehicle with biggest capacity
     * @param vehicles Set of vehicles
     * @param tasks set of tasks
     * @param plan empty plan
     */
    private void initSolution(List<Vehicle> vehicles, TaskSet tasks, PlanState plan) {
        int maxCap = 0;
        int index = -1;
        for (Vehicle v : vehicles) {
            if (v.capacity() > maxCap) {
                maxCap = v.capacity();
                index = v.id();
            }
        }
        Vehicle v = vehicles.get(index);
        int t = 0;
        Iterator<Task> it = tasks.iterator();
        int prev = it.next().id;
        
        plan.getNextPickup()[index] = prev;
        plan.addVTasks(index, prev);
        plan.getTimeP()[prev] = t;
        plan.getLoad()[index][t] = getTask(tasks, prev).weight;
        plan.getTimeD()[prev] = ++t;
        plan.getLoad()[index][t] = 0; // task has been delivered

        while (it.hasNext()) {
            int next = it.next().id;
            plan.addVTasks(index, next);
            plan.getLoad()[index][t] = getTask(tasks, prev).weight; // 0 or weight in load in initialised state.
            plan.getTimeP()[next] = ++t;
            plan.getTimeD()[next] = ++t;
            plan.getLoad()[index][t] = 0;
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
    	
    	for(PlanState neighbour: neighbours) {
    		double cost = 0;
    		for(Vehicle v: neighbour.vehicles) {
    			 Integer next = neighbour.getNextPickup()[v.id()];
                 Task t = getTask(neighbour.tasks, next);
                 City current = v.homeCity();
                 ArrayIterator itP = new ArrayIterator(neighbour.getTimeP(), neighbour.getVTasks().get(v.id()));
                 ArrayIterator itD = new ArrayIterator(neighbour.getTimeD(), neighbour.getVTasks().get(v.id()));
                 while(itP.hasNext() || itD.hasNext()) {
                     int pickupTime = Integer.MAX_VALUE;
                     int pickupIndex = -1;
                     int deliverTime = Integer.MAX_VALUE;
                     int deliverIndex = -1;
                     if(itP.hasNext()) {
                         pickupTime = itP.next();
                         pickupIndex = itP.indexOf(pickupTime);
                     }
                     if(itD.hasNext()) {
                         deliverTime = itD.next();
                         deliverIndex = itD.indexOf(deliverTime);
                     }
                     if(pickupTime < deliverTime) {
                         cost += current.distanceTo(getTask(neighbour.tasks, pickupIndex).pickupCity)*v.costPerKm();
                         current = getTask(neighbour.tasks, pickupIndex).pickupCity;
               
                     } else {
                         cost += current.distanceTo(getTask(neighbour.tasks, deliverIndex).deliveryCity)*v.costPerKm();
                         current = getTask(neighbour.tasks, deliverIndex).deliveryCity;
                     }
                 }
    		}
    		if(cost < min) {
    			min = cost;
    			bestPlan = neighbour;
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
    	Vehicle v1;
        System.out.println("CentralizedTemplate.ChooseNeighbours");

        // Pick random vehicle
    	int vTasks;
    	do {
    		v1 = vehicles.get(new Random().nextInt(vehicles.size()));
    		vTasks = plan.getVTasks().get(v1.id()).size();
    	} while(vTasks < 1);
    	
    	// Change first task with all other vehicles
    	Integer task = plan.getNextPickup()[v1.id()];
    	for(Vehicle v2: vehicles) {
            if(v1.id() != v2.id() && getTask(tasks, task).weight <= v2.capacity())
                neighbours.addAll(changeVehicle(v1, v2, task, plan));
    	}
    	
    	// Changing task order 
    	neighbours.addAll(changeTaskOrder(v1, plan));
    	return neighbours;
    }

    /**
     * Creates neighbours with permuted task orders for a given vehicle
     * @param v1 given vehicle
     * @param plan current plan
     * @return neighbours of the current plan with permutations over task set of vehicle v1
     */
    private List<PlanState> changeTaskOrder(Vehicle v1, PlanState plan) {
        System.out.println("CentralizedTemplate.changeTaskOrder");
        List<PlanState> neighbours = new ArrayList<PlanState>();
    	int arraySize =  plan.getVTasks().get(v1.id()).size();
    	//sort the hash set to a list
    	ArrayList<Integer> tasks = new ArrayList<Integer>(new TreeSet<Integer>(plan.getVTasks().get(v1.id()))); 
    		
    	for(int i = 0; i < arraySize*2-1; i++) {
    		for(int j = i+1; j < arraySize*2; j++) {
    			PlanState neighbour = new PlanState(plan);

    			if(i < arraySize && j < arraySize) {
    				int t1 = neighbour.getTimeP()[tasks.get(i)];
    				neighbour.getTimeP()[tasks.get(i)] = neighbour.getTimeP()[tasks.get(j)];
    				neighbour.getTimeP()[tasks.get(j)] = t1;
    			}
    			else if(i < arraySize && j >= arraySize) {
    				int t1 = neighbour.getTimeP()[tasks.get(i)];
    				neighbour.getTimeP()[tasks.get(i)] = neighbour.getTimeD()[tasks.get(j-arraySize)];
    				neighbour.getTimeD()[tasks.get(j-arraySize)] = t1;
    			}
    			else {
    				int t1 = neighbour.getTimeD()[tasks.get(i-arraySize)];
    				neighbour.getTimeD()[tasks.get(i-arraySize)] = neighbour.getTimeD()[tasks.get(j-arraySize)];
    				neighbour.getTimeD()[tasks.get(j-arraySize)] = t1;
    			}
    			if(checkTimes(neighbour.getTimeP(), neighbour.getTimeD(), plan.getVTasks().get(v1.id())) 
    					&& updateLoad(neighbour, v1.id())){
        			neighbours.add(neighbour);
    			}
    		}
    	}
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
    private boolean updateLoad(PlanState plan, Integer vehicle) {
    	Integer next = plan.getNextPickup()[vehicle];
        Task t = getTask(plan.tasks, next);
        ArrayIterator itP = new ArrayIterator(plan.getTimeP(), plan.getVTasks().get(vehicle));
        ArrayIterator itD = new ArrayIterator(plan.getTimeD(), plan.getVTasks().get(vehicle));
        while(itP.hasNext() || itD.hasNext()) {
            int pickupTime = Integer.MAX_VALUE;
            int pickupIndex = -1;
            int deliverTime = Integer.MAX_VALUE;
            int deliverIndex = -1;
            if(itP.hasNext()) {
                pickupTime = itP.next();
                pickupIndex = findIndex(vehicle, plan, plan.getTimeP(),pickupTime);
            }
            if(itD.hasNext()) {
                deliverTime = itD.next();
                deliverIndex = findIndex(vehicle, plan, plan.getTimeD(),deliverTime);
            }
            if(pickupTime < deliverTime) {
            	System.out.println(pickupIndex);
                plan.getLoad()[vehicle][pickupTime] += getTask(plan.tasks, pickupIndex).weight;
                if(plan.getLoad()[vehicle][pickupTime] > plan.vehicles.get(vehicle).capacity()) return false;
      
            } else {
                plan.getLoad()[vehicle][deliverTime] -= getTask(plan.tasks, deliverIndex).weight;
            }
        }
        return true;
        
    }
    
    private Integer findIndex(Integer v, PlanState plan, Integer[] times, Integer t) {
    	for(int i = 0; i < times.length; i++) {
    		if(times[i] == t && plan.getVTasks().get(v).contains(i)) return i;
    	}
    	return null;
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
        System.out.println("CentralizedTemplate.changeVehicle");
        List<PlanState> neighbours = new ArrayList<PlanState>();
    	PlanState neighbour = new PlanState(plan);
    	int weight = getTask(plan.tasks, task).weight;
    	neighbour.removeVTasks(v1.id(), task);
    	neighbour.addVTasks(v2.id(), task);
    	
    	for(Integer t: neighbour.getVTasks().get(v1.id())) {
    		neighbour.getTimeP()[t] -= 1;
    		neighbour.getTimeD()[t] -= 1;
    		if(neighbour.getTimeD()[task] <= neighbour.getTimeP()[t]) {
    			neighbour.getTimeP()[t] -= 1;
    		}
    		if(neighbour.getTimeD()[task] <= neighbour.getTimeD()[t]) {
    			neighbour.getTimeD()[t] -= 1;
    		}
    	}
    	
    	for(int i = 0; i < plan.getTimeD()[task]; i++) {
    		neighbour.getLoad()[v1.id()][i] -= weight + neighbour.getLoad()[v1.id()][i+1];
    	}
    	
    	// increment v2 times because of pickup
    	for(Integer t: neighbour.getVTasks().get(v2.id())) {
    		neighbour.getTimeP()[t] += 1;
    		neighbour.getTimeD()[t] += 1;
    	}
    	
    	// update load of vehicle 2 (since we changed the times)
    	for(int i = 0; i < (plan.getVTasks().get(v2.id()).size()-1)*2; i++) {
    		neighbour.getLoad()[v2.id()][i+1] = neighbour.getLoad()[v2.id()][i];
    	}
    	
    	// try to add pickup until it's not possible anymore
		for(int deliver = 1; deliver < 2 * plan.getVTasks().get(v2.id()).size(); deliver++){
			PlanState newNeighbour = new PlanState(neighbour);
			boolean add = true;
			while(add) {
				if(neighbour.getLoad()[v2.id()][deliver] + weight > v2.capacity()){
					add = false;
				}
				// add at least one delivery
				for(Integer t: neighbour.getVTasks().get(v2.id())) {
		    		if(neighbour.getTimeD()[task] >= deliver) {
		    			newNeighbour.getTimeP()[t] += 1;
		    		}
		    		if(neighbour.getTimeD()[task] >= deliver) {
		    			newNeighbour.getTimeD()[t] += 1;
		    		}
		    	}
				newNeighbour.getTimeD()[task] = deliver;
				// update load
				for(int i = 0; i < deliver; i++) {
		    		newNeighbour.getLoad()[v2.id()][i] += weight;
		    	}
				neighbours.add(newNeighbour);
			}
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

        private Integer[] nextPickup; //
        private Integer[] timeP; // [p0, p1, ..., pn]
        private Integer[] timeD; // [d0, d1, ..., dn]
        private int[][] load;
        private final List<Vehicle> vehicles;
        private final TaskSet tasks;
        private Map<Integer, HashSet<Integer>> vTasks = new HashMap<Integer, HashSet<Integer>>(); // Map from vehicle_id to Set of tasks in vehicle's track

        /**
         * Constructor for empty PlanState
         * @param vehicles
         * @param tasks
         */
        public PlanState(List<Vehicle> vehicles, TaskSet tasks) {
            nextPickup = new Integer[vehicles.size()];
            timeP = new Integer[tasks.size()];
            timeD = new Integer[tasks.size()];
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
            nextPickup = p.getNextPickup().clone();
            timeP = p.getTimeP().clone();
            timeD = p.getTimeD().clone();
            load = new int[p.vehicles.size()][2 * p.tasks.size()];
            System.arraycopy(p.getLoad(), 0, this.load, 0, p.getLoad().length);
            this.tasks = p.tasks;
            this.vehicles = p.vehicles;
            //Copy of HashSet values in Map
            for(Vehicle v: vehicles) vTasks.put(v.id(), new HashSet<Integer>());
            for(Map.Entry<Integer, HashSet<Integer>> e: p.getVTasks().entrySet()) {
                for(Integer i: e.getValue()) {
                    vTasks.get(e.getKey()).add(i);
                }
            }
        }

        public Integer[] getNextPickup() {
            return nextPickup;
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

        public Map<Integer, HashSet<Integer>> getVTasks() {
            return vTasks;
        }
        
        public void addVTasks(Integer v, Integer t) {
        	vTasks.get(v).add(t);
        }
        
        public void removeVTasks(Integer v, Integer t) {
        	vTasks.get(v).remove(t);
        }

    }

    /**
     * Iterates over an int array from min to max
     */
    class ArrayIterator implements Iterator<Object> {

        private List<Integer> l = new ArrayList<Integer>();
        private HashSet<Integer> tasks = new HashSet<Integer>();

        ArrayIterator(Integer[] time, HashSet<Integer> t) {
            l = Arrays.asList(time);
            for (Integer i: t) {
                tasks.add(i);
            }
        }

        @Override
        public boolean hasNext() {
            for (Integer i: tasks) {
                if(l.get(i.intValue()) != null) return true;
            }
            return false;
        }

        @Override
        public Integer next() {
            Integer min;
            Integer index;
            do {
                min = Collections.min(l);
                index = l.indexOf(min);
            } while (!tasks.contains(index));
            l.set(index, null); //remove
            return min;
        }

        @Override
        public void remove() {

        }

        public int indexOf(Integer val) {
            return l.indexOf(val);
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


