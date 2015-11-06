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

        //List<Plan> plans = centralizedPlan(vehicles, tasks, plan);

        Plan planVehicle1 = naivePlan(vehicles.get(0), tasks);

        List<Plan> plans = new ArrayList<Plan>();
        plans.add(planVehicle1);
        while (plans.size() < vehicles.size()) {
            plans.add(Plan.EMPTY);
        }

        long time_end = System.currentTimeMillis();
        long duration = time_end - time_start;
        System.out.println("The plan was generated in " + duration + " milliseconds.");

        return plans;
    }

    private List<Plan> centralizedPlan(List<Vehicle> vehicles, final TaskSet tasks, PlanState plan) {
        initSolution(vehicles, tasks, plan);
        for (int i = 0; i < 10000; i++) {
            List<PlanState> neighbours = ChooseNeighbours(plan, tasks, vehicles);
            plan = localChoice(neighbours, vehicles, tasks);
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
    
    private PlanState localChoice(List<PlanState> neighbours, List<Vehicle> vehicles, TaskSet tasks) {
    	PlanState bestPlan = null;
    	double min = Double.MAX_VALUE;
    	
    	for(PlanState neighbour: neighbours) {
    		double cost = 0;
    		for(Vehicle v: vehicles) {
    			 Integer next = neighbour.getNextPickup()[v.id()];
                 Task t = getTask(tasks, next);
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
                         cost += current.distanceTo(getTask(tasks, pickupIndex).pickupCity)*v.costPerKm();
                         current = getTask(tasks, pickupIndex).pickupCity;
               
                     } else {
                         cost += current.distanceTo(getTask(tasks, deliverIndex).deliveryCity)*v.costPerKm();
                         current = getTask(tasks, deliverIndex).deliveryCity;
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
    
    private List<PlanState> ChooseNeighbours(PlanState plan, TaskSet tasks, List<Vehicle> vehicles){
    	List<PlanState> neighbours = new ArrayList<PlanState>();
    	Vehicle v1;
    	
    	// Pick random vehicle
    	int vTasks = 0;
    	do {
    		v1 = vehicles.get(new Random().nextInt(vehicles.size() + 1));
    		vTasks = plan.getVTasks().get(v1.id()).size();
    	} while(vTasks < 1);
    	
    	// Change first task with all other vehicles
    	Integer task = plan.getNextPickup()[v1.id()];
    	for(Vehicle v2: vehicles) {
    		if(v1.id() != v2.id()) {
    			if(v2.capacity() <= getTask(tasks, task).weight) {
    				neighbours.addAll(changeVehicle(v1, v2, task, plan));
    			}
    		}
    	}
    	
    	// Changing task order 
    	neighbours.addAll(changeTaskOrder(v1, plan));
    	
    	return neighbours;
    }
    
    private List<PlanState> changeTaskOrder(Vehicle v1, PlanState plan) {
    	List<PlanState> neighbours = new ArrayList<PlanState>();
    	
    	// iterate through all swaps of two pickups if it's possible
    	// 		iterate through swaps of deliver
    	
    	//OR
    	
    	// iterate through swaps of pickup/deliver together?
    	
    	return neighbours;
    }
    
    private List<PlanState> changeVehicle(Vehicle v1, Vehicle v2, Integer task, PlanState plan) {
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
    	for(int i = 0; i < plan.getTimeD()[task]-1; i++) {
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
    
    private Task getTask(TaskSet tasks, int id) {
        for (Task t : tasks) {
            if (t.id == id) {
                return t;
            }
        }
        return null;
    }

    public class PlanState {

        private Integer[] nextPickup; //
        private Integer[] timeP; // [p0, p1, ..., pn]
        private Integer[] timeD; // [d0, d1, ..., dn]
        private Integer[][] load;
        private List<Vehicle> vehicles;
        private TaskSet tasks;
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
            load = new Integer[vehicles.size()][2 * tasks.size()];
            this.vehicles = vehicles;
            this.tasks = tasks;
        }

        /**
         * Constructor that copies a plan
         * @param vehicles
         * @param tasks
         * @param p plan to be copied to new plan
         */
        public PlanState(PlanState p) {
            nextPickup = p.getNextPickup().clone();
            timeP = p.getTimeP().clone();
            timeD = p.getTimeD().clone();
            load = getLoad().clone();
            this.tasks = p.tasks;
            this.vehicles = p.vehicles;
            //Copy of HashSet values in Map
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

        public Integer[][] getLoad() {
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

        private List<Integer> l;
        private HashSet<Integer> tasks;

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


