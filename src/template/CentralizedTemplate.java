package template;

//the list of imports
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
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

        List<Plan> plans = new ArrayList<>();
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
                Integer next = state.getNextPickup()[tasks.size() + v.id()];
                Task t = getTask(tasks, state.getNextPickup()[tasks.size() + v.id()]);
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
                        p.appendPickup(getTask(tasks, pickupIndex));
                    } else {
                        for (City c: current.pathTo(getTask(tasks, deliverIndex).deliveryCity)){
                            p.appendMove(c);
                        }
                        p.appendDelivery(getTask(tasks, deliverIndex));
                    }
                    //TODO nextPickup still necessary with hashSet of tasks for each vehicle?
                }
                
                
                
                //TODO!!!!!!!DIRTY IMPLEMENTATION!!!!!! 
                // IDEA: change the function local choice, would calculate the cost simply by 
                // taking the total distance of each plan calculated in this function!
                // actually not good idea to build a plan before the end... mhhhh..... problem.... brain fried...
                
                City newCity;
    			ArrayList<Integer> vTasks = new ArrayList<Integer>();
    			Set<Integer> delivered = new HashSet<Integer>();
    			if(state.getNextPickup()[v.id()] != null) {
    				
    				// moving to the first city
    				int firstPickup = state.getNextPickup()[tasks.size()+v.id()];
    				vTasks.add(firstPickup);
    				newCity = getTask(tasks, firstPickup).pickupCity;
    				p.appendMove(newCity); // moving to pickup first task
    				
    				// moving to the next pickup or delivery city
    				int nextPickup = firstPickup;
    				while(state.getNextPickup()[nextPickup] != null) {
    					nextPickup = state.getNextPickup()[nextPickup];
    					vTasks.add(nextPickup); // all the tasks of a vehicle
    					int timeP = state.getTimeP()[nextPickup];
    					
    					// Check if there are tasks to deliver before the task to pickup
    					int minTime = timeP;
    					Map<Integer, Integer> toDeliver = new HashMap<Integer, Integer>();
    					for(Integer vTask: vTasks) {
    						int timeD = state.getTimeD()[vTask];
    						if(timeD < minTime && (!delivered.contains(t))) {
    							toDeliver.put(timeD, vTask);
    						}
    					}
    					
    					while(toDeliver.size() != 0) {
    						int minD = Collections.min(toDeliver.keySet());
    						int deliver = toDeliver.get(minD);
    						toDeliver.remove(minD);
    						delivered.add(deliver);
    						City currentCity = newCity;
    						newCity = getTask(tasks, deliver).deliveryCity;
    						p.appendMove(newCity);
    					}
    					
    					//cost to the pickup city
    					City prevCity = newCity;
    					newCity = getTask(tasks, nextPickup).pickupCity;
    				}

    				// we still need to deliver all the remaining tasks
    				Map<Integer, Integer> toDeliver = new HashMap<Integer, Integer>();
    				for(Integer vTask: vTasks) {
    					if(!delivered.contains(t)) {
    						toDeliver.put(state.getTimeD()[vTask], vTask);
    					}
    				}
    				
    				while(toDeliver.size()!=0) {
    					int minD = Collections.min(toDeliver.keySet());
						int deliver = toDeliver.get(minD);
						toDeliver.remove(minD);
						delivered.add(deliver);
						City currentCity = newCity;
						newCity = getTask(tasks, deliver).deliveryCity;
						p.appendMove(newCity);
    				}
    			}

                return null; //return Plan for vehicle v
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
        plan.getNextPickup()[tasks.size() + index] = prev;
        plan.getTimeP()[prev] = t;
        plan.getLoad()[index][t] = getTask(tasks, prev).weight;
        plan.getTimeD()[prev] = ++t;
        plan.getLoad()[index][t] = 0; // task has been delivered

        while (it.hasNext()) {
            int next = it.next().id;
            plan.getNextPickup()[prev] = next;
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
    			City newCity;
    			ArrayList<Integer> vTasks = new ArrayList<Integer>();
    			Set<Integer> delivered = new HashSet<Integer>();
    			if(neighbour.getNextPickup()[v.id()] != null) {
    				
    				// moving to the first city
    				int firstPickup = neighbour.getNextPickup()[tasks.size()+v.id()];
    				vTasks.add(firstPickup);
    				newCity = getTask(tasks, firstPickup).pickupCity;
    				cost += v.homeCity().distanceTo(newCity)*v.costPerKm(); // moving to pickup first task
    				
    				// moving to the next pickup or delivery city
    				int nextPickup = firstPickup;
    				while(neighbour.getNextPickup()[nextPickup] != null) {
    					nextPickup = neighbour.getNextPickup()[nextPickup];
    					vTasks.add(nextPickup); // all the tasks of a vehicle
    					int timeP = neighbour.getTimeP()[nextPickup];
    					
    					// Check if there are tasks to deliver before the task to pickup
    					int minTime = timeP;
    					Map<Integer, Integer> toDeliver = new HashMap<Integer, Integer>();
    					for(Integer t: vTasks) {
    						int timeD = neighbour.getTimeD()[t];
    						if(timeD < minTime && (!delivered.contains(t))) {
    							toDeliver.put(timeD, t);
    						}
    					}
    					
    					while(toDeliver.size() != 0) {
    						int minD = Collections.min(toDeliver.keySet());
    						int deliver = toDeliver.get(minD);
    						toDeliver.remove(minD);
    						delivered.add(deliver);
    						City currentCity = newCity;
    						newCity = getTask(tasks, deliver).deliveryCity;
    						cost += currentCity.distanceTo(newCity)*v.costPerKm();
    					}
    					
    					//cost to the pickup city
    					City prevCity = newCity;
    					newCity = getTask(tasks, nextPickup).pickupCity;
    					cost += prevCity.distanceTo(newCity);
    				}
    				
    				//TODO CLEAN THIS DIRTY CODE
    				// we still need to deliver all the remaining tasks
    				Map<Integer, Integer> toDeliver = new HashMap<Integer, Integer>();
    				for(Integer t: vTasks) {
    					if(!delivered.contains(t)) {
    						toDeliver.put(neighbour.getTimeD()[t], t);
    					}
    				}
    				
    				while(toDeliver.size()!=0) {
    					int minD = Collections.min(toDeliver.keySet());
						int deliver = toDeliver.get(minD);
						toDeliver.remove(minD);
						delivered.add(deliver);
						City currentCity = newCity;
						newCity = getTask(tasks, deliver).deliveryCity;
						cost += currentCity.distanceTo(newCity)*v.costPerKm();
    				}
    			}
    		}
    		if(cost < min) {
        		bestPlan = neighbour;
        		min = cost;
        	}
    	}
    	return bestPlan;
    }
    
    private List<PlanState> ChooseNeighbours(PlanState plan, TaskSet tasks, List<Vehicle> vehicles){
    	List<PlanState> neighbours = new ArrayList<>();
    	
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

        private Integer[] nextPickup;
        private Integer[] timeP; // [p0, p1, ..., pn]
        private Integer[] timeD; // [d0, d1, ..., dn]
        private Integer[][] load;
        private Map<Integer, HashSet<Integer>> vTasks = new HashMap<>(); // Map from vehicle_id to Set of tasks in vehicle's track

        /**
         * Constructor for empty PlanState
         * @param vehicles
         * @param tasks
         */
        public PlanState(List<Vehicle> vehicles, TaskSet tasks) {
            nextPickup = new Integer[tasks.size() + vehicles.size()];
            timeP = new Integer[tasks.size()];
            timeD = new Integer[tasks.size()];
            load = new Integer[vehicles.size()][2 * tasks.size()];
        }

        /**
         * Constructor that copies a plan
         * @param vehicles
         * @param tasks
         * @param p plan to be copied to new plan
         */
        public PlanState(List<Vehicle> vehicles, TaskSet tasks, PlanState p) {
            nextPickup = p.getNextPickup().clone();
            timeP = p.getTimeP().clone();
            timeD = p.getTimeD().clone();
            load = getLoad().clone();
            //Copy of HashSet values in Map
            for(Map.Entry<Integer, HashSet<Integer>> e: p.getVTasks().entrySet()) {
                for(Integer i: e.getValue()) {
                    vTasks.get(e.getKey()).add(i);
                }
            }
        }

        /**
         *
         * @param v1 Vehicle that has task t in its set
         * @param v2 Vehicle that wants task t in its set
         * @param t Task to be moved from v1 to v2
         */
        public void moveTask(Integer v1, Integer v2, Integer t) {
            vTasks.get(v1).remove(t);
            vTasks.get(v2).add(t);
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

    }

    /**
     * Iterates over an int array from min to max
     */
    class ArrayIterator implements Iterator {

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


