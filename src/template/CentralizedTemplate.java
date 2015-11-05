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

    private List<Plan> centralizedPlan(List<Vehicle> vehicles, TaskSet tasks, PlanState plan) {
        final TaskSet myTasks = tasks; //To allow access in inner class Helper
        initSolution(vehicles, tasks, plan);
        for (int i = 0; i < 10000; i++) {
            List<PlanState> neighbours = ChooseNeighbours(plan, );
            plan = LocalChoice(neighbours, vehicles);
        }

        List<Plan> vplans = new ArrayList<>();

        Helper helper = new Helper() {
            @Override
            public Plan buildPlan(PlanState state, Vehicle v) {
                Integer next = state.getNextPickup()[myTasks.size() + v.id()];
                boolean pOrD = true; //true for pickup, false for delivery
                Task t = getTask(myTasks, state.getNextPickup()[myTasks.size() + v.id()].intValue());
                Plan p = new Plan(v.homeCity());
                while(next != null) {
                    //TODO Add move to next pickup or delivery location, followed by pickup or delivery of specific task found in nextPickup
                }
                return null; //return Plan for vehicle v
            }
        }

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
        private Integer[] timeP; // [p1, p2, ..., pn]
        private Integer[] timeD; // [d1, d2, ..., dn]
        private int[][] load;

        public PlanState(List<Vehicle> vehicles, TaskSet tasks) {
            nextPickup = new Integer[tasks.size() + vehicles.size()];
            timeP = new Integer[tasks.size()];
            timeD = new Integer[tasks.size()];
            load = new int[vehicles.size()][2 * tasks.size()];
        }

        public PlanState(List<Vehicle> vehicles, TaskSet tasks, PlanState p) {
            nextPickup = p.getNextPickup().clone();
            timeP = p.getTimeP().clone();
            timeD = p.getTimeD().clone();
            load = getLoad().clone();
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
    }

    /**
     * Iterates over an int array from min to max
     */
    class ArrayIterator implements Iterator {

        private int[] time;
        private List<Integer> l;

        //TODO Need to know which tasks in time belong to which vehicle - Map object in state or List of Lists?
        ArrayIterator(Integer[] t) {
            l = Arrays.asList(t);
        }

        @Override
        public boolean hasNext() {
            return false;
        }

        @Override
        public Integer next() {

            return null;
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


