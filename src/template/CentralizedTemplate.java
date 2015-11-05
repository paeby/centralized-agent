package template;

//the list of imports
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

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
    private int[] nextPickup;
    private int[] nextDeliver;
    private int[] time; // [p1, p2, ..., pn, d1, d2, ..., dn]
    private int[][] load;

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
        nextPickup = new int[tasks.size() + vehicles.size()];
        nextDeliver = new int[tasks.size() + vehicles.size()];
        time = new int[tasks.size() * 2];
        load = new int[vehicles.size()][2 * tasks.size()];

        initSolution(vehicles, tasks);
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

    private void initSolution(List<Vehicle> vehicles, TaskSet tasks) {
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
        nextPickup[tasks.size() + index] = prev;
        time[prev] = t;
        load[index][t] = getTask(tasks, prev).weight;
        time[prev + tasks.size()] = ++t;
        load[index][t] = 0; // task has been delivered

        while (it.hasNext()) {
            int next = it.next().id;
            nextPickup[prev] = next;
            load[index][t] = getTask(tasks, prev).weight; // 0 or weight in load in initialised state.
            time[next] = ++t;
            time[next + tasks.size()] = ++t;
            load[index][t] = 0;
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

    class PlanState {

        private int[] nextPickup;
        private int[] timeP; // [p1, p2, ..., pn]
        private int[] timeD; // [d1, d2, ..., dn]
        private int[][] load;

        public PlanState(List<Vehicle> vehicles, TaskSet tasks) {
            nextPickup = new int[tasks.size() + vehicles.size()];
            timeP = new int[tasks.size()];
            timeD = new int[tasks.size()];
            load = new int[vehicles.size()][2 * tasks.size()];
        }

        public PlanState(List<Vehicle> vehicles, TaskSet tasks, PlanState p) {
            nextPickup = p.getNextPickup().clone();
            timeP = p.getTimeP().clone();
            timeD = p.getTimeD().clone();
            load = getLoad().clone();
        }

        public int[] getNextPickup() {
            return nextPickup;
        }

        public int[] getTimeP() {
            return timeP;
        }

        public int[] getTimeD() {
            return timeD;
        }

        public int[][] getLoad() {
            return load;
        }
    }
}
