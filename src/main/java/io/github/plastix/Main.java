package io.github.plastix;

import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;
import com.graphhopper.GraphHopper;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;

public class Main {

    private static final String SOLVER_TYPE = "CBC_MIXED_INTEGER_PROGRAMMING";
    // TODO (Aidan) Read these params from file
    private static final int MAX_COST = 40_000; // In meters
    private static final int MIN_COST = 20_000; // In meters
    // TODO (Aidan) Maybe convert this from lat/lon?
    private static final int START_NODE_ID = 834911;
    private static final String RACE_BIKE_VEHICLE = "racingbike";
    private static final String ENABLED_VEHICLES = RACE_BIKE_VEHICLE;

    private static Graph graph;
    private static final EncodingManager encodingManager = new EncodingManager(ENABLED_VEHICLES);
    private static final FlagEncoder flagEncoder = encodingManager.getEncoder(RACE_BIKE_VEHICLE);
    private static final Weighting score = new BikePriorityWeighting(flagEncoder);

    private static MPSolver solver;

    static {
        // Load Google Optimization tools native libs
        System.loadLibrary("jniortools");
    }

    private static MPSolver createSolver(String solverType) {
        try {
            return new MPSolver("osm-integer-programming",
                    MPSolver.OptimizationProblemType.valueOf(solverType));
        } catch(java.lang.IllegalArgumentException e) {
            return null;
        }
    }

    private static void setupSolver() {
        solver = createSolver(SOLVER_TYPE);
        if(solver == null) {
            System.out.println("Could not create solver " + SOLVER_TYPE);
            System.exit(1);
        }

        double infinity = MPSolver.infinity();

        AllEdgesIterator allEdges = graph.getAllEdges();
        int numEdges = allEdges.getMaxId();
        int numNodes = graph.getNodes();

        // Make a decision variable "x_a" for every edge in our graph
        // x_a: =1 if arc is travelled, 0 otherwise
        MPVariable[] x_a = solver.makeBoolVarArray(numEdges);

        // Make a variable for every node in the graph
        // z_v: = the number of times vertex v is visited
        MPVariable[] z_v = solver.makeIntVarArray(numNodes, 0, infinity);

        // (1a)
        MPObjective objective = solver.objective();
        objective.setMaximization(); // Maximize our objective (default is min)

        // (1b)
        MPConstraint maxCost = solver.makeConstraint(-infinity, MAX_COST);

        while(allEdges.next()) {
            int edgeId = allEdges.getEdge();

            // (1a)
            // Objective maximizes total collected score of all roads
            objective.setCoefficient(x_a[edgeId], score.calcWeight(allEdges, false, edgeId));

            // (1b)
            // Limit length of path
            maxCost.setCoefficient(x_a[edgeId], allEdges.getDistance());
        }

        for(int i = 0; i < numNodes; i++) {

            // (1d)
            MPConstraint edgeCounts = solver.makeConstraint(0, 0);
            EdgeIterator incoming = incomingEdges(i);
            // TODO (Aidan) This might cause problems since outgoing and incoming could be same
            while(incoming.next()){
                edgeCounts.setCoefficient(x_a[incoming.getEdge()], 1);
            }
            EdgeIterator outgoing = outgoingEdges(i);
            while(outgoing.next()){
                edgeCounts.setCoefficient(x_a[outgoing.getEdge()], -1);
            }

            // (1e)
            MPConstraint vertexVisits = solver.makeConstraint(0, 0);
            outgoing = outgoingEdges(i);
            while(outgoing.next()){
                vertexVisits.setCoefficient(x_a[outgoing.getEdge()], 1);
            }
            vertexVisits.setCoefficient(z_v[i], -1);
        }
    }

    private static EdgeIterator outgoingEdges(int node) {
        return graph.createEdgeExplorer(edgeState -> edgeState.isForward(flagEncoder)).setBaseNode(node);
    }

    private static EdgeIterator incomingEdges(int node) {
        return graph.createEdgeExplorer(edgeState -> edgeState.isBackward(flagEncoder)).setBaseNode(node);
    }


    private static void runSolver() {
        final MPSolver.ResultStatus resultStatus = solver.solve();

        // Check that the problem has an optimal solution.
        if(resultStatus != MPSolver.ResultStatus.OPTIMAL) {
            System.err.println("The problem does not have an optimal solution!");
            return;
        }

        // Verify that the solution satisfies all constraints (when using solvers
        // others than GLOP_LINEAR_PROGRAMMING, this is highly recommended!).
        if(!solver.verifySolution(/*tolerance=*/1e-7, /*logErrors=*/true)) {
            System.err.println("The solution returned by the solver violated the"
                    + " problem constraints by at least 1e-7");
            return;
        }

        System.out.println("Problem solved in " + solver.wallTime() + " milliseconds");

        // The objective value of the solution.
        System.out.println("Optimal objective value = " + solver.objective().value());
    }

    public static void main(String[] args) {
        // Parse & Load Open Street Map data
        GraphHopper hopper = new GraphHopperOSM();
        hopper.setDataReaderFile("src/main/resources/ny_capital_district.pbf");
        hopper.setGraphHopperLocation("src/main/resources/ny_capital_district-gh/");
        hopper.forDesktop();
        hopper.setEncodingManager(encodingManager);
        hopper.setCHEnabled(false);
        hopper.importOrLoad();

        graph = hopper.getGraphHopperStorage().getBaseGraph();
        System.out.println(String.format("Graph loaded! Edges: %d Nodes: %d",
                graph.getAllEdges().getMaxId(), graph.getNodes()));

        // Solve integer programming problem
        System.out.println("---- Running integer programming optimizer with CBC ----");
        setupSolver();
//        runSolver();
    }

}
