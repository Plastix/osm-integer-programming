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

    private static final String SOLVER_TYPE = "GLPK_MIXED_INTEGER_PROGRAMMING";
    // TODO (Aidan) Read these params from file
    private static final int MAX_COST = 40_000; // In meters
    private static final int MIN_COST = 20_000; // In meters
    // TODO (Aidan) Maybe convert this from lat/lon?
    private static final int START_NODE_ID = 0;
    private static final String RACE_BIKE_VEHICLE = "racingbike";
    private static final String ENABLED_VEHICLES = RACE_BIKE_VEHICLE;

    // Graph variables
    private static Graph graph;
    private static GraphUtils graphUtils;
    private static Weighting score;
    private static FlagEncoder flagEncoder;

    // Optimization variables
    private static MPSolver solver;
    private static MPVariable[] arcsFwd;
    private static MPVariable[] arcsBwd;
    private static int[] arcBaseIds;

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

        AllEdgesIterator edges = graph.getAllEdges();
        int numEdges = edges.getMaxId();
        int numNodes = graph.getNodes();

        // Make a decision variable "x_i" for every arc in our graph
        // x_i: =1 if arc is travelled, 0 otherwise
        // Since every edge can be 2 arcs (forward + backward), we keep two lists
        arcsFwd = solver.makeBoolVarArray(numEdges);
        arcsBwd = solver.makeBoolVarArray(numEdges);
        arcBaseIds = new int[numEdges];

        // Make a variable for every node in the graph
        // verts[i]: = the number of times vertex i is visited
        MPVariable[] verts = solver.makeIntVarArray(numNodes, 0, infinity);

        // (1a)
        // Objective maximizes total collected score of all roads
        MPObjective objective = solver.objective();
        objective.setMaximization();

        // (1b)
        // Limit length of path
        MPConstraint maxCost = solver.makeConstraint(-infinity, MAX_COST);

        while(edges.next()) {
            int edgeId = edges.getEdge();
            arcBaseIds[edgeId] = edges.getBaseNode(); // Record the original base ID to keep direction
            MPVariable fwd = arcsFwd[edgeId];
            MPVariable bwd = arcsBwd[edgeId];
            double edgeScore = score.calcWeight(edges, false, edgeId);
            double edgeDist = edges.getDistance();


            if(edges.isForward(flagEncoder)) {
                objective.setCoefficient(fwd, edgeScore);
                maxCost.setCoefficient(fwd, edgeDist);
            } else {
                fwd.setInteger(false);
            }

            if(edges.isBackward(flagEncoder)) {
                objective.setCoefficient(bwd, edgeScore);
                maxCost.setCoefficient(bwd, edgeDist);
            } else {
                bwd.setInteger(false);
            }

            // (1j)
            MPConstraint arcConstraint = solver.makeConstraint(-infinity, 1);
            arcConstraint.setCoefficient(fwd, 1);
            arcConstraint.setCoefficient(bwd, 1);
        }

        for(int i = 0; i < numNodes; i++) {
            // (1d)
            MPConstraint edgeCounts = solver.makeConstraint(0, 0);
            EdgeIterator incoming = graphUtils.incomingEdges(i);
            while(incoming.next()) {
                edgeCounts.setCoefficient(getArc(incoming), 1);
            }

            EdgeIterator outgoing = graphUtils.outgoingEdges(i);
            while(outgoing.next()) {
                MPVariable arc = getArc(outgoing);
                // Check if we already recorded it as an incoming edge
                if(edgeCounts.getCoefficient(arc) == 1) {
                    edgeCounts.setCoefficient(arc, 0);
                } else {
                    edgeCounts.setCoefficient(arc, -1);
                }
            }

            // (1e)
            MPConstraint vertexVisits = solver.makeConstraint(0, 0);
            outgoing = graphUtils.outgoingEdges(i);
            while(outgoing.next()) {
                vertexVisits.setCoefficient(getArc(outgoing), 1);
            }
            vertexVisits.setCoefficient(verts[i], -1);
        }

        // (1h)/(1i)
        // Start vertex can only be visited once
        MPVariable startNode = verts[START_NODE_ID];
        startNode.setBounds(1, 1);
    }

    private static MPVariable getArc(EdgeIterator edge) {
        int edgeId = edge.getEdge();
        int baseNode = edge.getBaseNode();
        if(baseNode == arcBaseIds[edgeId]) {
            return arcsFwd[edgeId];
        } else {
            return arcsBwd[edgeId];
        }
    }

    private static void runSolver() {
        System.out.println("Number of constraints: " + solver.numConstraints());
        System.out.println("Number of variables: " + solver.numVariables());

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

    private static void loadOSM() {
        System.out.println("---- Starting GraphHopper ----");
        GraphHopper hopper = new GraphHopperOSM();
        hopper.setDataReaderFile("src/main/resources/ny_capital_district.pbf");
        hopper.setGraphHopperLocation("src/main/resources/ny_capital_district-gh/");
        hopper.forDesktop();

        EncodingManager encodingManager = new EncodingManager(ENABLED_VEHICLES);
        hopper.setEncodingManager(encodingManager);
        hopper.setCHEnabled(false);
        hopper.importOrLoad();

        graph = hopper.getGraphHopperStorage().getBaseGraph();
        flagEncoder = encodingManager.getEncoder(RACE_BIKE_VEHICLE);
        score = new BikePriorityWeighting(flagEncoder);
        graphUtils = new GraphUtils(graph, flagEncoder);

        AllEdgesIterator edges = graph.getAllEdges();
        int nonTraversable = 0;
        int oneWay = 0;
        while(edges.next()) {
            if(!graphUtils.isTraversable(edges)) nonTraversable++;
            if(graphUtils.isOneWay(edges)) oneWay++;
        }

        System.out.println("\n---- OSM Graph Loaded ----");
        System.out.println(String.format("Edges: %d\nNodes: %d\nNon-traversable edges: %d\nOne-way edges: %d\n",
                graph.getAllEdges().getMaxId(), graph.getNodes(), nonTraversable, oneWay));
    }

    public static void main(String[] args) {
        // Parse & Load Open Street Map data
        loadOSM();

        // Solve integer programming problem
        System.out.println("---- Running integer programming optimizer ----");
        setupSolver();
        runSolver();
    }

}
