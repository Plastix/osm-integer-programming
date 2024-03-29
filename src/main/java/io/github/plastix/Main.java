package io.github.plastix;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.GraphHopper;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;
import gurobi.*;

public class Main {

    private static Params params;
    private static int START_NODE_ID;

    // Graph variables
    private static GraphHopper hopper;
    private static Graph graph;
    private static GraphUtils graphUtils;

    // Solver variables
    private static GRBEnv env;
    private static GRBModel model;

    private static void setupSolver() throws GRBException {
        env = new GRBEnv("osm.log");
        model = new GRBModel(env);
        Vars vars = new Vars(graph, model, graphUtils);

        // (1a)
        // Objective maximizes total collected score of all roads
        GRBLinExpr objective = new GRBLinExpr();

        // (1b)
        // Limit length of path
        GRBLinExpr maxCost = new GRBLinExpr();

        AllEdgesIterator edges = graph.getAllEdges();
        while(edges.next()) {
            double edgeScore = graphUtils.getArcScore(edges);
            double edgeDist = edges.getDistance();

            GRBVar forward = vars.getArcVar(edges);
            GRBVar backward = vars.getComplementArcVar(edges);

            objective.addTerm(edgeScore, forward);
            objective.addTerm(edgeScore, backward);
            maxCost.addTerm(edgeDist, forward);
            maxCost.addTerm(edgeDist, backward);

            // (1j)
            GRBLinExpr arcConstraint = new GRBLinExpr();
            arcConstraint.addTerm(1, forward);
            arcConstraint.addTerm(1, backward);
            model.addConstr(arcConstraint, GRB.LESS_EQUAL, 1, "arc_constraint");
        }

        model.setObjective(objective, GRB.MAXIMIZE);
        model.addConstr(maxCost, GRB.LESS_EQUAL, params.getMaxCost(), "max_cost");

        int numNodes = graph.getNodes();
        for(int i = 0; i < numNodes; i++) {
            // (1d)
            GRBLinExpr edgeCounts = new GRBLinExpr();
            IntHashSet incomingIds = new IntHashSet();
            EdgeIterator incoming = graphUtils.incomingEdges(i);
            while(incoming.next()) {
                incomingIds.add(incoming.getEdge());
                edgeCounts.addTerm(1, vars.getArcVar(incoming));
            }

            EdgeIterator outgoing = graphUtils.outgoingEdges(i);
            while(outgoing.next()) {
                GRBVar arc = vars.getArcVar(outgoing);
                // Check if we already recorded it as an incoming edge
                if(incomingIds.contains(outgoing.getEdge())) {
                    edgeCounts.remove(arc);
                } else {
                    edgeCounts.addTerm(-1, arc);
                }
            }

            model.addConstr(edgeCounts, GRB.EQUAL, 0, "edge_counts");

            // (1e)
            GRBLinExpr vertexVisits = new GRBLinExpr();
            outgoing = graphUtils.outgoingEdges(i);
            while(outgoing.next()) {
                vertexVisits.addTerm(1, vars.getArcVar(outgoing));
            }
            vertexVisits.addTerm(-1, vars.getVertexVar(i));
            model.addConstr(vertexVisits, GRB.EQUAL, 0, "vertex_visits");
        }

        // (1h)/(1i)
        // Start vertex must be visited exactly once
        GRBVar startNode = vars.getVertexVar(START_NODE_ID);
        startNode.set(GRB.DoubleAttr.LB, 1);
        startNode.set(GRB.DoubleAttr.UB, 1);

        // Must set LazyConstraints parameter when using lazy constraints
        model.set(GRB.IntParam.LazyConstraints, 1);
        model.setCallback(new SubtourConstraint(vars, START_NODE_ID, graphUtils));
    }

    private static void runSolver() throws GRBException {
        System.out.println("Max cost: " + params.getMaxCost());
        System.out.println("Start position: " + params.getStartLat() + ", " + params.getStartLon() +
                " (Node " + START_NODE_ID + ")");

//        model.set(GRB.IntParam.LogToConsole, 0);
        model.optimize();

        env.dispose();
        model.dispose();
        hopper.close();
    }

    private static void loadOSM() {
        System.out.println("---- Starting GraphHopper ----");
        hopper = new GraphHopperOSM();
        hopper.setDataReaderFile(params.getGraphFile());
        hopper.setGraphHopperLocation(params.getGraphFolder());
        hopper.forDesktop();

        EncodingManager encodingManager = new EncodingManager(params.getVehicle());
        hopper.setEncodingManager(encodingManager);
        hopper.setCHEnabled(false);
        hopper.importOrLoad();

        graph = hopper.getGraphHopperStorage().getBaseGraph();
        graphUtils = new GraphUtils(graph, hopper.getLocationIndex(), encodingManager, params);
        START_NODE_ID = graphUtils.getStartNode();

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
        params = new Params();
        params.loadParams();

        // Parse & Load Open Street Map data
        loadOSM();

        // Solve integer programming problem
        System.out.println("---- Running integer programming optimizer ----");
        try {
            setupSolver();
            runSolver();
        } catch(GRBException e) {
            System.out.println("Error code: " + e.getErrorCode() + ". " +
                    e.getMessage());
        }
    }
}
