package io.github.plastix;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.GraphHopper;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.EdgeIterator;
import gurobi.*;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;
import java.util.Properties;

public class Main {

    private static String GRAPH_FILE;
    private static String GRAPH_FOLDER;
    private static double MAX_COST;
    private static double START_LAT;
    private static double START_LON;
    private static int START_NODE_ID;
    private static String VEHICLE;
    private static String ENABLED_VEHICLES;

    // Graph variables
    private static Graph graph;
    private static GraphUtils graphUtils;
    private static Weighting score;
    private static FlagEncoder flagEncoder;

    private static GRBModel model;
    private static GRBVar[] arcsFwd;
    private static GRBVar[] arcsBwd;
    private static int[] arcBaseIds;

    private static void setupSolver() throws GRBException {
        GRBEnv env = new GRBEnv("osm.log");
        model = new GRBModel(env);

        AllEdgesIterator edges = graph.getAllEdges();
        int numEdges = edges.getMaxId();
        int numNodes = graph.getNodes();

        // Make a decision variable "x_i" for every arc in our graph
        // x_i: =1 if arc is travelled, 0 otherwise
        // Since every edge can be 2 arcs (forward + backward), we keep two lists
        arcsFwd = model.addVars(numEdges, GRB.BINARY);
        arcsBwd = model.addVars(numEdges, GRB.BINARY);
        arcBaseIds = new int[numEdges];

        // Make a variable for every node in the graph
        // verts[i]: = the number of times vertex i is visited
        char[] types = new char[numNodes];
        Arrays.fill(types, GRB.INTEGER);
        GRBVar[] verts = model.addVars(null, null, null, types,
                null, 0, numNodes);

        // (1a)
        // Objective maximizes total collected score of all roads
        GRBLinExpr objective = new GRBLinExpr();

        // (1b)
        // Limit length of path
        GRBLinExpr maxCost = new GRBLinExpr();

        while(edges.next()) {
            int edgeId = edges.getEdge();
            arcBaseIds[edgeId] = edges.getBaseNode(); // Record the original base ID to keep direction
            GRBVar fwd = arcsFwd[edgeId];
            GRBVar bwd = arcsBwd[edgeId];
            double edgeScore = score.calcWeight(edges, false, edgeId);
            double edgeDist = edges.getDistance();

            if(edges.isForward(flagEncoder)) {
                objective.addTerm(edgeScore, fwd);
                maxCost.addTerm(edgeDist, fwd);
            } else {
                fwd.set(GRB.DoubleAttr.UB, 0);
            }

            if(edges.isBackward(flagEncoder)) {
                objective.addTerm(edgeScore, bwd);
                maxCost.addTerm(edgeDist, bwd);
            } else {
                bwd.set(GRB.DoubleAttr.UB, 0);
            }

            // (1j)
            GRBLinExpr arcConstraint = new GRBLinExpr();
            arcConstraint.addTerm(1, fwd);
            arcConstraint.addTerm(1, bwd);
            model.addConstr(arcConstraint, GRB.LESS_EQUAL, 1, "arc_constraint");
        }

        model.setObjective(objective, GRB.MAXIMIZE);
        model.addConstr(maxCost, GRB.LESS_EQUAL, MAX_COST, "max_cost");

        for(int i = 0; i < numNodes; i++) {
            // (1d)
            GRBLinExpr edgeCounts = new GRBLinExpr();
            IntHashSet incomingIds = new IntHashSet();
            EdgeIterator incoming = graphUtils.incomingEdges(i);
            while(incoming.next()) {
                incomingIds.add(incoming.getEdge());
                edgeCounts.addTerm(1, getArc(incoming));
            }

            EdgeIterator outgoing = graphUtils.outgoingEdges(i);
            while(outgoing.next()) {
                GRBVar arc = getArc(outgoing);
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
                vertexVisits.addTerm(1, getArc(outgoing));
            }
            vertexVisits.addTerm(-1, verts[i]);
            model.addConstr(vertexVisits, GRB.EQUAL, 0, "vertex_visits");
        }

        // (1h)/(1i)
        // Start vertex can only be visited once
        GRBVar startNode = verts[START_NODE_ID];
        startNode.set(GRB.DoubleAttr.LB, 1);
        startNode.set(GRB.DoubleAttr.UB, 1);
    }

    private static GRBVar getArc(EdgeIterator edge) {
        int edgeId = edge.getEdge();
        int baseNode = edge.getBaseNode();
        if(baseNode == arcBaseIds[edgeId]) {
            return arcsFwd[edgeId];
        } else {
            return arcsBwd[edgeId];
        }
    }

    private static void runSolver() throws GRBException {
        System.out.println("Max cost: " + MAX_COST);
        System.out.println("Start position: " + START_LAT + ", " + START_LON + " (Node " + START_NODE_ID + ")");

        model.optimize();
    }

    private static void loadOSM() {
        System.out.println("---- Starting GraphHopper ----");
        GraphHopper hopper = new GraphHopperOSM();
        hopper.setDataReaderFile(GRAPH_FILE);
        hopper.setGraphHopperLocation(GRAPH_FOLDER);
        hopper.forDesktop();

        EncodingManager encodingManager = new EncodingManager(ENABLED_VEHICLES);
        hopper.setEncodingManager(encodingManager);
        hopper.setCHEnabled(false);
        hopper.importOrLoad();

        graph = hopper.getGraphHopperStorage().getBaseGraph();
        flagEncoder = encodingManager.getEncoder(VEHICLE);
        score = new BikePriorityWeighting(flagEncoder);
        graphUtils = new GraphUtils(graph, flagEncoder);

        QueryResult result = hopper.getLocationIndex().findClosest(START_LAT, START_LON,
                new DefaultEdgeFilter(flagEncoder));
        if(!result.isValid()) {
            System.out.println("Unable to find starting node near lat/lon points!");
            System.exit(1);

        }

        START_NODE_ID = result.getClosestNode();

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

    private static void loadParams() {
        Properties properties = new Properties();
        InputStream inputStream = null;
        try {
            inputStream = new FileInputStream("src/main/resources/params.properties");
            properties.load(inputStream);
        } catch(FileNotFoundException e) {
            System.out.println("No params file!");
            e.printStackTrace();
        } catch(IOException e) {
            System.out.println("Error reading params!");
            e.printStackTrace();
        }
        GRAPH_FILE = properties.getProperty("graphFile");
        GRAPH_FOLDER = properties.getProperty("graphFolder");
        START_LAT = Double.parseDouble(properties.getProperty("startLat"));
        START_LON = Double.parseDouble(properties.getProperty("startLon"));
        MAX_COST = Double.parseDouble(properties.getProperty("maxCost"));
        VEHICLE = properties.getProperty("vehicle");
        ENABLED_VEHICLES = VEHICLE;
    }


    public static void main(String[] args) {
        loadParams();

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
