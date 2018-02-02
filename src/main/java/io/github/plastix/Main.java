package io.github.plastix;

import com.graphhopper.GraphHopper;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBModel;

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
    private static Vars vars;
    private static Constraints constraints;

    private static void setupSolver() throws GRBException {
        env = new GRBEnv("osm.log");
        model = new GRBModel(env);
        vars = new Vars(graph, model, graphUtils);
        constraints = new Constraints(graph, model, vars, graphUtils);
        vars.addVarsToModel();
        constraints.setupConstraints(START_NODE_ID, params.getMaxCost());
    }

    private static void runSolver() throws GRBException {
        System.out.println("Max cost: " + params.getMaxCost());
        System.out.println("Start position: " + params.getStartLat() + ", " + params.getStartLon() +
                " (Node " + START_NODE_ID + ")");

//        model.set(GRB.IntParam.LogToConsole, 0);
        model.optimize();

        if(model.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL) {
            System.out.println("Route score: " + constraints.getObjective().getValue());
            System.out.println("Route distance: " + constraints.getMaxCostConstraint().getValue());
        }

        model.dispose();
        env.dispose();
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

        FlagEncoder flagEncoder = encodingManager.getEncoder(params.getVehicle());
        Weighting weighting = new BikePriorityWeighting(flagEncoder);
        graph = hopper.getGraphHopperStorage();
        graphUtils = new GraphUtils(graph, flagEncoder, weighting);
        START_NODE_ID = params.getStartNode(hopper.getLocationIndex(), flagEncoder);

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
