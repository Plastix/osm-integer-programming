package io.github.plastix;

import com.carrotsearch.hppc.IntIntHashMap;
import com.carrotsearch.hppc.IntIntMap;
import com.carrotsearch.hppc.IntObjectHashMap;
import com.carrotsearch.hppc.IntObjectMap;
import com.carrotsearch.hppc.cursors.IntCursor;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;
import gurobi.GRB;
import gurobi.GRBException;
import gurobi.GRBModel;
import gurobi.GRBVar;

import java.util.Arrays;

public class Vars {

    private Graph graph;
    private GraphUtils graphUtils;
    private GRBModel model;

    private GRBVar[] verts;
    private IntObjectMap<GRBVar> forwardArcs;
    private IntObjectMap<GRBVar> backwardArcs;
    private IntIntMap arcBaseIds; // Records original "direction" of arc when processed.

    public Vars(Graph graph, GRBModel model, GraphUtils graphUtils) {
        this.graph = graph;
        this.model = model;
        this.graphUtils = graphUtils;

        backwardArcs = new IntObjectHashMap<>();
        forwardArcs = new IntObjectHashMap<>();
        arcBaseIds = new IntIntHashMap();
    }

    public void addVarsToModel() throws GRBException {
        AllEdgesIterator edges = graph.getAllEdges();
        int numNodes = graph.getNodes();

        // Make a variable for every node in the graph
        // verts[i] = n the number of times vertex i is visited
        char[] types = new char[numNodes];
        Arrays.fill(types, GRB.INTEGER);
        verts = model.addVars(null, null, null, types,
                null, 0, numNodes);

        while(edges.next()) {
            int edgeId = edges.getEdge();
            int baseNode = edges.getBaseNode();
            arcBaseIds.put(edgeId, baseNode);
            int adjNode = edges.getAdjNode();

            // Make a decision variable for every arc in our graph
            // arcs[i] = 1 if arc is travelled, 0 otherwise
            GRBVar forward = model.addVar(0, 1, 0, GRB.BINARY, "forward_" + edgeId + "|" + baseNode + "->" + edges.getAdjNode());
            GRBVar backward = model.addVar(0, 1, 0, GRB.BINARY, "backward_" + edgeId + "|" + baseNode + "->" + edges.getAdjNode());

            forwardArcs.put(edgeId, forward);
            backwardArcs.put(edgeId, backward);

            if(!graphUtils.isForward(edges)) {
                forward.set(GRB.DoubleAttr.UB, 0);
            }

            if(!graphUtils.isBackward(edges)) {
                backward.set(GRB.DoubleAttr.UB, 0);
            }
        }
    }

    private IntObjectMap<GRBVar> getMap(EdgeIterator edge, boolean reverse) {
        int baseNode = edge.getBaseNode();
        if(reverse) {
            baseNode = edge.getAdjNode();
        }
        return arcBaseIds.get(edge.getEdge()) == baseNode ? forwardArcs : backwardArcs;
    }


    public GRBVar getArcVar(EdgeIterator edge, boolean reverse) {
        return getMap(edge, reverse).get(edge.getEdge());
    }

    public GRBVar getVertexVar(int id) {
        if(id < 0 || id >= graph.getNodes()) {
            throw new IllegalArgumentException(String.format("Invalid node id %d", id));
        }
        return verts[id];
    }

    public GRBVar[] getVertexVars() {
        return verts;
    }

    public GRBVar[] getArcVars() {
        GRBVar[] result = new GRBVar[forwardArcs.values().size() * 2];

        int j = 0;
        for(IntCursor intCursor : forwardArcs.keys()) {
            result[j++] = forwardArcs.get(intCursor.value);
            result[j++] = backwardArcs.get(intCursor.value);
        }

        return result;
    }
}
