package io.github.plastix;

import com.carrotsearch.hppc.IntIntHashMap;
import com.carrotsearch.hppc.IntIntMap;
import com.carrotsearch.hppc.IntObjectHashMap;
import com.carrotsearch.hppc.IntObjectMap;
import com.carrotsearch.hppc.cursors.IntObjectCursor;
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

    Vars(Graph graph, GRBModel model, GraphUtils graphUtils) {
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

            // Make a decision variable for every arc in our graph
            // arcs[i] = 1 if arc is travelled, 0 otherwise
            GRBVar forward = model.addVar(0, 1, 0, GRB.BINARY, "forward_" + edgeId);
            GRBVar backward = model.addVar(0, 1, 0, GRB.BINARY, "backward_" + edgeId);

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

    private IntObjectMap<GRBVar> getMap(EdgeIterator edge) {
        return arcBaseIds.get(edge.getEdge()) == edge.getBaseNode() ? forwardArcs : backwardArcs;
    }

    private IntObjectMap<GRBVar> getComplementMap(EdgeIterator edge) {
        return arcBaseIds.get(edge.getEdge()) != edge.getBaseNode() ? forwardArcs : backwardArcs;
    }

    public GRBVar getArcVar(EdgeIterator edge) {
        return getMap(edge).get(edge.getEdge());
    }

    public GRBVar getComplementArcVar(EdgeIterator edge) {
        return getComplementMap(edge).get(edge.getEdge());
    }

    public GRBVar getVertexVar(int id) {
        return verts[id];
    }

    public GRBVar[] getVertexVars() {
        return verts;
    }

    public GRBVar[] getArcVars() {
        GRBVar[] result = new GRBVar[forwardArcs.size() * 2];

        int i = 0;
        for(IntObjectCursor<GRBVar> forwardArc : forwardArcs) {
            result[i++] = forwardArc.value;
        }

        for(IntObjectCursor<GRBVar> backwardArc : backwardArcs) {
            result[i++] = backwardArc.value;
        }

        return result;
    }
}
