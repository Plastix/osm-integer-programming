package io.github.plastix;

import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;
import gurobi.GRB;
import gurobi.GRBException;
import gurobi.GRBModel;
import gurobi.GRBVar;

import java.util.Arrays;

public class Vars {

    private Graph graph;
    private FlagEncoder flagEncoder;
    private GRBModel model;

    // arcs[arcId][0] = forwardArc
    // arcs[arcId][1] = backwardArc
    private GRBVar[][] arcs;
    private GRBVar[] verts;
    private int[] arcBaseIds;

    Vars(Graph graph, GRBModel model, FlagEncoder flagEncoder) throws GRBException {
        this.graph = graph;
        this.model = model;
        this.flagEncoder = flagEncoder;
        addVarsToModel();
    }

    private void addVarsToModel() throws GRBException {
        AllEdgesIterator edges = graph.getAllEdges();
        int numEdges = edges.getMaxId();
        int numNodes = graph.getNodes();
        arcBaseIds = new int[numEdges];

        // Make a decision variable for every arc in our graph
        // arcs[i] = 1 if arc is travelled, 0 otherwise
        GRBVar[] arcVars = model.addVars(2 * numEdges, GRB.BINARY);
        arcs = new GRBVar[numEdges][2];

        // Make a variable for every node in the graph
        // verts[i] = n the number of times vertex i is visited
        char[] types = new char[numNodes];
        Arrays.fill(types, GRB.INTEGER);
        verts = model.addVars(null, null, null, types,
                null, 0, numNodes);

        int i = 0;
        while(edges.next()) {
            int edgeId = edges.getEdge();
            int baseNode = edges.getBaseNode();
            arcBaseIds[edgeId] = baseNode;

            GRBVar forward = arcVars[i++];
            GRBVar backward = arcVars[i++];

            arcs[edgeId][0] = forward;
            arcs[edgeId][1] = backward;

            if(!edges.isForward(flagEncoder)) {
                forward.set(GRB.DoubleAttr.UB, 0);
            }

            if(!edges.isBackward(flagEncoder)) {
                backward.set(GRB.DoubleAttr.UB, 0);
            }
        }
    }

    private int getIndex(EdgeIterator edge) {
        return arcBaseIds[edge.getEdge()] == edge.getBaseNode() ? 0 : 1;
    }

    public GRBVar getArc(EdgeIterator edge) {
        return arcs[edge.getEdge()][getIndex(edge)];
    }

    public GRBVar getComplementArc(EdgeIterator edge) {
        return arcs[edge.getEdge()][getIndex(edge) ^ 1];
    }

    public GRBVar getVertex(int id) {
        return verts[id];
    }

    public GRBVar[] getVerts() {
        return verts;
    }
}
