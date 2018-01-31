package io.github.plastix;

import com.carrotsearch.hppc.IntArrayDeque;
import com.carrotsearch.hppc.IntHashSet;
import com.carrotsearch.hppc.cursors.IntCursor;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import gurobi.*;

public class SubtourConstraint extends GRBCallback {

    private final int START_NODE_ID;
    private GraphUtils graphUtils;
    private Vars vars;

    SubtourConstraint(Vars vars, int startNodeId, GraphUtils graphUtils) {
        this.vars = vars;
        this.START_NODE_ID = startNodeId;
        this.graphUtils = graphUtils;
    }

    @Override
    protected void callback() {
        try {
            if(where == GRB.CB_MIPSOL) { // Found an integer feasible solution

                IntHashSet visitedVertices = getReachableVertexSubset(START_NODE_ID);
                int numVerticesInSolution = numVerticesInSolution();

                System.out.println("-- Callback --");
                System.out.println("Solution vertices: " + numVerticesInSolution);
                System.out.println("Reachable vertices: " + visitedVertices.size());
                System.out.println("Solution arcs: " + numArcsInSolution());

                // If the number of vertices we can reach from the start is not the number of vertices we
                // visit in the entire solution, we have a disconnected tour
                if(visitedVertices.size() < numVerticesInSolution) {
                    visitedVertices.remove(START_NODE_ID);

                    // Add sub-tour elimination constraint
                    GRBLinExpr subtourConstraint = new GRBLinExpr();
                    int sumVertexVisits = 0;
                    int totalOutgoingEdges = 0;

                    double lhs = 0;
                    for(IntCursor cursor : visitedVertices) {
                        int vertexId = cursor.value;
                        EdgeIterator outgoing = graphUtils.outgoingEdges(vertexId);

                        while(outgoing.next()) {
                            GRBVar var = vars.getArcVar(outgoing, false);
                            subtourConstraint.addTerm(1, var);
                            totalOutgoingEdges += 1;

                            lhs += getSolution(var);
                        }


                        sumVertexVisits += getSolution(vars.getVertexVar(vertexId));
                    }

                    double rhs = ((double) sumVertexVisits) / ((double) totalOutgoingEdges);
                    System.out.println("adding lazy constraint! " + lhs + " <= " + rhs);
                    addLazy(subtourConstraint, GRB.LESS_EQUAL, rhs);

                }
            }
        } catch(GRBException e) {
            System.out.println("Error code: " + e.getErrorCode() + ". " +
                    e.getMessage());
            e.printStackTrace();
        }
    }

    private int numVerticesInSolution() throws GRBException {
        double[] values = getSolution(vars.getVertexVars());

        int visited = 0;
        for(double value : values) {
            if(value > 0) {
                visited++;
            }
        }
        return visited;
    }

    private IntHashSet verticesInSolution() throws GRBException {
        double[] values = getSolution(vars.getVertexVars());

        IntHashSet result = new IntHashSet();

        for(int i = 0; i < values.length; i++) {
            if(values[i] > 0) {
                result.add(i);
            }
        }

        return result;
    }

    private IntHashSet getReachableVertexSubset(int startNode) throws GRBException {
        EdgeExplorer explorer = graphUtils.getEdgeExplorer();
        IntArrayDeque stack = new IntArrayDeque();
        stack.addLast(startNode);

        IntHashSet explored = new IntHashSet();
        int current;
        while(stack.size() > 0) {
            current = stack.removeLast();
            if(!explored.contains(current)) {
                EdgeIterator iter = explorer.setBaseNode(current);
                while(iter.next()) {
                    int connectedId = iter.getAdjNode();
                    if(getSolution(vars.getArcVar(iter, false)) > 0.5) {
                        stack.addLast(connectedId);
                    }
                }
                explored.add(current);
            }
        }

        return explored;
    }

    private int numArcsInSolution() throws GRBException {
        double[] values = getSolution(vars.getArcVars());

        int visited = 0;
        for(double value : values) {
            if(value > 0) {
                visited++;
            }
        }
        return visited;
    }
}
