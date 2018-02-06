package io.github.plastix;

import com.carrotsearch.hppc.IntArrayDeque;
import com.carrotsearch.hppc.IntHashSet;
import com.carrotsearch.hppc.cursors.IntCursor;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import gurobi.*;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

import static gurobi.GRB.Callback.RUNTIME;

public class SubtourConstraint extends GRBCallback {

    private final int START_NODE_ID;
    private GraphUtils graphUtils;
    private Vars vars;
    private double time = 0;

    SubtourConstraint(Vars vars, int startNodeId, GraphUtils graphUtils) {
        this.vars = vars;
        this.START_NODE_ID = startNodeId;
        this.graphUtils = graphUtils;
    }

    @Override
    protected void callback() {
        try {
            if(where == GRB.CB_MIPSOL) { // Found an integer feasible solution
                long start = System.nanoTime();
                IntHashSet solutionVertices = getSolutionVertices();
                IntHashSet visitedVertices = getReachableVertexSubset(START_NODE_ID);

//                System.out.println("--- Callback ---");
//                System.out.println("Verts in solution: " + numVerticesInSolution);
//                System.out.println(solutionVertices);
//                System.out.println("Reachable vertices: " + visitedVertices.size());
//                printSolution();

                // If the number of vertices we can reach from the start is not the number of vertices we
                // visit in the entire solution, we have a disconnected tour
                if(visitedVertices.size() < solutionVertices.size()) {
                    solutionVertices.removeAll(visitedVertices);

                    // Add sub-tour elimination constraint
                    GRBLinExpr subtourConstraint = new GRBLinExpr();
                    int sumVertexVisits = 0;
                    int totalOutgoingEdges = 0;

                    double lhs = 0;
                    for(IntCursor cursor : solutionVertices) {
                        int vertexId = cursor.value;
                        EdgeIterator outgoing = graphUtils.outgoingEdges(vertexId);

                        while(outgoing.next()) {
                            GRBVar var = vars.getArcVar(outgoing, false);
                            if(!solutionVertices.contains(outgoing.getAdjNode())) {
                                subtourConstraint.addTerm(1, var);
                                lhs += getSolution(var);
                            }
                            totalOutgoingEdges += 1;
                        }
                        sumVertexVisits += getSolution(vars.getVertexVar(vertexId));
                    }

                    double rhs = ((double) sumVertexVisits) / ((double) totalOutgoingEdges);
//                    System.out.println("adding lazy constraint! " + lhs + " >= " + rhs);
                    addLazy(subtourConstraint, GRB.GREATER_EQUAL, rhs);

                }

                long end = System.nanoTime();
                time += TimeUnit.NANOSECONDS.toSeconds(end - start);

                double solverTime = getDoubleInfo(RUNTIME);
                if(solverTime > 600) {
                    System.out.println(String.format("Lazy constraint time: %f s", time));
                    System.out.println(String.format("Gurobi wall time: %f s", solverTime));
                    System.exit(0);
                }
            }
        } catch(GRBException e) {
            System.out.println("Error code: " + e.getErrorCode() + ". " +
                    e.getMessage());
            e.printStackTrace();
        }
    }

    private IntHashSet getSolutionVertices() throws GRBException {
        IntHashSet result = new IntHashSet();
        GRBVar[] verts = vars.getVertexVars();
        double[] values = getSolution(verts);

        for(int i = 0; i < verts.length; i++) {
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
                    if(getSolution(vars.getArcVar(iter, false)) > 0) {
                        stack.addLast(connectedId);
                    }
                }
                explored.add(current);
            }
        }

        return explored;
    }

    private void printSolution() throws GRBException {
        GRBVar[] arcVars = vars.getArcVars();
        double[] values = getSolution(arcVars);

        StringBuilder arcString = new StringBuilder();

        for(int i = 0; i < arcVars.length - 1; i++) {
            arcString.append(values[i]);
            arcString.append(", ");
            arcString.append(arcVars[i].get(GRB.StringAttr.VarName));
            if(i < arcVars.length - 2) {
                arcString.append("\n");
            }
        }
        System.out.println("Arcs: " + arcString.toString());
        double[] verts = getSolution(vars.getVertexVars());
        System.out.println("Verts: " + Arrays.toString(verts));
    }
}
