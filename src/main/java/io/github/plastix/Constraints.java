package io.github.plastix;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;
import gurobi.*;

public class Constraints {

    private final double MAX_COST;
    private Graph graph;
    private GRBModel model;
    private GraphUtils graphUtils;
    private int START_NODE_ID;

    public Constraints(Graph graph, GRBModel model, GraphUtils graphUtils, int START_NODE_ID, double maxCost) {
        this.graph = graph;
        this.model = model;
        this.graphUtils = graphUtils;
        this.START_NODE_ID = START_NODE_ID;
        this.MAX_COST = maxCost;
    }

    public void setupConstraints() throws GRBException {
        Vars vars = new Vars(graph, model, graphUtils);
        vars.addVarsToModel();

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
        model.addConstr(maxCost, GRB.LESS_EQUAL, MAX_COST, "max_cost");

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
}
