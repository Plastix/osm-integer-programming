package io.github.plastix;

import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;
import gurobi.*;

public class Constraints {

    private Graph graph;
    private GRBModel model;
    private Vars vars;
    private GraphUtils graphUtils;

    public Constraints(Graph graph, GRBModel model, Vars vars, GraphUtils graphUtils) {
        this.graph = graph;
        this.model = model;
        this.vars = vars;
        this.graphUtils = graphUtils;
    }

    public void setupConstraints(int startNodeId, double maxCostMeters) throws GRBException {

        // (1a)
        // Objective maximizes total collected score of all roads
        GRBLinExpr objective = new GRBLinExpr();

        // (1b)
        // Limit length of path
        GRBLinExpr maxCostConstraint = new GRBLinExpr();

        AllEdgesIterator edges = graph.getAllEdges();
        while(edges.next()) {
            double edgeScore = graphUtils.getArcScore(edges);
            double edgeDist = edges.getDistance();

            GRBVar forward = vars.getArcVar(edges, false);
            GRBVar backward = vars.getArcVar(edges, true);

            objective.addTerm(edgeScore, forward);
            objective.addTerm(edgeScore, backward);
            maxCostConstraint.addTerm(edgeDist, forward);
            maxCostConstraint.addTerm(edgeDist, backward);

            // (1j)
            GRBLinExpr arcConstraint = new GRBLinExpr();
            arcConstraint.addTerm(1, forward);
            arcConstraint.addTerm(1, backward);
            model.addConstr(arcConstraint, GRB.LESS_EQUAL, 1, "arc_constraint");
        }

        model.setObjective(objective, GRB.MAXIMIZE);
        model.addConstr(maxCostConstraint, GRB.LESS_EQUAL, maxCostMeters, "max_cost");

        int numNodes = graph.getNodes();
        for(int i = 0; i < numNodes; i++) {
            // (1d)
            GRBLinExpr edgeCounts = new GRBLinExpr();
            EdgeIterator incoming = graphUtils.incomingEdges(i);
            while(incoming.next()) {
                edgeCounts.addTerm(1, vars.getArcVar(incoming, true));
            }

            EdgeIterator outgoing = graphUtils.outgoingEdges(i);
            while(outgoing.next()) {
                GRBVar arc = vars.getArcVar(outgoing, false);
                edgeCounts.addTerm(-1, arc);
            }

            model.addConstr(edgeCounts, GRB.EQUAL, 0, "edge_counts");

            // (1e)
            GRBLinExpr vertexVisits = new GRBLinExpr();
            outgoing = graphUtils.outgoingEdges(i);
            while(outgoing.next()) {
                vertexVisits.addTerm(1, vars.getArcVar(outgoing, false));
            }
            vertexVisits.addTerm(-1, vars.getVertexVar(i));
            model.addConstr(vertexVisits, GRB.EQUAL, 0, "vertex_visits");
        }

        // (1h)/(1i)
        // Start vertex must be visited exactly once
        GRBVar startNodeVar = vars.getVertexVar(startNodeId);
        startNodeVar.set(GRB.DoubleAttr.LB, 1);
        startNodeVar.set(GRB.DoubleAttr.UB, 1);

        // Must set LazyConstraints parameter when using lazy constraints
        model.set(GRB.IntParam.LazyConstraints, 1);
        model.setCallback(new SubtourConstraint(vars, startNodeId, graphUtils));
    }
}
