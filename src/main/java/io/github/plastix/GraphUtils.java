package io.github.plastix;

import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

public class GraphUtils {

    private Graph graph;
    private FlagEncoder flagEncoder;
    private Weighting weighting;

    public GraphUtils(Graph graph, FlagEncoder flagEncoder, Weighting weighting) {
        this.graph = graph;
        this.flagEncoder = flagEncoder;
        this.weighting = weighting;
    }

    public EdgeIterator outgoingEdges(int node) {
        return graph.createEdgeExplorer(edgeState -> edgeState.isForward(flagEncoder)).setBaseNode(node);
    }

    public EdgeIterator incomingEdges(int node) {
        return graph.createEdgeExplorer(edgeState -> edgeState.isBackward(flagEncoder)).setBaseNode(node);
    }

    public boolean isTraversable(EdgeIterator edgeIterator) {
        return edgeIterator.isForward(flagEncoder) || edgeIterator.isBackward(flagEncoder);
    }

    public boolean isOneWay(EdgeIterator edgeIterator) {
        return edgeIterator.isForward(flagEncoder) != edgeIterator.isBackward(flagEncoder);
    }

    public EdgeExplorer getEdgeExplorer() {
        return graph.createEdgeExplorer(new DefaultEdgeFilter(flagEncoder));
    }

    public double getArcScore(EdgeIteratorState edge) {
        return weighting.calcWeight(edge, false, edge.getEdge());
    }

    public boolean isForward(EdgeIteratorState edge) {
        return edge.isForward(flagEncoder);
    }

    public boolean isBackward(EdgeIteratorState edge) {
        return edge.isBackward(flagEncoder);
    }
}
