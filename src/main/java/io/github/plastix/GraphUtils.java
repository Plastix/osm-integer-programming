package io.github.plastix;

import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;

public class GraphUtils {

    private Graph graph;
    private FlagEncoder flagEncoder;

    GraphUtils(Graph graph, FlagEncoder flagEncoder) {
        this.graph = graph;
        this.flagEncoder = flagEncoder;
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

}
