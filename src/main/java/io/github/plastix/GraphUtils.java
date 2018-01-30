package io.github.plastix;

import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

public class GraphUtils {

    private Params params;
    private Graph graph;
    private LocationIndex locationIndex;
    private FlagEncoder flagEncoder;
    private BikePriorityWeighting weighting;

    GraphUtils(Graph graph, LocationIndex locationIndex, EncodingManager encodingManager, Params params) {
        this.graph = graph;
        this.locationIndex = locationIndex;
        this.flagEncoder = encodingManager.getEncoder(params.getVehicle());
        this.params = params;
        weighting = new BikePriorityWeighting(flagEncoder);
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

    public int getStartNode() {
        QueryResult result = locationIndex.findClosest(params.getStartLat(), params.getStartLon(),
                new DefaultEdgeFilter(flagEncoder));
        if(!result.isValid()) {
            throw new RuntimeException("Unable to find node at start lat/lon!");
        }
        return result.getClosestNode();

    }

    public boolean isForward(EdgeIteratorState edge) {
        return edge.isForward(flagEncoder);
    }

    public boolean isBackward(EdgeIteratorState edge) {
        return edge.isBackward(flagEncoder);
    }
}
