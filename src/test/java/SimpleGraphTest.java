import com.carrotsearch.hppc.IntDoubleHashMap;
import com.carrotsearch.hppc.IntDoubleMap;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.util.HintsMap;
import com.graphhopper.routing.util.RacingBikeFlagEncoder;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.GraphBuilder;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.util.EdgeIteratorState;
import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBModel;
import io.github.plastix.Constraints;
import io.github.plastix.GraphUtils;
import org.junit.Before;
import org.junit.Test;

public class SimpleGraphTest {

    GraphHopperStorage graph;
    FlagEncoder flagEncoder;
    GRBModel model;
    IntDoubleMap weights;

    @Before
    public void setUp() throws Exception {
        flagEncoder = new RacingBikeFlagEncoder();
        EncodingManager encodingManager = new EncodingManager(flagEncoder);
        GraphBuilder graphBuilder = new GraphBuilder(encodingManager)
                .setStore(false);
        graph = graphBuilder.create();
        weights = new IntDoubleHashMap();

        createGraph();

        GRBEnv env = new GRBEnv("osm-test.log");
        model = new GRBModel(env);

    }

    private void createGraph() {
        addEdge(0, 1, true, 1, 1);
        addEdge(1, 2, true, 1, 1);
        addEdge(2, 0, true, 1, 1);

        addEdge(3, 4, true, 1, 1);
        addEdge(4, 5, true, 1, 1);
        addEdge(5, 3, true, 1, 1);

    }

    private void addEdge(int a, int b, boolean bidirectional, double cost, double score) {
        EdgeIteratorState edge = graph.edge(a, b, cost, bidirectional);
        weights.put(edge.getEdge(), score);
    }

    @Test
    public void optimizeRoute_disconnectedGraph() {
        try {
            GraphUtils graphUtils = new GraphUtils(graph, flagEncoder, new TestWeighting(weights));
            Constraints constraints = new Constraints(graph, model, graphUtils, 0, 4);
            constraints.setupConstraints();
            model.set(GRB.IntParam.LogToConsole, 0);
            model.optimize();
        } catch(GRBException e) {
            e.printStackTrace();
        }
    }

    private static class TestWeighting implements Weighting {

        private IntDoubleMap weights;

        TestWeighting(IntDoubleMap weights) {
            this.weights = weights;
        }

        @Override
        public double getMinWeight(double distance) {
            return 0;
        }

        @Override
        public double calcWeight(EdgeIteratorState edgeState, boolean reverse, int prevOrNextEdgeId) {
            return weights.get(edgeState.getEdge());
        }

        @Override
        public long calcMillis(EdgeIteratorState edgeState, boolean reverse, int prevOrNextEdgeId) {
            return 0;
        }

        @Override
        public FlagEncoder getFlagEncoder() {
            return null;
        }

        @Override
        public String getName() {
            return null;
        }

        @Override
        public boolean matches(HintsMap map) {
            return false;
        }
    }
}
