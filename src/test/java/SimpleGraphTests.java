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
import io.github.plastix.Vars;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

@SuppressWarnings("WeakerAccess")
public class SimpleGraphTests {

    static final double FP_PRECISION = 0.01;

    GRBEnv env;
    GRBModel model;
    Vars vars;
    Constraints constraints;

    GraphHopperStorage graph;
    GraphUtils graphUtils;
    FlagEncoder flagEncoder;
    IntDoubleMap weights;

    @Before
    public void setUp() throws Exception {
        flagEncoder = new RacingBikeFlagEncoder();
        EncodingManager encodingManager = new EncodingManager(flagEncoder);
        GraphBuilder graphBuilder = new GraphBuilder(encodingManager)
                .setStore(false);
        graph = graphBuilder.create();
        weights = new IntDoubleHashMap();

        env = new GRBEnv();
        model = new GRBModel(env);
        graphUtils = new GraphUtils(graph, flagEncoder, new TestWeighting(weights));
        vars = new Vars(graph, model, graphUtils);
        constraints = new Constraints(graph, model, vars, graphUtils);

        model.set(GRB.IntParam.LogToConsole, 0);
    }

    @After
    public void tearDown() throws Exception {
        env.dispose();
        model.dispose();
        graph.close();
    }

    private void addEdge(int a, int b, boolean bidirectional, double cost, double score) {
        EdgeIteratorState edge = graph.edge(a, b, cost, bidirectional);
        weights.put(edge.getEdge(), score);
    }

    private void runSolver(int startNode, int maxCost) throws GRBException {
        vars.addVarsToModel();
        constraints.setupConstraints(startNode, maxCost);
        model.optimize();
    }

    @Test(expected = IllegalArgumentException.class)
    public void emptyGraph() throws GRBException {
        // Fails to run since we don't have a node ID (0)
        runSolver(0, 1);
    }

    @Test
    public void singleDirectedArcGraph() throws GRBException {
        addEdge(0, 1, false, 1, 1);

        runSolver(0, 2);
        assertNoSolution();
    }

    @Test
    public void singleUndirectedArcGraph() throws GRBException {
        addEdge(0, 1, true, 1, 1);

        runSolver(0, 2);
        assertNoSolution();
    }

    @Test
    public void disconnectedArcs() throws GRBException {
        addEdge(0, 1, false, 1, 1);
        addEdge(2, 3, false, 1, 1);

        runSolver(0, 2);
        assertNoSolution();
    }

    @Test
    public void singleDirectedThreeCycle() throws GRBException {
        addEdge(0, 1, false, 1, 1);
        addEdge(1, 2, false, 1, 1);
        addEdge(2, 0, false, 1, 1);

        runSolver(0, 3);
        assertHasSolution();
        assertSolution(3, 3);
    }

    @Test
    public void singleUndirectedThreeCycle() throws GRBException {
        addEdge(0, 1, true, 1, 1);
        addEdge(1, 2, true, 1, 1);
        addEdge(2, 0, true, 1, 1);

        runSolver(0, 3);
        assertHasSolution();
        assertSolution(3, 3);
    }

    @Test
    public void singleUndirectedThreeCycle_limitedBudget() throws GRBException {
        addEdge(0, 1, true, 1, 1);
        addEdge(1, 2, true, 1, 1);
        addEdge(2, 0, true, 1, 1);

        runSolver(0, 2);
        // We can't find a solution here since we aren't allowed to take a road backwards
        assertNoSolution();
    }


    @Test
    public void twoDisconnectedThreeCycles() throws GRBException {
        addEdge(0, 1, true, 1, 1);
        addEdge(1, 2, true, 1, 1);
        addEdge(2, 0, true, 1, 1);

        addEdge(3, 4, true, 1, 1);
        addEdge(4, 5, true, 1, 1);
        addEdge(5, 3, true, 1, 1);

        runSolver(0, 3);
        assertHasSolution();
        assertSolution(3, 3);
    }

    @Test
    public void multipleDisconnectedThreeCycles() throws GRBException {
        int numThreeCycles = 10;
        int nodeId = 0;

        for(int i = 0; i < numThreeCycles; i++) {
            addEdge(nodeId, nodeId + 1, false, 1, 1);
            addEdge(nodeId + 1, nodeId + 2, false, 1, 1);
            addEdge(nodeId + 2, nodeId, false, 1, 1);
            nodeId += 3;
        }

        runSolver(0, 3 * numThreeCycles);
        assertHasSolution();
        assertSolution(3, 3);
    }

    @Test
    public void twoConnectedThreeCycles() throws GRBException {
        addEdge(0, 1, false, 1, 1);
        addEdge(1, 2, false, 1, 1);
        addEdge(2, 0, false, 1, 1);

        addEdge(2, 3, true, 2, 2);

        addEdge(3, 4, false, 1, 1);
        addEdge(4, 5, false, 1, 1);
        addEdge(5, 3, false, 1, 1);

        runSolver(0, 6);
        assertHasSolution();
        assertSolution(3, 3);
    }

    @Test
    public void sixCycleWithTwoThreeCycles() throws GRBException {
        addEdge(0, 1, true, 1, 1);
        addEdge(1, 2, true, 1, 1);
        addEdge(2, 0, true, 1, 1);

        addEdge(2, 3, true, 1, 1);
        addEdge(1, 4, true, 1, 1);

        addEdge(3, 4, true, 1, 1);
        addEdge(4, 5, true, 1, 1);
        addEdge(5, 3, true, 1, 1);

        runSolver(0, 6);
        assertHasSolution();
        assertSolution(6, 6);
    }


    @Test
    public void directedKFour() throws GRBException {
        addEdge(0, 1, false, 1, 2);
        addEdge(1, 2, false, 1, 2);
        addEdge(2, 3, false, 1, 2);
        addEdge(3, 0, false, 1, 2);
        addEdge(0, 2, false, 1, 1);
        addEdge(1, 3, false, 1, 1);

        runSolver(0, 4);
        assertHasSolution();
        assertSolution(8, 4);
    }

    @Test
    public void simpleDirectedCactusGraph() throws GRBException {
        addEdge(0, 1, false, 1, 1);
        addEdge(1, 2, false, 1, 1);
        addEdge(2, 0, false, 1, 1);

        addEdge(1, 3, false, 1, 1);
        addEdge(3, 4, false, 1, 3);
        addEdge(4, 1, false, 1, 1);

        addEdge(2, 5, false, 1, 1);
        addEdge(5, 6, false, 1, 1);
        addEdge(6, 2, false, 1, 1);

        runSolver(0, 6);
        assertHasSolution();
        assertSolution(8, 6);
    }

    @Test
    public void threeNodeMultiEdgeGraph() throws GRBException {
        addEdge(0, 1, false, 1, 1);
        addEdge(2, 0, false, 1, 1);
        for(int i = 0; i < 20; i++) {
            addEdge(1, 2, true, 1, 1);
        }

        runSolver(0, 10);
        assertHasSolution();
        assertSolution(9, 9);
    }

    private void assertHasSolution() throws GRBException {
        if(model.get(GRB.IntAttr.Status) != GRB.Status.OPTIMAL) {
            fail("Gurobi could not find an optimal solution!");
        }
    }

    private void assertNoSolution() throws GRBException {
        if(model.get(GRB.IntAttr.Status) != GRB.Status.INFEASIBLE) {
            fail("Gurobi found an optimal solution!");
        }
    }

    private void assertSolution(double score, double cost) throws GRBException {
        assertEquals(score, constraints.getObjective().getValue(), FP_PRECISION);
        assertEquals(cost, constraints.getMaxCostConstraint().getValue(), FP_PRECISION);
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
