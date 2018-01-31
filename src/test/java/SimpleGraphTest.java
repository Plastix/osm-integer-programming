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
import org.junit.Ignore;
import org.junit.Test;

import java.util.Arrays;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

@SuppressWarnings("WeakerAccess")
public class SimpleGraphTest {

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

    @Test
    public void singleThreeCycle() throws GRBException {
        addEdge(0, 1, false, 1, 1);
        addEdge(1, 2, false, 1, 1);
        addEdge(2, 0, false, 1, 1);

        vars.addVarsToModel();
        constraints.setupConstraints(0, 3);
        model.optimize();

        assertHasSolution();
        printSolution();

        double score = model.get(GRB.DoubleAttr.ObjVal);
        assertEquals(3, score, FP_PRECISION);
    }

    @Ignore
    @Test
    public void singleThreeCycle_limitedBudget() throws GRBException {
        addEdge(0, 1, true, 1, 1);
        addEdge(1, 2, true, 1, 1);
        addEdge(2, 0, true, 1, 1);

        vars.addVarsToModel();
        constraints.setupConstraints(0, 2);
        model.optimize();

        double score = model.get(GRB.DoubleAttr.ObjVal);
        assertEquals(2, score, FP_PRECISION);
    }


    @Ignore
    @Test
    public void twoDisconnectedThreeCycles() throws GRBException {
        addEdge(0, 1, true, 1, 1);
        addEdge(1, 2, true, 1, 1);
        addEdge(2, 0, true, 1, 1);

        addEdge(3, 4, true, 1, 1);
        addEdge(4, 5, true, 1, 1);
        addEdge(5, 3, true, 1, 1);

        vars.addVarsToModel();
        constraints.setupConstraints(0, 4);
        model.optimize();

        double score = model.get(GRB.DoubleAttr.ObjVal);

        assertEquals(3, score, FP_PRECISION);
    }

    private void printSolution() throws GRBException {
        System.out.println("---- Final Solution ----");
        double[] arcs = model.get(GRB.DoubleAttr.X, vars.getArcVars());

        StringBuilder arcString = new StringBuilder();

        for(int i = 0; i < arcs.length - 1; i += 2) {
            arcString.append("(");
            arcString.append(arcs[i]);
            arcString.append(", ");
            arcString.append(arcs[i + 1]);
            arcString.append(") ");
        }
        System.out.println("Arcs: " + arcString.toString());

        double[] verts = model.get(GRB.DoubleAttr.X, vars.getVertexVars());
        System.out.println("Verts: " + Arrays.toString(verts));

    }

    private void assertHasSolution() throws GRBException {
        if(model.get(GRB.IntAttr.Status) != GRB.Status.OPTIMAL) {
            fail("Gurobi could not find an optimal solution!");
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
