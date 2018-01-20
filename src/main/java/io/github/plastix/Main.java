package io.github.plastix;

import com.graphhopper.GraphHopper;
import com.graphhopper.reader.osm.GraphHopperOSM;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.storage.Graph;

public class Main {

    public static void main(String[] args) {
        GraphHopper hopper = new GraphHopperOSM();
        hopper.setDataReaderFile("src/main/resources/ny_capital_district.pbf");
        hopper.setGraphHopperLocation("src/main/resources/ny_capital_district-gh/");
        hopper.setEncodingManager(new EncodingManager("racingbike"));
        System.out.println("Loading graph!");
        hopper.importOrLoad();

        Graph graph = hopper.getGraphHopperStorage().getBaseGraph();
        System.out.println("Graph loaded!");
    }

}
