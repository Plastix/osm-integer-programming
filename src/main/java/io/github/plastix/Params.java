package io.github.plastix;

import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.QueryResult;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class Params {

    private String GRAPH_FILE;
    private String GRAPH_FOLDER;
    private double MAX_COST;
    private double START_LAT;
    private double START_LON;
    private String VEHICLE;
    private int NUM_THREADS;

    public void loadParams() {
        Properties properties = new Properties();
        InputStream inputStream;
        try {
            String paramPath = getClass().getResource("/params.properties").getPath();
            inputStream = new FileInputStream(paramPath);
            properties.load(inputStream);
            GRAPH_FILE = properties.getProperty("graphFile");
            GRAPH_FOLDER = properties.getProperty("graphFolder");
            START_LAT = Double.parseDouble(properties.getProperty("startLat"));
            START_LON = Double.parseDouble(properties.getProperty("startLon"));
            MAX_COST = Double.parseDouble(properties.getProperty("maxCost"));
            VEHICLE = properties.getProperty("vehicle");
            NUM_THREADS = Integer.parseInt(properties.getProperty("numThreads"));
        } catch(FileNotFoundException e) {
            System.out.println("No params file!");
            e.printStackTrace();
        } catch(IOException e) {
            System.out.println("Error reading params!");
            e.printStackTrace();
        } catch(NumberFormatException e) {
            System.out.println("Invalid parameter");
            e.printStackTrace();
        }
    }

    public String getGraphFile() {
        return GRAPH_FILE;
    }

    public String getGraphFolder() {
        return GRAPH_FOLDER;
    }

    public double getMaxCost() {
        return MAX_COST;
    }

    public double getStartLat() {
        return START_LAT;
    }

    public double getStartLon() {
        return START_LON;
    }

    public String getVehicle() {
        return VEHICLE;
    }

    public int getNumThreads() {
        return NUM_THREADS;
    }

    public int getStartNode(LocationIndex locationIndex, FlagEncoder flagEncoder) {
        QueryResult result = locationIndex.findClosest(START_LAT, START_LON,
                new DefaultEdgeFilter(flagEncoder));
        if(!result.isValid()) {
            throw new RuntimeException("Unable to find node at start lat/lon!");
        }
        return result.getClosestNode();
    }
}
