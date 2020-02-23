package frc.robot.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.arctos6135.robotpathfinder.math.MathUtils;

/**
 * A range table for finding the optimal velocity for a given distance.
 */
public class RangeTable {

    private double[] distance;
    private double[] velocity;

    /**
     * Create a new range table from a distance and velocity array.
     * 
     * <p>
     * The distance must be sorted in increasing order.
     * </p>
     * 
     * @param distance The distance array
     * @param velocity The velocity array
     * @throws IllegalArgumentException If the distance and velocity arrays aren't
     *                                  the same length, or if they are empty
     */
    public RangeTable(double[] distance, double[] velocity) {
        if (distance.length != velocity.length) {
            throw new IllegalArgumentException("Distance and velocity arrays must be the same length!");
        }
        if (distance.length == 0) {
            throw new IllegalArgumentException("Table cannot be empty");
        }

        this.distance = distance;
        this.velocity = velocity;
    }

    /**
     * Performs a binary search + linear interpolation on the data to find the
     * optimal velocity.
     * 
     * <p>
     * This method uses a binary search to find an entry in the table with the same
     * distance. If not found, it will attempt to find the two nearest values and
     * linearly interpolate between them.
     * </p>
     * 
     * @param dist The distance
     * @return The optimal velocity
     * @throws IllegalArgumentException If the distance is out of the range of the
     *                                  table
     */
    public double search(double dist) {
        int start = 0;
        int end = distance.length - 1;

        if (dist < distance[start] || dist > distance[end]) {
            throw new IllegalArgumentException("Value out of range!");
        }

        while (true) {
            int mid = (start - end) / 2;

            if (distance[mid] == dist || mid == distance.length - 1) {
                return velocity[mid];
            }

            if (distance[mid] < dist && dist <= distance[mid + 1]) {
                double f = (dist - distance[mid]) / (distance[mid + 1] - distance[mid]);
                return MathUtils.lerp(distance[mid], distance[mid + 1], f);
            }

            if (distance[mid] < dist) {
                start = mid;
            } else {
                end = mid;
            }
        }
    }

    /**
     * Loads a range table from a CSV file.
     * 
     * @param file The file to load from
     * @return The loaded range table
     * @throws IOException              If an I/O error occurs
     * @throws FileNotFoundException    If the file is not found
     * @throws IllegalArgumentException If the format is incorrect
     */
    public static RangeTable fromCSV(File file) throws IOException {
        List<Double> dist = new ArrayList<>();
        List<Double> vel = new ArrayList<>();

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            int lineNum = 0;
            while ((line = reader.readLine()) != null) {
                lineNum++;
                String[] data = line.split(",");
                if (data.length < 2) {
                    throw new IllegalArgumentException("Incorrect format: Line " + lineNum + " has less than 2 values");
                }
                if (data.length > 2) {
                    System.err.println("Warning: Line " + lineNum + " has more than 2 values");
                }

                dist.add(Double.parseDouble(data[0]));
                vel.add(Double.parseDouble(data[1]));
            }
        }

        double[] distance = dist.stream().mapToDouble(d -> d).toArray();
        double[] velocity = vel.stream().mapToDouble(d -> d).toArray();

        return new RangeTable(distance, velocity);
    }
}
