package frc.robot;

import java.util.Arrays;

public class SlidingWindow {
    private final double[] data;

    public SlidingWindow(int length) {
        data = new double[length];
    }

    public void fill(double value) {
        Arrays.fill(data, value);
    }

    /**
     * @implNote Could use some kind of insertion sort for efficiency
     * @return The median of the dataset
     */
    public double median() {
        double[] copy = Arrays.copyOf(data, data.length);
        Arrays.sort(copy);

        int middleIdx = copy.length / 2; // int division floors
        if (copy.length % 2 == 0) {
            return (copy[middleIdx - 1] + copy[middleIdx]) / 2.0;
        } else {
            return copy[middleIdx];
        }
    }

    /**
     * @implNote Could get fancy in the `update` function for efficiency,
     * instead of re-computing here every time
     * @return The mean of the dataset
     */
    public double mean() {
        double sum = 0;
        for (int i = 0; i < data.length; i++) {
            sum += data[i];
        }
        return sum / (double)(data.length);
    }

    /**
     * Shifts all values backwards in the data array. Should be called
     * when we receive a new observation.
     * @param value The new observation
     */
    public void update(double value) {
        for (int i = 0; i < data.length - 1; i++) {
            data[i] = data[i + 1];
        }
        data[data.length - 1] = value;
    }
}
