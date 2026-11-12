package org.firstinspires.ftc.teamcode.utils;

import java.util.Arrays;

public class MovingAverage {
    private final double[] buffer;
    private double runningSum = 0;
    private int index = 0;
    private int count = 0;

    public MovingAverage(int windowSize) {
        if (windowSize <= 0) throw new IllegalArgumentException("windowSize must be positive");
        buffer = new double[windowSize];
    }

    public double get() {
        return count == 0 ? 0 : runningSum / count;
    }

    public void update(double value) {
        runningSum -= buffer[index];
        buffer[index] = value;
        runningSum += value;
        index = (index + 1) % buffer.length;
        if (count < buffer.length) {
            count++;
        }
    }

    public void reset() {
        Arrays.fill(buffer, 0);
        runningSum = 0;
        index = 0;
        count = 0;
    }
}
