// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;

/** This is a class to help with different types of number logging */
public class LoggingUtils {
    public static enum LogType {
        AVERAGE, MAX, MIN, ROLLING_AVERAGE, DIRECT
    };

    private double value; // store output val
    private double[] values; // for rolling avg
    private ArrayList<Double> avg_values; // for overall avg
    private int roll_index; // for rolling avg index
    private int nums;
    private LogType state;

    /**
     * logging constructor, uses Max logging
     * @param init_val
     */
    LoggingUtils(double init_val) {
        this(init_val, LogType.MAX, 1);
    }

    /**
     * Logging constructor: specify init val and state type. 
     * If rolling avg, nums is how many values to store in 
     * the rolling average, otherwise it doesn't matter what 
     * the value is set to
     * @param init_val
     * @param state
     * @param nums
     */
    LoggingUtils(double init_val, LogType state, int nums) {
        switch (state) {
            case AVERAGE: 
                avg_values = new ArrayList<>();
                avg_values.add(init_val);
            case ROLLING_AVERAGE: 
                values = new double[nums];
                for (int i = 0; i < nums; i++)
                    values[i] = init_val;
                roll_index = 0;
                this.nums = nums;
            default: 
                value = init_val;
                this.state = state;
        }
    }

    /**
     * Update the number logging based on log type (Average of all, min, max, or a rolling average)
     * @param next
     * @return
     */
    public double update(double next) {
        switch (state) {
            case AVERAGE:
                avg_values.add(next);
                value = average(avg_values.iterator());
                break;
            case MAX:
                value = Math.max(value, next);
                break;
            case MIN:
                value = Math.min(value, next);
                break;
            case ROLLING_AVERAGE:
                roll(next);
                value = average(Arrays.stream(values).iterator());
                break;
            default: value = next;
        }
        return value;
    }

    /**
     * get average of an itarable 
     * @param i iterator
     * @return average
     */
    private double average(Iterator<Double> i) {
        double sum = 0;
        double count = 0;
        while (i.hasNext()) {
            sum += i.next();
            count += 1;
        }
        return sum / count;
    }

    /**
     * Updates rolling average. Uses rolling index to prevent rewriting or shifting of array
     * @param next
     */
    private void roll(double next) {
        values[roll_index]  = next;
        roll_index += 1;
        if (roll_index == nums)
            roll_index = 0;
    }

    /**
     * get the log value
     * @return value
     */
    private double getValue() {
        return value;
    }

    /**
     * get the log type
     * @return logtype 
     */
    private LogType getLogType() {
        return state;
    }
}
