/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Current Spike Counter that can help identify when current is spiking
 */
public class CurrentSpikeCounter {
    private double threshold, deadband, off_val;
    private boolean isSpiking = false, ramped = false, isRamping = false;

    /**
     * Create a spike counter
     * 
     * @param t - The threshold of current the counter should consider a success
     * @param d - The deadband of current we must drop below before another success
     *          is observed
     */
    public CurrentSpikeCounter(double t, double d) {
        threshold = t;
        deadband = d;
        off_val = threshold - deadband;
    }

    /**
     * Update the counter based on current
     * 
     * @param current  - The current
     * @param skipRamp - Whether or not the ramp should be considered a success
     * @return - Whether a spike has been observed
     */
    public boolean update(double current, boolean skipRamp) {
        if (!ramped && current > threshold) {
            isRamping = true;
            return !skipRamp;
        } else if (isRamping && current < off_val) {
            ramped = true;
            isRamping = false;
            return false;
        } else if (ramped && current > threshold && !isSpiking) {
            isSpiking = true;
            return true;
        } else if (ramped && current < off_val && isSpiking) {
            isSpiking = false;
            return false;
        } else {
            return false;
        }
    }

    /**
     * Has the current ramped
     * 
     * @return - Whether or not the current has ramped
     */
    public boolean hasRamped() {
        return ramped;
    }

    /**
     * Is the current currently ramping
     * 
     * @return - Whether or not the current is ramping
     */
    public boolean isRamping() {
        return isRamping;
    }

    /**
     * Is the current spiking (has already ramped)
     * 
     * @return - Whether or not the current is spiking (after ramping)
     */
    public boolean isSpiking() {
        return isSpiking;
    }
}