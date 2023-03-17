package frc.robot.GenericPID;

import frc.robot.GenericPID.Extensible.DoubleFunction;
import frc.robot.GenericPID.Extensible.EstimatedTimeArrival;
import frc.robot.GenericPID.Implementations.Instant;
import frc.robot.GenericPID.Implementations.Zero;

/**
 * A public configuration for a PID controller or the PID class.
 * @param kP Proportional Gain
 * @param kI Integral Gain
 * @param kD Derivative Gain
 * @param extraInfluence Other factors to be added, enclosed in a DoubleFunction.
 * @param estimatedTimeArrival An estimate for the amount of time the pid controller will need to get from point A to point B. 
 * Used for approximations if desired later. E.g. starting moving the motor earlier to get there on time.
 */
public class PIDConfig {
    public double kP = 1;
    public double kI = 1;
    public double kD = 1; //don't use 1 lol
    public DoubleFunction extraInfluence = new Zero();
    public EstimatedTimeArrival estimatedTimeArrival = new Instant();

    public PIDConfig(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public PIDConfig() {}
    public PIDConfig(PIDConfig other)  {
        this.kP = other.kP;
        this.kI = other.kI;
        this.kD = other.kD;
    }
}