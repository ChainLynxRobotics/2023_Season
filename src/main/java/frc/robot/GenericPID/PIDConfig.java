package frc.robot.GenericPID;

import frc.robot.GenericPID.Approximations.DoubleFunction;
import frc.robot.GenericPID.Approximations.Implementations.Zero;

public class PIDConfig {
    public double kP = 1;
    public double kI = 1;
    public double kD = 1; //don't use 1 lol
    public DoubleFunction extraInfluence = new Zero();

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