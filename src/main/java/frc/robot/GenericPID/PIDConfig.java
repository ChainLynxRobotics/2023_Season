package frc.robot.GenericPID;

public class PIDConfig {
    public double kP = 1;
    public double kI = 1;
    public double kD = 1; //don't use 1 lol

    public PIDConfig(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public PIDConfig() {

    }
}