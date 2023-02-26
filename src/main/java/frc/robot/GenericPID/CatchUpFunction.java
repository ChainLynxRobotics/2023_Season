package frc.robot.GenericPID;

public interface CatchUpFunction {
    public double dt(double timeBehind, int timesBehind, double normalDt); //timebehind is not behind target, but behind "on track" which includes dt
}