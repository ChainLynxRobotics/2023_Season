package frc.robot.GenericPID.Implementations;

import frc.robot.GenericPID.CatchUpFunction;

public class FullCatchUp implements CatchUpFunction {
    public double dt(double timeBehind, int timesBehind, double normalDt) {
        return timeBehind + normalDt;
    }
}