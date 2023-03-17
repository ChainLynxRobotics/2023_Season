package frc.robot.GenericPID.Implementations;

import frc.robot.GenericPID.Extensible.DoubleFunction;

public class Match implements DoubleFunction {
    public double eval(double x) {
        return x;
    }
}
