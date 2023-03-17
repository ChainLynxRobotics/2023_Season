package frc.robot.GenericPID.Implementations;

import frc.robot.GenericPID.Extensible.DoubleFunction;

public class Zero implements DoubleFunction {
    public double eval(double x) {
        return 0;
    }
}