package frc.robot.GenericPID.Approximations.Implementations;

import frc.robot.GenericPID.Approximations.DoubleFunction;

public class Zero implements DoubleFunction {
    public double eval(double x) {
        return 0;
    }
}