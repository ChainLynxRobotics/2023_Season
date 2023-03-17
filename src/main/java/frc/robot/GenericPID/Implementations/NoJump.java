package frc.robot.GenericPID.Implementations;

import frc.robot.GenericPID.Extensible.BooleanFunction;

public class NoJump implements BooleanFunction {
    public boolean eval(double x) {
        return false;
    }
}
