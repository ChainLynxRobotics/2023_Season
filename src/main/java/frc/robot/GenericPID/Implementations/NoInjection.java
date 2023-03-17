package frc.robot.GenericPID.Implementations;

import frc.robot.GenericPID.Extensible.DescentInjectionClosure;

public class NoInjection implements DescentInjectionClosure {
    public void eval(double idc1, double idc2, double idc3) {
        return;
    }
}
