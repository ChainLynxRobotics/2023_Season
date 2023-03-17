package frc.robot.GenericPID.Extensible;

import frc.robot.GenericPID.Pair;

public interface PairFunction {
    public Pair eval(double T);
}