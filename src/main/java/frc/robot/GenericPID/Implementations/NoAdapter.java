package frc.robot.GenericPID.Implementations;

import frc.robot.GenericPID.MotorControlProfile.UnknownControlStrategyException;
import frc.robot.GenericPID.Extensible.ControlStrategy;
import frc.robot.GenericPID.MotorControlProfile.ControlLevel;

public class NoAdapter implements ControlStrategy {
    public double calculate(double effectv, ControlLevel given, ControlLevel needed, double currv) throws UnknownControlStrategyException {
        throw new UnknownControlStrategyException("No adapter defined");
    }
}
