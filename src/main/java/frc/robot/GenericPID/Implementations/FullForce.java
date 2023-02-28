package frc.robot.GenericPID.Implementations;

import java.util.ArrayList;

import frc.robot.GenericPID.ControlStrategy;
import frc.robot.GenericPID.MotorControlProfile.ControlLevel;
import frc.robot.GenericPID.MotorControlProfile.UnknownControlStrategyException;

public class FullForce implements ControlStrategy {
    private double F;
    public FullForce(double maxForce) {
        F = maxForce;
    }
    public double calculate(double effectv, ControlLevel given, ControlLevel needed, double currv) throws UnknownControlStrategyException{
        if (given.ordinal() + 1 != needed.ordinal()) {
            throw new UnknownControlStrategyException("Conversion not compatible with this control strategy");
        }
        if (effectv < currv) {
            return F;
        } else if (effectv > currv) {
            return -F;
        } else {
            return 0;
        }
    }
}
