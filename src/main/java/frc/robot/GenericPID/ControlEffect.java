package frc.robot.GenericPID;

import java.util.ArrayList;

import frc.robot.GenericPID.MotorControlProfile.ControlLevel;
import frc.robot.GenericPID.MotorControlProfile.UnknownControlStrategyException;

public class ControlEffect {
    //a class that holds a control effect with the adapter

    private MotorControlProfile c;
    private double effect;

    public Double motorEffect(double currEffect) {
        try {
            return c.mEffect(effect, currEffect);
        }
        catch (UnknownControlStrategyException e) {
            return null;
        }
    }
}