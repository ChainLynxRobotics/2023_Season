package frc.robot.GenericPID;

import java.util.ArrayList;

import frc.robot.GenericPID.ControlEffectProfile.ControlLevel;
import frc.robot.GenericPID.ControlEffectProfile.UnknownControlStrategyException;

public interface ControlStrategy {
    public double determine(double effect, ControlLevel given, ControlLevel needed, ArrayList<Double> contextInfo) throws UnknownControlStrategyException;
    //given the control effect and context info and a conversion requiremen, determine how to output to the motor.
    //Context info is formatted as the control strategy wants it to be.
}
