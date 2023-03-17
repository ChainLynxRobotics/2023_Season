package frc.robot.GenericPID.Extensible;

import java.util.ArrayList;

import frc.robot.GenericPID.MotorControlProfile;
import frc.robot.GenericPID.MotorControlProfile.ControlLevel;
import frc.robot.GenericPID.MotorControlProfile.UnknownControlStrategyException;

/**
 * <p> Make your own method for how one control effect level is turned into another. <p>
 * Does not have to be in SI units or the same calculation (eg, FullForce returns a force but is at the level of Acceleration.)
 */
public interface ControlStrategy {
    //to do: need a way of getting information to this determine in some sender box because it doesn't seem like there's a good way to pass the arbitrary information without
    //"collecting it all"
    //i need to give up this is fine
    public double calculate(double effect, ControlLevel given, ControlLevel needed, double curreffect) throws UnknownControlStrategyException;
    //given the control effect and context info and a conversion requiremen, determine how to output to the motor.
}
