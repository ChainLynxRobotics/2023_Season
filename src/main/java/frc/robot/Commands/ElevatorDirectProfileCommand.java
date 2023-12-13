package frc.robot.Commands;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.MotionProfiles.DirectFeedProfile;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;

//bind this command to a trigger in RobotContainer for testing
public class ElevatorDirectProfileCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private double[] vel_samples;
    private DirectFeedProfile mProfile;
    private double timeCounter;
    private SparkMaxPIDController controller;
    

    //maxVel should be neo free speed (rpm) and maxAccel is tunable
    public ElevatorDirectProfileCommand(ElevatorSubsystem elevator, DirectFeedProfile mProfile) {
        this.mProfile = mProfile;
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        //initialize profile according to initial conditions and maximas
        vel_samples = mProfile.sampleAlongProfile();
        System.out.printf("velocities: %f", vel_samples);
        mProfile.setVelocityArray(vel_samples);

        timeCounter = 0;
        controller = elevator.getController();
    }

    @Override
    public void execute() {
        timeCounter++;

        //apply this control effort to elevator motors
        double velSetpoint = mProfile.calculate(timeCounter*Constants.GLOBAL_TIMESTEP);
        System.out.printf("current velocity setpoint: %f%n", velSetpoint);
        controller.setReference(velSetpoint, ControlType.kVelocity);
    }

    @Override
    public boolean isFinished() {
        //set to true once last setpoint is reached
        return mProfile.isProfileFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("finished command!");
    }
    
}