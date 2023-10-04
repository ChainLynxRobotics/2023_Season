package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.Lib.MotionProfiles.DirectFeedProfile;
import frc.Lib.MotionProfiles.DirectFeedProfile.DirectConfig;
import frc.robot.Constants;
import frc.robot.Subsystems.ElevatorSubsystem;

//bind this command to a trigger in RobotContainer for testing
public class ElevatorDirectProfileCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private static double dt = Constants.GLOBAL_TIMESTEP;
    private double[] vel_samples;
    private DirectFeedProfile mProfile;

    //maxVel should be neo free speed and maxAccel is tunable
    public ElevatorDirectProfileCommand(ElevatorSubsystem elevator, double maxVel, double maxAccel, DirectConfig initState, DirectConfig finalState) {
        this.mProfile = new DirectFeedProfile(maxVel, maxAccel, initState, finalState, dt);
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        //initialize profile according to initial conditions and maximas
        vel_samples = mProfile.sampleAlongProfile();
        mProfile.setVelocityArray(vel_samples);
    }

    @Override
    public void execute() {
        double curVel = elevator.getEncoderVelocityRPM();
        mProfile.setCurVel(curVel);

        //apply this control effort to elevator motors
        elevator.setMotors(mProfile.calculate());
    }

    @Override
    public boolean isFinished() {
        //set to true once last setpoint is reached
        return mProfile.isProfileFinished();
    }
    
}
