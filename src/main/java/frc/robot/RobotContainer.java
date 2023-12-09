package frc.robot;  

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Bindings;
import frc.robot.Constants.NeoMotorConstants;
import frc.MotionProfiles.DirectFeedProfile;
import frc.robot.Commands.ElevatorDirectProfileCommand;
import frc.robot.Commands.ElevatorManualControlCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.ElevatorSubsystem;


public class RobotContainer {

  private final ElevatorSubsystem m_elevator;

  Joystick m_operatorController = new Joystick(
    OIConstants.kIOperatorControllerPort
  );

  public RobotContainer() {
    m_elevator = new ElevatorSubsystem();

    new Trigger(() -> m_operatorController.getRawButton(Bindings.manualElevatorControl))
      .whileTrue(new ElevatorManualControlCommand(m_operatorController, m_elevator));
}


  public Joystick getOperatorController() {
    return m_operatorController;
  }

  ElevatorSubsystem getElevator() {
    return m_elevator;
  }


  public Command getAutoCommand() {
    DirectFeedProfile.DirectConfig initialConfig = new DirectFeedProfile.DirectConfig(0.0, 0.0);
    DirectFeedProfile.DirectConfig finalConfig = new DirectFeedProfile.DirectConfig(0.5, 0.0);
    DirectFeedProfile m_profile = new DirectFeedProfile(NeoMotorConstants.kFreeSpeedRpm, NeoMotorConstants.kMaxAccel, initialConfig, finalConfig, Constants.GLOBAL_TIMESTEP);

    //return new ElevatorDirectProfileCommand(m_elevator, m_profile);
    return new RunCommand(() -> m_elevator.setMotors(0.4), m_elevator).withTimeout(5);
    }
  }
