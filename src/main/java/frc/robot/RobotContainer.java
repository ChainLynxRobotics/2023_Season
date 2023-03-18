
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.VisionTurnCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeGamePiece;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ReleaseCommand;
import frc.robot.Commands.SimpleDriveCommand;
import frc.robot.Commands.VisionTranslateCommand;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem m_robotDrive;
  private final VisionSubsystem m_vision;
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  private final IntakeSubsystem m_intake;

  private final AutoUtils autoUtils = new AutoUtils();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_operatorController = new Joystick(OIConstants.kIOperatorControllerPort);


  public RobotContainer() {
    SmartDashboard.putBoolean("GamePiece/CubeMode", false);
    SmartDashboard.putBoolean("GamePiece/ConeMode", false);
   
    m_robotDrive = new DriveSubsystem();
    m_vision = new VisionSubsystem();
    m_elevator = new ElevatorSubsystem();
    m_arm = new ArmSubsystem();
    m_intake = new IntakeSubsystem();

    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_driverController.getRightY(), OIConstants.kDriveDeadband),
                m_driverController.getRightBumper(), m_driverController.getAButton()),
            m_robotDrive));
    
    m_elevator.setDefaultCommand(
        new RunCommand(
          () -> m_elevator.simpleMovement(
            MathUtil.applyDeadband(m_operatorController.getRawAxis(1), ElevatorConstants.elevatorDeadband)), m_elevator));
  }


  private void configureButtonBindings() {

    new Trigger(() -> m_driverController.getBButton())
      .whileTrue(new VisionTurnCommand(m_vision, m_robotDrive, m_driverController));
   
    new Trigger(() -> m_driverController.getYButton())
      .onTrue(new VisionTranslateCommand(m_vision, m_robotDrive, m_driverController));

    new Trigger(() -> triggerPressed())
      .whileTrue(new SimpleDriveCommand(m_robotDrive, m_driverController));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.releaseGamePiece))
      .whileTrue(new ReleaseCommand(m_intake, m_intake.getWAxisSpeedMultiplier(OIConstants.INTAKE_SPEED_AXIS)));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.intakeGamePiece))
      .whileTrue(new IntakeCommand(m_intake, m_intake.getWAxisSpeedMultiplier(OIConstants.INTAKE_SPEED_AXIS)));

    //sets the current game pice type to cones when button 4 is pressed
    new Trigger(() -> m_operatorController.getRawButton(Bindings.setIntakeTypeCones))
      .onTrue(new InstantCommand(() -> m_intake.setState(IntakeGamePiece.CONE), m_intake));

    //sets the current game piece type to cubes when button 3 is pressed
    new Trigger(() -> m_operatorController.getRawButton(Bindings.setIntakeTypeCubes))
      .onTrue(new InstantCommand(() -> m_intake.setState(IntakeGamePiece.CUBE), m_intake));
    
    //fully lowers the arm when button 16 is pressed
    new Trigger(() -> m_operatorController.getRawButton(Bindings.lowerArm))
      .onTrue(new InstantCommand(m_arm::retract));
       
    //fully raises the arm when button 15 is pressed
    new Trigger(() -> m_operatorController.getRawButton(Bindings.raiseArm))
      .onTrue(new InstantCommand(m_arm::expand));


    //test for elevator pid
    new Trigger(() -> m_operatorController.getRawButton(Bindings.elevatorDoubleSubstation))
      .onTrue(new ElevatorCommand(m_elevator, ElevatorConstants.doubleSubstation));

   /*new Trigger(() -> m_operatorController.getRawButton(Bindings.elevatorDoubleSubstation))
    *  .onTrue(
            Commands.runOnce(
                () -> {
                  elevatorTrapezoidal.setGoal(ElevatorConstants.elevator);
                  elevatorTrapezoidal.enable();
                },
                elevatorTrapezoidal));
    */
  }


  public boolean triggerPressed() {
    if (m_driverController.getLeftTriggerAxis() != 0 || m_driverController.getRightTriggerAxis() != 0) {
      return true;
    }
    return false;
  }


  public XboxController getController() {
    return m_driverController;
  }

  public DriveSubsystem getDrive() {
    return m_robotDrive;
  }
 

  public VisionSubsystem getVision() {
    return m_vision;
  }

  public ElevatorSubsystem getElevator() {
    return m_elevator;
  }

  public ArmSubsystem getArm() {
    return m_arm;
  }

  public IntakeSubsystem getIntake() {
    return m_intake;
  }

  public AutoUtils getAutoUtils() {
    return autoUtils;
  }
}
