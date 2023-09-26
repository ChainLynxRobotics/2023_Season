package frc.robot;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.VisionTurnCommand;
import frc.robot.Constants.Bindings;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.ElevatorManualControlCommand;
import frc.robot.Commands.ReleaseCommand;
import frc.robot.Commands.SimpleDriveCommand;
import frc.robot.Commands.VisionTranslateCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Pairings;
import frc.robot.PathPlanningCode.AutoRoutine;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import frc.robot.Commands.ModifiedCSBalanceCommand;


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
  XboxController m_driverController = new XboxController(
    OIConstants.kDriverControllerPort
  );
  Joystick m_operatorController = new Joystick(
    OIConstants.kIOperatorControllerPort
  );

  public RobotContainer() {
    SmartDashboard.putBoolean("GamePiece/CubeMode", false);
    SmartDashboard.putBoolean("GamePiece/ConeMode", false);

    m_robotDrive = new DriveSubsystem();
    m_vision = new VisionSubsystem();
    m_elevator = new ElevatorSubsystem();
    m_arm = new ArmSubsystem();
    m_intake = new IntakeSubsystem();

    configureButtonBindings();
    CameraServer.startAutomaticCapture("intake camera", 0);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () ->
          m_robotDrive.drive(
            MathUtil.applyDeadband(
              -m_driverController.getLeftY(),
              OIConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
              -m_driverController.getLeftX(),
              OIConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
              -m_driverController.getRightX(),
              OIConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
              -m_driverController.getRightY(),
              OIConstants.kDriveDeadband
            ),
            m_driverController.getRightBumper(),
            m_driverController.getAButton()
          ),
        m_robotDrive
      )
    );
  }

  private void configureButtonBindings() {
    new Trigger(() -> m_driverController.getBButton())
      .whileTrue(
        new VisionTurnCommand(m_vision, m_robotDrive, m_driverController)
      );

    new Trigger(() -> m_driverController.getYButton())
      .onTrue(
        new VisionTranslateCommand(m_vision, m_robotDrive, m_driverController)
      );

    new Trigger(() -> triggerPressed())
      .whileTrue(new SimpleDriveCommand(m_robotDrive, m_driverController));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.releaseGamePiece))
      .whileTrue(
        new ReleaseCommand(
          m_intake,
          m_intake.getWAxisSpeedMultiplier(OIConstants.INTAKE_SPEED_AXIS)
        )
      );

    new Trigger(() -> m_operatorController.getRawButton(Bindings.intakeGamePiece))
      .whileTrue(
        new IntakeCommand(
          m_intake,
          m_intake.getWAxisSpeedMultiplier(OIConstants.INTAKE_SPEED_AXIS)
        )
      );

    //sets the current game piece type to cones when button 4 is pressed
    new Trigger(() -> m_operatorController.getRawButton(Bindings.setGamePieceCone))
      .onTrue(
        new InstantCommand(
          () -> m_intake.setState(GamePiece.CONE),
          m_intake
        )
      );

    //sets the current game piece type to cubes when button 3 is pressed
    new Trigger(() -> m_operatorController.getRawButton(Bindings.setGamePieceCube))
      .onTrue(
        new InstantCommand(
          () -> m_intake.setState(GamePiece.CUBE),
          m_intake
        )
      );

    POVButton upPOV = new POVButton(m_operatorController, 0);
    POVButton downPOV = new POVButton(m_operatorController, 180);

    upPOV.onTrue(new InstantCommand(m_arm::expand));
    downPOV.onTrue(new InstantCommand(m_arm::retract));

    int[] pickups = new int[]{Bindings.groundPickUp, Bindings.lowScoreElevatorSetpoint, Bindings.midScoreElevatorSetpoint, Bindings.highScoreElevatorSetpoint, Bindings.doubleSubstationSetpoint, Bindings.fullRetraction};
    for (int pickup : pickups) {
      instantiateTriggerBinding(pickup);
    }

    new Trigger(() -> m_operatorController.getRawButton(Bindings.manualElevatorControl))
      .whileTrue(new ElevatorManualControlCommand(m_operatorController, m_elevator));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.chargeStationBalance))
      .onTrue(new ModifiedCSBalanceCommand(m_robotDrive, m_elevator, m_operatorController, true));
  }

  private Trigger instantiateTriggerBinding(int binding) {
    double setpoint;

    if (binding == Bindings.highScoreElevatorSetpoint) {
      setpoint = ElevatorConstants.highElevatorConeSetpoint; //default
    } else {
      setpoint = Pairings.bindingsToSetpoints.get(binding);
    }
     
    return new Trigger(() -> m_operatorController.getRawButton(binding))
      .onTrue(
        //if a button corresponds to 2 diff setpoints according to gamepiece, reassign setpoint value if necessary
        new InstantCommand(() -> getGamePieceHighSetpoint(setpoint), m_intake)
        .andThen(new ElevatorCommand(
          m_elevator,
          setpoint)));
  }

  //get button setpoint according to current game piece
  private void getGamePieceHighSetpoint(double input) {
    GamePiece gamePiece = m_intake.getState();
    double setpoint = gamePiece == GamePiece.CONE ? ElevatorConstants.highElevatorConeSetpoint : ElevatorConstants.highElevatorCubeSetpoint;
    input = setpoint;
  }


  public boolean triggerPressed() {
    if (
      m_driverController.getLeftTriggerAxis() != 0 ||
      m_driverController.getRightTriggerAxis() != 0
    ) {
      return true;
    }
    return false;
  }

  public XboxController getController() {
    return m_driverController;
  }

  public Joystick getOperatorController() {
    return m_operatorController;
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

  public Command getAutoCommand() {
    AutoRoutine.Builder builder = new AutoRoutine.Builder();
    AutoRoutine testRoutine = builder
      .withPathCommand(this, "Priority 1 auto", true, Map.of("init retract p1a", new IntakeCommand(m_intake, 0.5)))
      .build();
    return testRoutine.getCommandGroup();
  }
}
