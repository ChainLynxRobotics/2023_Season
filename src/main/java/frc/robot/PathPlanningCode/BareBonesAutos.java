package frc.robot.PathPlanningCode;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ChargeStationBalanceCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.ReleaseCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Bindings;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;


public class BareBonesAutos {

    private SendableChooser<AutoModes> autoChooser = new SendableChooser<>();

    //stores all the command bindings along path checkpoints
    private HashMap<String, Command> eventMap = new HashMap<>();

    public BareBonesAutos(RobotContainer container) {

        configureAutoCommands(container);
        SmartDashboard.putData("auto choices", autoChooser);
    }

    private void configureAutoCommands(RobotContainer container) {
        Command scoreCommand = new SequentialCommandGroup(
            new InstantCommand(container.getArm()::expand),
            new ElevatorCommand(
              container.getElevator(), 
              container.getIntake(), 
              Bindings.midScoreElevatorSetpoint),
            new ReleaseCommand(
              container.getIntake(), 
              0.8));

        Command balanceCommand = new ChargeStationBalanceCommand(
            container.getDrive(), 
            container.getElevator(), 
            container.getOperatorController());

        Command intakeCommand = new SequentialCommandGroup(
            new ElevatorCommand(
                container.getElevator(), 
                container.getIntake(), 
                Bindings.groundPickUp)
        );
        
        //add all key-value pairs to event map
        eventMap.put("score test", new ReleaseCommand(container.getIntake(), 0.8));
        eventMap.put("marker print", new ReleaseCommand(container.getIntake(), 0.8));
        //should only actually need to create 3 of these, but I don't want to rename everything and mess up AutoUtils
        eventMap.put("init score p1a", scoreCommand);
        eventMap.put("init score p2a", scoreCommand);
        eventMap.put("balance p2a", balanceCommand);
        eventMap.put("init score p3a", scoreCommand);
        eventMap.put("intake p3a", intakeCommand);
        eventMap.put("end score p3a", scoreCommand);
        eventMap.put("balance p4a", balanceCommand);
        eventMap.put("intake p5a", intakeCommand);
        eventMap.put("score p5a", scoreCommand);
        eventMap.put("balance p5a", balanceCommand);
        eventMap.put("score p6a", scoreCommand);
        eventMap.put("intake p6a", intakeCommand);
        eventMap.put("score p6a", scoreCommand);
        eventMap.put("balance p6a", balanceCommand);

        autoChooser.setDefaultOption("Test Auto", AutoModes.TEST_AUTO);
        autoChooser.addOption("Priority 1 Auto", AutoModes.PRIORITY_1_AUTO);
        autoChooser.addOption("Priority 2 Auto", AutoModes.PRIORITY_2_AUTO);
    }


public static Command followTrajectoryCommand(RobotContainer container, PathPlannerTrajectory trajectory, boolean stopAtEnd) {

  var thetaController =
      new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController,
          AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
  thetaController.enableContinuousInput(-Math.PI, Math.PI);

  Command swerveControllerCommand = new PPSwerveControllerCommand(
    trajectory,
    container.getDrive()::getPose,
    DriveConstants.kDriveKinematics,
    new PIDController(1, 0, 0), // X controller
    new PIDController(1, 0, 0), // Y controller
    new PIDController(5, 0, 0), // Rotation controller
    container.getDrive()::setModuleStates, // Module states consumer
    false, // Should the path be automatically mirrored depending on alliance color.
    container.getDrive());

  if (stopAtEnd) {
    // Stop at the end. A good safe default, but not desireable if running two paths back to back
    swerveControllerCommand =
        swerveControllerCommand.andThen(() -> container.getDrive().mainDrive(0, 0, 0));
  }
  return swerveControllerCommand;
}

//works as expected! is drivetrain stopping when release command is called? -if so, separate into 2 paths and forget about event map
private Command testAuto(RobotContainer container) {
    PathPlannerTrajectory path = PathPlanner.loadPath("Test Path", new PathConstraints(4, 3));

    Command command = new SequentialCommandGroup(
      new ReleaseCommand(container.getIntake(), 0.8).withTimeout(2),
      new FollowPathWithEvents(
        followTrajectoryCommand(container, path, false),
        path.getMarkers(),
        eventMap),
      new PrintCommand("middle"));

    return command;
}


private Command priorityOneAuto(RobotContainer container) {
    PathPlannerTrajectory path = PathPlanner.loadPath("Priority 1 auto", new PathConstraints(4, 3));
    FollowPathWithEvents cmd = new FollowPathWithEvents(followTrajectoryCommand(container, path, true), path.getMarkers(), eventMap);
    return followTrajectoryCommand(container, path, true);
} 

private Command priorityTwoAuto(RobotContainer container) {
    PathPlannerTrajectory path = PathPlanner.loadPath("Priority 2 auto", new PathConstraints(4, 3));
    
    FollowPathWithEvents command = new FollowPathWithEvents(
    followTrajectoryCommand(container, path, false),
    path.getMarkers(),
    eventMap);

    return command;
} 




  //start position chooser is fed into this for start position-dependent trajectories
  public Command chooseAuto(RobotContainer container) {
    switch(autoChooser.getSelected()) {
      case PRIORITY_1_AUTO:
        return priorityOneAuto(container);
      case PRIORITY_2_AUTO:
        return priorityTwoAuto(container);
      /*case PRIORITY_3_AUTO:
        return priorityThreeAuto(container);
      case PRIORITY_4_AUTO:
        return priorityThreeAuto(container)
            .andThen(priorityFourAuto(container));
      case PRIORITY_5_AUTO:
        return priorityThreeAuto(container)
            .andThen(priorityFiveSecondPickup(container))
            .andThen(priorityFiveExit(container))
            .andThen(priorityFiveEnding(container));
      case PRIORITY_6_AUTO:
        return priorityThreeAuto(container)
            .andThen(priorityFiveSecondPickup(container))
            .andThen(priorityFiveExit(container))
            .andThen(priority6Pickup(container))
            .andThen(priority6Ending(container));*/
      default:
       return testAuto(container);
        }
    }


    public SendableChooser<AutoModes> getChooser() {
        return autoChooser;
    }


  private enum AutoModes {
    TEST_AUTO,
    PRIORITY_1_AUTO,
    PRIORITY_2_AUTO,
    PRIORITY_3_AUTO,
    PRIORITY_4_AUTO,
    PRIORITY_5_AUTO,
    PRIORITY_6_AUTO
  }
}
