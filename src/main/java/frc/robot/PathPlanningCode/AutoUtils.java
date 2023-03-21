package frc.robot.PathPlanningCode;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Commands.ChargeStationBalanceCommand;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ReleaseCommand;
import frc.robot.Commands.ScoreGamePieceCommand;
import frc.robot.Commands.VisionTurnCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Bindings;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;


public class AutoUtils {

    private SendableChooser<AutoModes> autoChooser = new SendableChooser<>();

    //stores all the command bindings along path checkpoints
    private HashMap<String, Command> eventMap = new HashMap<>();

    public AutoUtils() {
        autoChooser.setDefaultOption("Priority 1 Auto", AutoModes.PRIORITY_1_AUTO);
        autoChooser.addOption("Priority 2 Auto", AutoModes.PRIORITY_2_AUTO);
        autoChooser.addOption("Priority 3 Auto", AutoModes.PRIORITY_3_AUTO);
        autoChooser.addOption("Priority 4 Auto", AutoModes.PRIORITY_4_AUTO);
        autoChooser.addOption("Priority 5 Auto", AutoModes.PRIORITY_5_AUTO);
        autoChooser.addOption("Priority 6 Auto", AutoModes.PRIORITY_6_AUTO);

        SmartDashboard.putData("auto choices", autoChooser);
    }

    private Command getRetractCommand(RobotContainer container) {
      Command initPosCommand = new SequentialCommandGroup(
        new ElevatorCommand(
            container.getElevator(), 
            container.getIntake(), 
            Bindings.fullRetraction),
        new InstantCommand(container.getArm()::retract));

        return initPosCommand;
    }

    public Command simpleCmdGrp(RobotContainer container) {
        return new RunCommand(
            () -> container.getDrive().mainDrive(0.8, 0, 0),
            container.getDrive()
        )
            .withTimeout(2);
    }

    public Command simpleTrajectoryCommand(
        RobotContainer container,
        Trajectory trajectory
    ) {
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            container.getDrive()::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            container.getDrive()::setModuleStates,
            container.getDrive()
        );

        container.getDrive().zeroHeading();
        container.getDrive().resetOdometry(trajectory.getInitialPose());

        return swerveControllerCommand.andThen(() ->
            container.getDrive().mainDrive(0, 0, 0)
        );
    }

    public Command followTrajectoryCommand(RobotContainer container, PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if (isFirstPath) {
                    container
                        .getDrive()
                        .resetOdometry(traj.getInitialHolonomicPose());
                }
            }),

            new PPSwerveControllerCommand(
                traj,
                container.getDrive()::getPose,
                DriveConstants.kDriveKinematics,
                new PIDController(1, 0, 0), // X controller
                new PIDController(1, 0, 0), // Y controller
                new PIDController(5, 0, 0), // Rotation controller
                container.getDrive()::setModuleStates, // Module states consumer
                false, // Should the path be automatically mirrored depending on alliance color.
                container.getDrive()
            )
        );
    }

    public Command createPath(RobotContainer container, String pathName, boolean isFirstPath, Map<String, Command> checkpoints) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, new PathConstraints(4, 3));

        int entry = 0;
        while (entry < checkpoints.size()) {
            String key = checkpoints.keySet().iterator().next();
            if (!eventMap.containsKey(key)) {
                eventMap.put(key, checkpoints.get(key));
            }
            entry++;
        }
        PathPlanner.loadPathGroup(null,null,null,null);

        FollowPathWithEvents command = new FollowPathWithEvents(
          followTrajectoryCommand(container, path, isFirstPath),
          path.getMarkers(),
          eventMap
        );
  
        return command;
    }


    //if event map doesn't work (it should), put stuff in a sequential command group
    public Command priorityOneAuto(RobotContainer container) {
        return createPath(
            container, 
            "Priority 1 auto", 
            true, 
            Map.of("init score p1a", new SequentialCommandGroup(
              new InstantCommand(container.getArm()::expand),
              new ElevatorCommand(
                container.getElevator(), 
                container.getIntake(), 
                Bindings.midScoreElevatorSetpoint),
              new ReleaseCommand(
                container.getIntake(), 
                0.8))));
    }


    

    public Command priorityTwoAuto(RobotContainer container) {

        return new SequentialCommandGroup(
          new SequentialCommandGroup(
            new InstantCommand(container.getArm()::expand),
            new ElevatorCommand(
              container.getElevator(), 
              container.getIntake(), 
              Bindings.midScoreElevatorSetpoint),
            new ReleaseCommand(
              container.getIntake(), 
              0.8)),
          createPath(
            container, 
            "Priority 2 auto", 
            true, 
            Map.of("init retract p2a", getRetractCommand(container))),
          new ChargeStationBalanceCommand(
            container.getDrive(), 
            container.getElevator(), 
            container.getOperatorController()));
    }

    public Command priorityThreeAuto(RobotContainer container) {
        return createPath(
            container, 
            "Priority 3 auto", 
            true, 
            Map.of(
                "init score p3a", new ScoreGamePieceCommand(
                  container.getElevator(),
                  container.getIntake(),
                  container.getArm(),
                  Bindings.midScoreElevatorSetpoint),
                "intake p3a", new IntakeCommand(container.getIntake(), 0.8).withTimeout(2),
                "end score p3a", new ScoreGamePieceCommand(
                  container.getElevator(),
                  container.getIntake(),
                  container.getArm(),
                  Bindings.midScoreElevatorSetpoint)));
    }

    public Command priorityFourAuto(RobotContainer container) {
        return new SequentialCommandGroup(
          createPath(
            container, 
            "Priority 4 ending", 
            false, 
            Map.of()),
          new ChargeStationBalanceCommand(
            container.getDrive(), 
            container.getElevator(), 
            container.getOperatorController())
        );
    }
    

    public Command priorityFiveSecondPickup(RobotContainer container) {
      return createPath(
            container, 
            "Priority 5 2nd pickup", 
            false, 
            Map.of("intake p5a", 
              new SequentialCommandGroup(
                new ElevatorCommand(container.getElevator(), container.getIntake(), Bindings.groundPickUp),
                new IntakeCommand(container.getIntake(), 0.8).withTimeout(1)
              )));
    }

    public Command priorityFiveExit(RobotContainer container) {
      return createPath(
            container, 
            "Priority 5 pickup exit", 
            false, 
            Map.of());
    }

    public Command priorityFiveEnding(RobotContainer container) {
      return createPath(
            container, 
            "Priority 5 ending", 
            false, 
            Map.of(
              "end score p5a", new ScoreGamePieceCommand(
                container.getElevator(),
                container.getIntake(),
                container.getArm(),
                Bindings.midScoreElevatorSetpoint), 
              "balance p5a", new ChargeStationBalanceCommand(
                container.getDrive(),
                container.getElevator(), 
                container.getOperatorController()))
              );
    }

    public Command priority6Pickup(RobotContainer container) {
      return createPath(
            container, 
            "Priority 6 pickup", 
            false, 
            Map.of(
              "score p6a", new ScoreGamePieceCommand(
                container.getElevator(),
                container.getIntake(),
                container.getArm(),
                Bindings.midScoreElevatorSetpoint),
              "intake p6a", new SequentialCommandGroup(
                new ElevatorCommand(
                  container.getElevator(), 
                  container.getIntake(), 
                  Bindings.groundPickUp),
                  new IntakeCommand(
                  container.getIntake(),
                  0.8).withTimeout(1)
              )));
    }


    public Command priority6Ending(RobotContainer container) {
      return createPath(
        container, 
        "Priority 6 ending", 
        false, 
        Map.of(
          "end score p6a", new ScoreGamePieceCommand(
            container.getElevator(),
            container.getIntake(),
            container.getArm(),
            Bindings.midScoreElevatorSetpoint),
          "balance p6a", new ChargeStationBalanceCommand(
            container.getDrive(), 
            container.getElevator(), 
            container.getOperatorController())));
    }


    public Command trajectoryAutoAlign(RobotContainer container,Trajectory trajectory) {
        return simpleTrajectoryCommand(container, trajectory)
            .andThen(
                new VisionTurnCommand(
                    container.getVision(),
                    container.getDrive(),
                    container.getController()
                )
            );
    }


  //start position chooser is fed into this for start position-dependent trajectories
  public Command chooseAuto(RobotContainer container) {
    switch(autoChooser.getSelected()) {
      case PRIORITY_1_AUTO:
        return priorityOneAuto(container);
      case PRIORITY_2_AUTO:
        return priorityTwoAuto(container);
      case PRIORITY_3_AUTO:
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
            .andThen(priority6Ending(container));
      default:
       return priorityOneAuto(container);
        }
    }


    public SendableChooser<AutoModes> getChooser() {
        return autoChooser;
    }


  private enum AutoModes {
    SIMPLE_DRIVE,
    SIMPLE_TRAJECTORY,
    AUTO_ALIGN_TRAJECTORY, 
    PRIORITY_1_AUTO,
    PRIORITY_2_AUTO,
    PRIORITY_3_AUTO,
    PRIORITY_4_AUTO,
    PRIORITY_5_AUTO,
    PRIORITY_6_AUTO
  }
}
