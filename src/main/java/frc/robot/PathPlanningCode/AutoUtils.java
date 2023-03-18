
package frc.robot.PathPlanningCode;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.ChargeStationBalanceCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ScoreGamePieceCommand;
import frc.robot.Commands.VisionTranslateCommand;
import frc.robot.Commands.VisionTurnCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoringLocation;

    /*
     * priority 1 auto:
     * go forward a bit -- score gamepiece command (arm up, elevator extended)
     * drive backwards (out of community, dependent on starting position)
     * 
     * priority 2 auto:
     * go forward a bit -- score gamepiece
     * rotate 180 degrees -- charge station balance command (tiny backup, arm down, elevator extended forward)
     * get onto charge station (dependent on starting position)
     * 
     * priority 3 auto:
     * go forward a bit -- score gamepiece
     * rotation 180 degrees
     * drive to staged gamepiece -- intake staged gamepiece command (cone or cube)
     * 
     * priorty 4 auto:
     * priority 3 auto + balance on charge station
     * 
     * 
     * 
     * starting positions: left CS (charge station), center CS, right CS
     * 
     * assuming +x dir is forward and +y dir is right
     */


public class AutoUtils {
    private SendableChooser<AutoModes> autoChooser = new SendableChooser<>();
    private SendableChooser<StartPos> startPosChooser = new SendableChooser<>();
    private SendableChooser<ScoringLocation> scoreLocationChooser = new SendableChooser<>();

    private final TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

    
    public AutoUtils() {
      autoChooser.setDefaultOption("Priority 1 Auto", AutoModes.PRIORITY_1_AUTO);
      autoChooser.addOption("Simple Trajectory", AutoModes.SIMPLE_TRAJECTORY);
      autoChooser.addOption("Auto Align Trajectory", AutoModes.AUTO_ALIGN_TRAJECTORY);
      autoChooser.addOption("Priority 2 Auto", AutoModes.PRIORITY_2_AUTO);
      autoChooser.addOption("Priority 3 Auto", AutoModes.PRIORITY_3_AUTO);
      autoChooser.addOption("Priority 4 Auto", AutoModes.PRIORITY_4_AUTO);

      startPosChooser.setDefaultOption("Left of charge station", StartPos.LEFT_CS);
      startPosChooser.addOption("Middle of charge station", StartPos.MID_CS);
      startPosChooser.addOption("Right of charge station", StartPos.RIGHT_CS);

      scoreLocationChooser.setDefaultOption("Low", ScoringLocation.LOW);
      scoreLocationChooser.addOption("Middle", ScoringLocation.MID);
      scoreLocationChooser.addOption("high", ScoringLocation.HIGH);

      SmartDashboard.putData("auto choices", autoChooser);
      SmartDashboard.putData("start position choices", startPosChooser);
      SmartDashboard.putData("initial score location", scoreLocationChooser);
    }


    public Command simpleCmdGrp(RobotContainer container) {
        return new RunCommand(() -> container.getDrive().mainDrive(0.8, 0, 0), container.getDrive()).withTimeout(2);
    }


    public Command simpleTrajectoryCommand(RobotContainer container, Trajectory trajectory) {
      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
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
          container.getDrive());

      container.getDrive().zeroHeading();
      container.getDrive().resetOdometry(trajectory.getInitialPose());

      return swerveControllerCommand.andThen(() -> container.getDrive().mainDrive(0, 0, 0));
    }

    
    public Command followTrajectoryCommand(RobotContainer container, PathPlannerTrajectory traj, boolean isFirstPath) {
      return new SequentialCommandGroup(
           new InstantCommand(() -> {
             if(isFirstPath){
                container.getDrive().resetOdometry(traj.getInitialHolonomicPose());
             }
           }),
           new PPSwerveControllerCommand(
               traj, 
               container.getDrive()::getPose, 
               DriveConstants.kDriveKinematics, 
               new PIDController(0, 0, 0), // X controller
               new PIDController(0, 0, 0), // Y controller
               new PIDController(0, 0, 0), // Rotation controller
               container.getDrive()::setModuleStates, // Module states consumer
               true, // Should the path be automatically mirrored depending on alliance color. 
               container.getDrive()
           )
       );
   }

 

    public Command trajectoryAutoAlign(RobotContainer container, Trajectory trajectory) {
      return simpleTrajectoryCommand(container, trajectory)
        .andThen(new VisionTurnCommand(container.getVision(), container.getDrive(), container.getController()));
    }

    //to do: write score command (if for low level, no additional code is needed)
    public Command priorityOneAuto(RobotContainer container, StartPos startPos, ScoringLocation location) {
      return simpleTrajectoryCommand(container, initDrive());
    }

    //just do a backup to score and then drive forward if 180 turn still offsets gyro weirdly
    public Command priorityTwoAuto(RobotContainer container, StartPos startPos) {
      return simpleTrajectoryCommand(container, initDriveToScore())
      .andThen(rotate180(container))
        .alongWith(simpleTrajectoryCommand(container, getOnChargeStation(startPos)))
          .alongWith(new ChargeStationBalanceCommand(container.getDrive(), container.getElevator()));
    }

    public Command priorityThreeAuto(RobotContainer container) {
      PathPlannerTrajectory priority3Path = PathPlanner.loadPath("Priority 3 auto", new PathConstraints(4, 3));

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("pickup", new IntakeCommand(container.getIntake(), 1));

      FollowPathWithEvents command = new FollowPathWithEvents(
        followTrajectoryCommand(container, priority3Path, true),
        priority3Path.getMarkers(),
        eventMap
      );

      return command;
    }

    public Command priorityFourAuto(RobotContainer container) {
      PathPlannerTrajectory priority4Path = PathPlanner.loadPath("Priority 4 ending", new PathConstraints(4, 3));

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("score", new ScoreGamePieceCommand(container.getElevator(), container.getIntake()));

      FollowPathWithEvents command = new FollowPathWithEvents(
        followTrajectoryCommand(container, priority4Path, false),
        priority4Path.getMarkers(),
        eventMap
      );

      return command;
    }

    public Command priorityFiveSecondPickup(RobotContainer container) {
      PathPlannerTrajectory priority5Path = PathPlanner.loadPath("Priority 5 2nd pickup", new PathConstraints(4, 3));

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("pickup2", new IntakeCommand(container.getIntake(), 1));

      FollowPathWithEvents Command = new FollowPathWithEvents(
        followTrajectoryCommand(container, priority5Path, false),
        priority5Path.getMarkers(),
        eventMap
      );

      return Command;
    }

    public Command priorityFiveExit(RobotContainer container) {
      PathPlannerTrajectory priority5Path = PathPlanner.loadPath("Priority 5 pickup exit", new PathConstraints(4, 3));

      HashMap<String, Command> eventMap = new HashMap<>();

      FollowPathWithEvents Command = new FollowPathWithEvents(
        followTrajectoryCommand(container, priority5Path, false),
        priority5Path.getMarkers(),
        eventMap
      );

      return Command;
    }

    public Command priorityFiveEnding(RobotContainer container) {
      PathPlannerTrajectory priority5Path = PathPlanner.loadPath("Priority 5 ending", new PathConstraints(4, 3));

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("score2", new ScoreGamePieceCommand(container.getElevator(), container.getIntake()));

      FollowPathWithEvents Command = new FollowPathWithEvents(
        followTrajectoryCommand(container, priority5Path, false),
        priority5Path.getMarkers(),
        eventMap
      );

      return Command;
    }

    public Command priority6Pickup(RobotContainer container) {
      PathPlannerTrajectory priority6Path = PathPlanner.loadPath("Priority 6 pickup", new PathConstraints(4, 3));

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("pickup3", new IntakeCommand(container.getIntake(), 1));

      FollowPathWithEvents Command = new FollowPathWithEvents(
        followTrajectoryCommand(container, priority6Path, false),
        priority6Path.getMarkers(),
        eventMap
      );

      return Command;
    }

    public Command priority6Ending(RobotContainer container) {
      PathPlannerTrajectory priority6Path = PathPlanner.loadPath("Priority 6 ending", new PathConstraints(4, 3));

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("score3", new ScoreGamePieceCommand(container.getElevator(), container.getIntake()));

      FollowPathWithEvents Command = new FollowPathWithEvents(
        followTrajectoryCommand(container, priority6Path, false),
        priority6Path.getMarkers(),
        eventMap
      );

      return Command;
    }


    //test
    private Trajectory simpleCurve() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          new Pose2d(3, 0, new Rotation2d(0)),
          config);

      return trajectory;
    }

    //test
    private Trajectory driveToScore() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(3, 0.5)),
        new Pose2d(2, 2, new Rotation2d(45)),
        config);

      return trajectory;
    }


    private Command rotate180(RobotContainer container) {
      return new RunCommand(() -> container.getDrive().mainDrive(0, 0, 1), container.getDrive()).withTimeout(1.64);
    }

    private Trajectory initDrive() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(),
        List.of(new Translation2d(1, 0)), new Pose2d(4, 0, new Rotation2d()), config);
      return trajectory;
    }
    
    //drive 1 m forward
    private Trajectory initDriveToScore() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(),
        List.of(new Translation2d(1, 0)), new Pose2d(1, 0, new Rotation2d()), config);
      return trajectory;
    }

    //back up or drive around charge station
    private Trajectory driveOutOfCommunity(StartPos pos) {
      Trajectory trajectory;
      if (pos == StartPos.LEFT_CS || pos == StartPos.RIGHT_CS) {
        trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(),
          List.of(new Translation2d(-3, 0)),
          new Pose2d(-3.1, -0.1, new Rotation2d()),
          config);
        return trajectory;
      } else if (pos == StartPos.MID_CS) {
        trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(),
          List.of(new Translation2d(-3.9, -4)),
          new Pose2d(-4, -4, new Rotation2d(0)),
          config);
        return trajectory;
      }
      return null;
    }


    private Trajectory getOnChargeStation(StartPos pos) {
      Trajectory trajectory;
        if (pos == StartPos.LEFT_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(), 
            List.of(new Translation2d(2, 2)), 
            new Pose2d(3,2, new Rotation2d()),
            config);
          return trajectory;
        } else if (pos == StartPos.MID_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), 
            List.of(new Translation2d(-2, 0)),
            new Pose2d(-3, 0.1, new Rotation2d(0)),
            config);
          return trajectory;
        } else if (pos == StartPos.RIGHT_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(new Translation2d(2, -2)),
            new Pose2d(3, -2, new Rotation2d()),
            config);
          return trajectory;
        }
        return null;
    }

    private Trajectory driveToStagedGamePiece(StartPos pos) {
      Trajectory trajectory;
        if (pos == StartPos.LEFT_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(new Translation2d(5.9, 0)),
            new Pose2d(5.9, 0.4, new Rotation2d()),
            config);
          return trajectory;
        } else if (pos == StartPos.MID_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(new Translation2d(0, 1.8)),
            new Pose2d(5.9, 1.8, new Rotation2d()),
            config);
          return trajectory;
        } else if (pos == StartPos.RIGHT_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(new Translation2d(5.9, 0)),
            new Pose2d(5.9, -0.4, new Rotation2d()),
            config);
          return trajectory;
        }
        return null;
    }


  //start position chooser is fed into this for start position-dependent trajectories
  public Command chooseAuto(RobotContainer container, StartPos startPos, ScoringLocation location) {
    switch(autoChooser.getSelected()) {
      case SIMPLE_TRAJECTORY:
        return simpleTrajectoryCommand(container, simpleCurve());
      case AUTO_ALIGN_TRAJECTORY:
        return trajectoryAutoAlign(container, driveToScore());
      case PRIORITY_1_AUTO:
        return priorityOneAuto(container, startPos, location);
      case PRIORITY_2_AUTO:
        return priorityTwoAuto(container, startPos);
      case PRIORITY_3_AUTO:
        return priorityThreeAuto(container);
      case PRIORITY_4_AUTO:
        return priorityThreeAuto(container).andThen(priorityFourAuto(container));
      case PRIORITY_5_AUTO:
        return priorityThreeAuto(container).andThen(priorityFiveSecondPickup(container)).andThen(priorityFiveExit(container)).andThen(priorityFiveEnding(container));
      case PRIORITY_6_AUTO:
        return priorityThreeAuto(container).andThen(priorityFiveSecondPickup(container)).andThen(priorityFiveExit(container)).andThen(priority6Pickup(container)).andThen(priority6Ending(container));
      default:
        return simpleCmdGrp(container);
    }
  }

  public StartPos chooseStartPos() {
    switch(startPosChooser.getSelected()) {
      case LEFT_CS:
        return StartPos.LEFT_CS;
      case MID_CS:
        return StartPos.MID_CS;
      default:
        return StartPos.RIGHT_CS;
    }
  }

  public ScoringLocation chooseInitScoreLocation() {
    switch(scoreLocationChooser.getSelected()) {
      case MID:
        return ScoringLocation.MID;
      case HIGH:
        return ScoringLocation.HIGH;
      default:
        return ScoringLocation.LOW;
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

  private enum StartPos {
    LEFT_CS,
    MID_CS,
    RIGHT_CS
  }
}
