package frc.robot.PathPlanningCode;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/** Used as a builder to streamline the autonomous routine creation process.
 * A simple routine can be created using the structure 
 * {@link AutoRoutine new AutoRoutine.Builder.withCommand(new PrintCommand("test")).build()}
*/
public class AutoRoutine {
    private final List<Command> commandActions;

    private AutoRoutine(Builder builder) {
        this.commandActions = builder.commandActions;
    }

    public static class Builder {
        private List<Command> commandActions = new ArrayList<>();

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
                    new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD), // X controller
                    new PIDController(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD), // Y controller
                    new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD), // Rotation controller
                    container.getDrive()::setModuleStates, // Module states consumer
                    false, // Should the path be automatically mirrored depending on alliance color.
                    container.getDrive()
                )
            );
        }
    
        public Command createPath(RobotContainer container, String pathName, boolean isFirstPath, Map<String, Command> checkpoints) {
            PathPlannerTrajectory path = PathPlanner.loadPath(pathName, new PathConstraints(4, 3));
    
            FollowPathWithEvents command = new FollowPathWithEvents(
              followTrajectoryCommand(container, path, isFirstPath),
              path.getMarkers(),
              checkpoints
            );
      
            return command;
        }

        public Builder withPathCommand(RobotContainer container, String path, boolean isFirst, Map<String, Command> checkpoints) {
            commandActions.add(createPath(container, path, isFirst, checkpoints));
            return this;
        }

        public Builder withCommand(Command command) {
            commandActions.add(command);
            return this;
        }

        public AutoRoutine build() {
            return new AutoRoutine(this);
        }

    }

    public List<Command> getCommandActions() {
        return commandActions;
    }

    public SequentialCommandGroup getCommandGroup() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        List<Command> commandList = this.getCommandActions();

        for (Command command : commandList) {
            commandGroup.addCommands(command);
        }

        return commandGroup;
    }
}
