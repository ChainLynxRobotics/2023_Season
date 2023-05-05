package frc.robot.PathPlanningCode;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;

/** Used as a builder to streamline the autonomous routine creation process.
 * A simple routine can be created using the structure 
 * {@link AutoRoutineBuilder new AutoBuilder.Builder.withCommand(new RunCommand()).withPath("path name").build()}
*/
public class AutoRoutineBuilder {
    private final List<Consumer<Command>> commandActions;
    private final List<String> path;

    private AutoRoutineBuilder(Builder builder) {
        this.commandActions = builder.commandActions;
        this.path = builder.path;
    }

    public static class Builder {
        private List<Consumer<Command>> commandActions;
        private List<String> path;

        public Builder withCommand(Consumer<Command> command) {
            this.commandActions.add(command);
            return this;
        }

        public Builder withPath(String path, Command... concurrent) {
            this.path.add(path);
            return this;
        }

        public void acceptCommand(Command command) {
            for (Consumer<Command> consumer : this.commandActions) {
                consumer.accept(command);
            }
        }

        public AutoRoutineBuilder build() {
            return new AutoRoutineBuilder(this);
        }
    }

    public List<Consumer<Command>> getCommandActions() {
        return commandActions;
    }

    public List<String> getPath() {
        return path;
    }
    
    //AutoRoutineBuilder builder = AutoRoutineBuilder.Builder().withCommand(null).build();
}
