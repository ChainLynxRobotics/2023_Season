package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends CommandBase {

    private IntakeSubsystem intake;
    private double intakeSpeedMultiplier;
    private GamePiece gamePiece;

    //intakes cube or cone
    public AutoIntakeCommand(IntakeSubsystem intake, double intakeSpeedMultiplier, GamePiece gamePiece) {
        this.intake = intake;
        this.intakeSpeedMultiplier = intakeSpeedMultiplier;
        this.gamePiece = gamePiece;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(gamePiece);
        intake.intakeGamePiece(intakeSpeedMultiplier);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopMotors();
    }
}
