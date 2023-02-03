package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotorInner;
    private CANSparkMax intakeMotorOuter;

    public IntakeSubsystem() {
        intakeMotorInner = new CANSparkMax(Constants.INTAKE_MOTOR_INNER, MotorType.kBrushless);
        intakeMotorOuter = new CANSparkMax(Constants.INTAKE_MOTOR_OUTER, MotorType.kBrushless);

        intakeMotorInner.setIdleMode(IdleMode.kBrake);
        intakeMotorInner.restoreFactoryDefaults();
        intakeMotorOuter.setIdleMode(IdleMode.kBrake);
        intakeMotorOuter.restoreFactoryDefaults();
    }

    public void releaseGamePiece(double speedMultiplier) {
        if (Constants.curGamePiece == GamePiece.CONE) {
            releaseCone(speedMultiplier);
        } else if (Constants.curGamePiece == GamePiece.CUBE) {
            releaseCube(speedMultiplier);
        }
    }

    public void intakeGamePiece(double speedMultiplier) {
        if (Constants.curGamePiece == GamePiece.CONE) {
            pickUpCone(speedMultiplier);
        } else if (Constants.curGamePiece == GamePiece.CUBE) {
            pickUpCube(speedMultiplier);
        }
    }

    public void pickUpCone(double speedMultiplier) {
        intakeMotorInner.set(-speedMultiplier);
        intakeMotorOuter.set(-speedMultiplier);
    }

    public void releaseCone(double speedMultiplier) {
        intakeMotorInner.set(speedMultiplier);
        intakeMotorOuter.set(speedMultiplier);
    }

    public void pickUpCube(double speedMultiplier) {
        intakeMotorInner.set(speedMultiplier);
        intakeMotorOuter.set(-speedMultiplier);
    }

    public void releaseCube(double speedMultiplier) {
        intakeMotorInner.set(-speedMultiplier);
        intakeMotorOuter.set(speedMultiplier);
    }

    public void stopMotors() {
        intakeMotorInner.set(0);
        intakeMotorOuter.set(0);
    }

}