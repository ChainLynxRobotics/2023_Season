package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeGamePiece;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax intakeMotorInner;
    private CANSparkMax intakeMotorOuter;

    private IntakeGamePiece state;

    public IntakeGamePiece getState() {
        return state;
    }

    public void setState(IntakeGamePiece state) {
        this.state = state;
        if (state == IntakeGamePiece.CUBE) {
            SmartDashboard.putBoolean("GamePiece/ConeMode", false);
            SmartDashboard.putBoolean("GamePiece/CubeMode", true);
        } else {
            SmartDashboard.putBoolean("GamePiece/ConeMode", true);
            SmartDashboard.putBoolean("GamePiece/CubeMode", false);
        }
    }

    public IntakeSubsystem() {
        intakeMotorInner =
            new CANSparkMax(Constants.INTAKE_MOTOR_INNER, MotorType.kBrushless);
        intakeMotorOuter =
            new CANSparkMax(Constants.INTAKE_MOTOR_OUTER, MotorType.kBrushless);

        intakeMotorInner.setIdleMode(IdleMode.kBrake);
        intakeMotorInner.restoreFactoryDefaults();
        intakeMotorOuter.setIdleMode(IdleMode.kBrake);
        intakeMotorOuter.restoreFactoryDefaults();

        state = IntakeGamePiece.CUBE;
    }

    public double getWAxisSpeedMultiplier(double axisValue) {
        double mult = (axisValue + 1) / 2;
        return MathUtil.clamp(Math.log(mult * 10), 0.1, 0.8);
    }

    public void releaseGamePiece(double speedMultiplier) {
        if (state == IntakeGamePiece.CONE) {
            releaseCone(speedMultiplier);
        } else if (state == IntakeGamePiece.CUBE) {
            releaseCube(speedMultiplier);
        }
    }

    public void intakeGamePiece(double speedMultiplier) {
        if (state == IntakeGamePiece.CONE) {
            pickUpCone(speedMultiplier);
        } else if (state == IntakeGamePiece.CUBE) {
            pickUpCube(speedMultiplier);
        }
    }

    public void pickUpCone(double speedMultiplier) {
        // TODO: tune motor speeds
        intakeMotorInner.set(-0.4);
        intakeMotorOuter.set(-0.4);
    }

    public void releaseCone(double speedMultiplier) {
        // TODO:tune motor speeds
        intakeMotorInner.set(0.4);
        intakeMotorOuter.set(0.4);
    }

    public void pickUpCube(double speedMultiplier) {
        // TODO:tune motor speeds
        intakeMotorInner.set(0.4);
        intakeMotorOuter.set(-0.4);
    }

    public void releaseCube(double speedMultiplier) {
        // TODO:tune motor speeds
        intakeMotorInner.set(-0.4);
        intakeMotorOuter.set(0.4);
    }

    public void stopMotors() {
        intakeMotorInner.set(0);
        intakeMotorOuter.set(0);
    }
}
