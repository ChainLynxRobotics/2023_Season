package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax intakeMotorInner;
    private CANSparkMax intakeMotorOuter;

    private static boolean coneHold;
    private GamePiece state;

    public GamePiece getState() {
        return state;
    }

    public void setState(GamePiece state) {
        this.state = state;
        if (state == GamePiece.CUBE) {
            SmartDashboard.putBoolean("Intake/Mode", true);
        } else {
            SmartDashboard.putBoolean("Intake/Mode", false);
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

        state = GamePiece.CUBE;

        coneHold = false;
        SmartDashboard.putBoolean("Intake/coneHold", coneHold);
    }

    public double getWAxisSpeedMultiplier(double axisValue) {
        double mult = (axisValue + 1) / 2;
        return MathUtil.clamp(Math.log(mult * 10), 0.1, 0.8);
    }

    public void releaseGamePiece(double speedMultiplier) {
        if (state == GamePiece.CONE) {
            releaseCone(speedMultiplier);
        } else if (state == GamePiece.CUBE) {
            releaseCube(speedMultiplier);
        }
    }

    public void intakeGamePiece(double speedMultiplier) {
        if (state == GamePiece.CONE) {
            pickUpCone(speedMultiplier);
        } else if (state == GamePiece.CUBE) {
            pickUpCube(speedMultiplier);
        }
    }

    public void pickUpCone(double speedMultiplier) {
        intakeMotorInner.set(-0.4);
        intakeMotorOuter.set(-0.4);
        coneHold = true;
    }

    public void releaseCone(double speedMultiplier) {
        intakeMotorInner.set(0.4);
        intakeMotorOuter.set(0.4);
        coneHold = false;
    }

    public void pickUpCube(double speedMultiplier) {
        intakeMotorInner.set(0.4);
        intakeMotorOuter.set(-0.4);
        coneHold = false;
    }

    public void releaseCube(double speedMultiplier) {
        intakeMotorInner.set(-0.4);
        intakeMotorOuter.set(0.4);
        coneHold = false;
    }

    public void stopMotors() {
        if (coneHold) {
            intakeMotorInner.set(-0.05);
            intakeMotorOuter.set(-0.05);
        } else {
            intakeMotorInner.set(0);
            intakeMotorOuter.set(0);
        }
        
    }
}
