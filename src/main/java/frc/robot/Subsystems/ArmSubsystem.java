package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private DoubleSolenoid sol;

    public ArmSubsystem() {
        sol =
            new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                Constants.SOLENOID_forward1,
                Constants.SOLENOID_reverse1
            );
    }

    public void expand() {
        sol.set(Value.kForward);
    }

    public void retract() {
        sol.set(Value.kReverse);
    }
}
