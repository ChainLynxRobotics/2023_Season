package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private DoubleSolenoid sol;

    public ArmSubsystem() {
        sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_forward1, Constants.SOLENOID_reverse1);
        SmartDashboard.putBoolean("Arm/State", false);
        SmartDashboard.putString("Arm/StateString", "retracted");
    }

    public void expand() {
        sol.set(Value.kForward);
        SmartDashboard.putBoolean("Arm/State", true);
        SmartDashboard.putString("Arm/StateString", "extended");
    }

    public void retract() {
        sol.set(Value.kReverse);
        SmartDashboard.putBoolean("Arm/State", false);
        SmartDashboard.putString("Arm/StateString", "retracted");
    }
}
