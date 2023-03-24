package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    private DoubleSolenoid sol;
    private AnalogInput input = new AnalogInput(0);
    

    public ArmSubsystem() {
        sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_forward1, Constants.SOLENOID_reverse1);
       

        SmartDashboard.putBoolean("Arm/State", false);
        SmartDashboard.putString("Arm/StateString", "retracted");
        SmartDashboard.putNumber("Arm/PressurePSI", input.getVoltage());
    }

    @Override
    public void periodic() {
        //put pressure reading here
    }

    public double getPressure() {
        return (250*input.getVoltage())-25;
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
