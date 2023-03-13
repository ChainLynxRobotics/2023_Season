package frc.robot.Subsystems;



import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
    private DoubleSolenoid sol;
    public static boolean extendedState = false;


    public ArmSubsystem() {
        sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_forward1, Constants.SOLENOID_reverse1);
    }
    

    public void expand() {
            sol.set(Value.kForward);
            extendedState = true; //tracks if the command to engage has been sent
            SmartDashboard.putBoolean("FourBar/extendedState", extendedState);
    }

    public void retract() {
            sol.set(Value.kReverse);
            extendedState = false; //tracsk if the comand to retract has been sent
            SmartDashboard.putBoolean("FourBar/extendedState", extendedState);
    }
}