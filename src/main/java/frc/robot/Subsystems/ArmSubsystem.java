package frc.robot.Subsystems;



import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
   /*  private DoubleSolenoid sol1;
    private DoubleSolenoid sol2;
    private DoubleSolenoid sol3;
    private DoubleSolenoid sol4;
    private DoubleSolenoid[] sols;

    public ArmSubsystem() {
        sol1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_forward1, Constants.SOLENOID_reverse1);
        sol2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_forward2, Constants.SOLENOID_reverse2);
        sol3 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_forward3, Constants.SOLENOID_reverse3);
        sol4 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_forward4, Constants.SOLENOID_reverse4);

        sols = new DoubleSolenoid[]{sol1, sol2, sol3, sol4};
        for (DoubleSolenoid sol : sols) {
            try {
                sol.wait(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }


    public void expand() {
        for (DoubleSolenoid sol : sols) {
            sol.set(Value.kForward);
        }
    }

    public void retract() {
        for (DoubleSolenoid sol : sols) {
            sol.set(Value.kReverse);
        }
    }


    public void turnOff() {
        for (DoubleSolenoid sol : sols) {
            sol.set(Value.kOff);
        }
    }*/

}