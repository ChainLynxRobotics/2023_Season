package frc.robot.GenericPID.Extensible;

public interface DescentInjectionClosure {
    public void eval(double oldx, double error, double derrordt);
}