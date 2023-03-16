package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


//TO DO: error handling for if pid controllers get misaligned
public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax elevatorMotor1;
    private final CANSparkMax elevatorMotor2;

    private final SparkMaxPIDController m_pidController1;
    private final RelativeEncoder m_encoder1;
    private final RelativeEncoder m_encoder2;
    public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, elevatorSpeed;

    public ElevatorSubsystem() {
      elevatorMotor1 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID_MASTER, MotorType.kBrushless);
      elevatorMotor2 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID_SLAVE, MotorType.kBrushless);

      elevatorMotor1.restoreFactoryDefaults();

      m_encoder1 = elevatorMotor1.getEncoder();
      m_encoder2 = elevatorMotor2.getEncoder();
      m_encoder1.setPosition(0);
      m_encoder2.setPosition(0);

      elevatorMotor1.setIdleMode(IdleMode.kCoast);
      elevatorMotor2.setIdleMode(IdleMode.kCoast);
      elevatorMotor1.setInverted(true);
      elevatorMotor2.follow(elevatorMotor1, true);
      elevatorMotor1.clearFaults();
      elevatorMotor2.clearFaults();
      

      m_pidController1 = elevatorMotor1.getPIDController();

      // PID coefficients
      kP = 1; 
      kI = 1e-4;
      kD = 0.0001; 
      kIz = 0; 
      kFF = 0; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      elevatorSpeed = 0;
  
      // set PID coefficients
      m_pidController1.setP(kP);
      m_pidController1.setI(kI);
      m_pidController1.setD(kD);
      m_pidController1.setIZone(kIz);
      m_pidController1.setFF(kFF);
      m_pidController1.setOutputRange(kMinOutput, kMaxOutput);

  
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
      SmartDashboard.putNumber("Elevator Setpoint (rotations)", 0);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("elevator/motor 17/position value", m_encoder1.getPosition());
      SmartDashboard.putNumber("elevator/motor 16/position value", m_encoder2.getPosition());
      SmartDashboard.putNumber("elevator/motor 17/output A", elevatorMotor1.getOutputCurrent());
      SmartDashboard.putNumber("elevator/motor 16/output A", elevatorMotor2.getOutputCurrent());
      SmartDashboard.putNumber("elevator/motor 17/output power", elevatorMotor1.getAppliedOutput());
      SmartDashboard.putNumber("elevator/motor 16/output power", elevatorMotor2.getAppliedOutput());
      SmartDashboard.putNumber("elevator/motor 17/raw output power", elevatorMotor1.get());
      SmartDashboard.putNumber("elevator/motor 16/raw output power", elevatorMotor2.get());

      double curSetpoint = SmartDashboard.getNumber("Elevator Setpoint (rotations)", 0);

      moveElevator(curSetpoint);
    }

    public void moveElevator(Double setpoint) {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 1);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
  
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) {
        m_pidController1.setP(p);
        kP = p;
      }
      if((i != kI)) {
        m_pidController1.setI(i);
        kI = i;
      }
      if((d != kD)) {
        m_pidController1.setD(d);
        kD = d;
      }
      if((iz != kIz)) {
        m_pidController1.setIZone(iz);
        kIz = iz;
      }
      if((ff != kFF)) {
        m_pidController1.setFF(ff);
        kFF = ff;
      }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController1.setOutputRange(min, max); 
        kMinOutput = min;
        kMaxOutput = max; 
      }
  
      m_pidController1.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    public void simpleMovement(double input) {
        if (input > 0.25) {
            elevatorMotor1.set(-0.25);
        } else if (input < -0.25) {
            elevatorMotor1.set(0.25);
        } else {
            elevatorMotor1.set(0);
        }
    }

    public void zeroEncoders() {
      m_encoder1.setPosition(0);
      m_encoder2.setPosition(0);
      SmartDashboard.putNumber("Elevator Setpoint (rotations)", 0);
    }
  
}