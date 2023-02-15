package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** A robot elevator subsystem that moves with a motion profile. */
public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorMotor1;
  private CANSparkMax elevatorMotor2;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder; //encoder class object
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput; //pid values, CONSTANT

  public ElevatorSubsystem() {
    //these two motors are symetric and should be CONTROLLED TOGETHER! 
    //pid uses floating point numbers but they should converge and stay near together.
    //If this changes, a pool pid should be implemented to make sure the motors are on the same page
    elevatorMotor1 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless); //RAI elevator motor object
    elevatorMotor2 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless); //RAI elevator motor object

    m1_encoder = elevatorMotor1.getEncoder();
    m2_encoder = elevatorMotor2.getEncoder();
    elevatorMotor1.setIdleMode(IdleMode.kBrake);
    elevatorMotor2.restoreFactoryDefaults();
    m_pidController1 = elevatorMotor1.getPIDController();
    m_pidController2 = elevatorMotor2.getPIDController();
    
    /*
    PID Elevator Coefficients, CONSTANT
    to be tuned more
    current Elevator Coefficients iteration number:
    1
    increment iteration number whenever you change, leave in commits
    */
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0;
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController1.setP(kP);
    m_pidController2.setP(kP);
    m_pidController1.setI(kI);
    m_pidController2.setI(kI);
    m_pidController1.setD(kD);
    m_pidController2.setD(kD);
    m_pidController1.setIZone(kIz);
    m_pidController2.setIZone(kIz);
    m_pidController1.setFF(kFF);
    m_pidController2.setFF(kFF);
    m_pidController1.setOutputRange(kMinOutput, kMaxOutput);
    m_pidController2.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }


 
  public void moveElevator(DoubleSupplier joystickPos) {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = MathUtil.clamp(joystickPos.getAsDouble(), -0.1, 0.1);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }

  public CANSparkMax getElevatorMotor() {
    return elevatorMotor;
  }

}