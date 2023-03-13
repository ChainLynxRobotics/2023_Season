
package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

//TO DO: error handling for if pid controllers get misaligned
public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax elevatorMotor1;
    private final CANSparkMax elevatorMotor2;
    public static boolean resetTest = false;
    public static boolean atSPZero= false;
    public static boolean atSPOne = false;
    public static boolean atSPTwo = false;
    public static boolean atSPThree = false;
    public static boolean atSPFour = false;

    private final SparkMaxPIDController m_pidController1;
    public final RelativeEncoder m_encoder1;
    private final RelativeEncoder m_encoder2;
    public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, elevatorSpeed, elevatorExtensionPercent, avg_encoder,neutralElevator;

    private List<CANSparkMax> motorList = new ArrayList<>();
  
    public ElevatorSubsystem() {
      elevatorMotor1 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID_MASTER, MotorType.kBrushless);
      elevatorMotor2 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID_SLAVE, MotorType.kBrushless);
      m_encoder1 = elevatorMotor1.getEncoder();
      m_encoder2 = elevatorMotor2.getEncoder();
      
      avg_encoder = (m_encoder1.getPosition());//+m_encoder2.getPosition())/2; encoder 2 overturning
      elevatorExtensionPercent = avg_encoder/16.5; //20 = max elevator motor revs

      elevatorMotor1.setIdleMode(IdleMode.kBrake);
      elevatorMotor1.restoreFactoryDefaults();
      elevatorMotor1.setInverted(true);
      elevatorMotor2.follow(elevatorMotor1, true);
      motorList.add(elevatorMotor1);
      motorList.add(elevatorMotor2);

      m_pidController1 = elevatorMotor1.getPIDController();

      // PID coefficients
      kP = 0.1; 
      kI = 1e-4;
      kD = 1; 
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
      SmartDashboard.putNumber("Set Rotations", 0);
      SmartDashboard.putNumber("elevator setpoint", 0);
      //SmartDashboard.putNumber("elevator motor power", elevatorSpeed);
    }

    // resetTest = true;
    //   SmartDashboard.putBoolean("Elevator/EncoderReset", resetTest);
    public void moveElevator(Double setpoint) {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 1);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
      double spt = SmartDashboard.getNumber("elevator setpoint", 0);
  
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { m_pidController1.setP(p); kP = p; }
      if((i != kI)) { m_pidController1.setI(i); kI = i; }
      if((d != kD)) { m_pidController1.setD(d); kD = d; }
      if((iz != kIz)) { m_pidController1.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { m_pidController1.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController1.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
  
      m_pidController1.setReference(spt, CANSparkMax.ControlType.kPosition);
      
      
      SmartDashboard.putNumber("ProcessVariable1", m_encoder1.getPosition());
      SmartDashboard.putNumber("ProcessVariable2", m_encoder2.getPosition());
    }
    public void updateSetPointsShuffle(){
      SmartDashboard.putBoolean("Elevator/Setpoints/Zero", atSPZero);
      SmartDashboard.putBoolean("Elevator/Setpoints/One", atSPOne);
      SmartDashboard.putBoolean("Elevator/Setpoints/Two", atSPTwo);
      SmartDashboard.putBoolean("Elevator/Setpoints/Three", atSPThree);
      SmartDashboard.putBoolean("Elevator/Setpoints/Four", atSPFour);
    }
    public void moveToSetPoint0(){
      atSPOne = false;
      atSPZero = true;
      atSPTwo = false;
      atSPThree = false;
      atSPFour= false;
      updateSetPointsShuffle();

    }
    public void moveToSetPoint1(){
      atSPOne = true;
      atSPZero = false;
      atSPTwo = false;
      atSPThree = false;
      atSPFour= false;
      updateSetPointsShuffle();

    }
    public void moveToSetPoint2(){
      atSPOne = false;
      atSPZero = false;
      atSPTwo = true;
      atSPThree = false;
      atSPFour= false;
      updateSetPointsShuffle();
    }
    public void moveToSetPoint3(){
      atSPOne = false;
      atSPZero = false;
      atSPTwo = false;
      atSPThree = true;
      atSPFour= false;
      updateSetPointsShuffle();

    }
    public void moveToSetPoint4(){
      atSPOne = false;
      atSPZero = false;
      atSPTwo = false;
      atSPThree = false;
      atSPFour= true;
      updateSetPointsShuffle();

    }
    public void simpleMovement(double input) {
      
      elevatorSpeed = -0.45*input;
      if (input > 0.1) {//deadband +y
        elevatorMotor1.set(elevatorSpeed);
        resetTest = false;
      } else if (input < -0.1) {//deadband -y
        elevatorMotor1.set(elevatorSpeed);
        resetTest = false;
      } else {
        neutralElevator=(ArmSubsystem.extendedState== true) ? 0.00 : 0.0;//keeping possibilty open to change neutral speed based on fourbar state, not viable unless we know pressure 
        elevatorMotor1.set(neutralElevator);
      }
      SmartDashboard.putNumber("Joystick/yInput", input);
      SmartDashboard.putNumber("Elevator/Elevator Speed", elevatorSpeed*100);
      SmartDashboard.putNumber("Elevator/Encoder 1", m_encoder1.getPosition());//using to measure encoder 1 values
      SmartDashboard.putNumber("Elevator/Encoder 2", m_encoder2.getPosition());//using to measure encoder 2 value
      SmartDashboard.putNumber("Elevator/Extension%", elevatorExtensionPercent);//using to try and calculte encoder extension, encodersrent resseting so doesnt wrok.
      SmartDashboard.putBoolean("Elevator/EncoderReset", resetTest);
    }
  
}