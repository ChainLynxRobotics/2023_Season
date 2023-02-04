package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
   public static final int MOTOR_LEFT_MASTER_ID = 1; 
   public static final int MOTOR_LEFT_SLAVE_ID = 2;
   public static final int MOTOR_RIGHT_MASTER_ID = 3;
   public static final int MOTOR_RIGHT_SLAVE_ID = 4;

   public static final int ARM_MOTOR_ID = 5;
   public static final int ELEVATOR_MOTOR_1_ID = 6; //elevator has two symetric motors for power and balance
   public static final int ELEVATOR_MOTOR_2_ID = 7;

   public static final int JOYSTICK_PORT1 = 0;
   public static final int JOYSTICK_PORT2 = 1;

   public static final int ANALOG_GYRO_PORT = 0;

   public static final double DEFAULT_DEADBAND = 0.02;
   public static final double WHEEL_RADIUS = Units.inchesToMeters(3); 
   public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;

   public static final double TRACK_WIDTH = Units.inchesToMeters(33); 
   public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

   public static final double MAX_VELOCITY = 4.57; //m/s estimate
   public static final double MAX_ACCEL = 1.95; //m/s^2 estimate
    
   //auto feedforwards (estimated; must do accurate characterization eventually)
   public static final double kS = 0.38; //volts
   public static final double kV = 2.1; //volts*sec/meter
   public static final double kA = 0.25; //volt*sec^2/meter
   
   //auto feedback constants
   public static final double kP = 0.2;
   public static final double kI = 0.01;
   public static final double kD = 0;

   public enum JoystickScaling{
      QUADRATIC,
      CUBIC,
      LINEAR,
      SQRT,
      CBRT,
      LOGARITHMIC
   }
     
   public enum DriveStyle{
      CUSTOM_TANK,
      NORMAL_ARCADE,
      SATURATED_ARCADE,
      ARCADE_TANK
   }
}
