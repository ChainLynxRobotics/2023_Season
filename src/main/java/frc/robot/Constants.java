package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {

        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds

        public static final double DRIVE_DEADBAND = 0.06;
        //4.45 m/s max speed
        public static final double kMaxSpeedBase = 4.8;
        public static final double kMaxSpeedScaleFactor = 0.9;
        public static final double kMaxSpeedMetersPerSecond =
            kMaxSpeedBase * kMaxSpeedScaleFactor;

        public static final double kMaxAngularSpeedBase = Math.PI;
        public static final double kMaxAngularSpeedScaleFactor = 0.7;
        public static final double kMaxAngularSpeed =
            kMaxAngularSpeedBase * kMaxAngularSpeedScaleFactor; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        //scaling factor for the alternative turning mode
        public static final int altTurnSmoothing = 20;

        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset =
            -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 6;
        public static final int kRearLeftDrivingCanId = 4;
        public static final int kFrontRightDrivingCanId = 8;
        public static final int kRearRightDrivingCanId = 2;

        public static final int kFrontLeftTurningCanId = 5;
        public static final int kRearLeftTurningCanId = 3;
        public static final int kFrontRightTurningCanId = 7;
        public static final int kRearRightTurningCanId = 1;

        public static final boolean kGyroReversed = false;

        public static final int gyroID = 15;
    }

    public static final class ModuleConstants {

        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps =
            NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters =
            kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction =
            (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps =
            (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) /
            kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor =
            (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor =
            ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor =
            (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor =
            (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput =
            kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {

        public static final int kDriverControllerPort = 0;
        public static final int kIOperatorControllerPort = 1;
        public static final int yAxis = 1;
        public static final double kDriveDeadband = 0.06;
        public static final double kMagnitudeDeadband = 0.06;
        public static final double kDirectionSlewRate = 10; // radians per second
        public static final double kMagnitudeSlewRate = 90; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 90; // percent per second (1 = 100%)

        //button bindings
        public static final int INTAKE_SPEED_AXIS = 3;
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        public static final double kIThetaController = 0.001;
        public static final double kDThetaController = 0.7;

        public static final double MIN_PITCH_CHANGE = 5;

        public static final double kP = 1;
        public static final double kI = 0.001;
        public static final double kD = 0.1;

        public static final double FIELD_LENGTH = 16.54;
        public static final double FIELD_WIDTH = 8.02;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularSpeedRadiansPerSecondSquared
        );
    }

    //TO DO: check these are the correct values (meters)
    public static final double CAMERA_HEIGHT = 0.15;
    public static final double TARGET_HEIGHT = 0.48;
    public static final double CAMERA_PITCH_RADIANS = 0;
    public static final double GOAL_RANGE = 1;

    public static final class NeoMotorConstants {

        public static final double kFreeSpeedRpm = 5676;
    }

    public class ElevatorConstants {
      public static final double groundPickupCubeHybrid = 7.5;
      public static final double coneDrivingWithLift = 3;
      public static final double midElevatorGamepiece = 7.5;
      public static final double highElevatorConeSetpoint = 15.5;
      public static final double highElevatorCubeSetpoint = 13.5;
      public static final double doubleSubstationSetpoint = 14.2;
      public static final double fullRetractionSetpoint = 0;

      public static final double ELEVATOR_RAMP_DIST = 3;
      public static final double MAX_TRAVEL_LIMIT = 8; //rotations of hall effect sensors

      public static final int ELEVATOR_MOTOR_ID_MASTER = 17;
      public static final int ELEVATOR_MOTOR_ID_SLAVE = 16;

      public static final double ELEVATOR_kS = 0.2;
      public static final double ELEVATOR_kA = 0.09;
      public static final double ELEVATOR_kV = 6.14;
      public static final double ELEVATOR_kG = 0.68;
    }

    public static final int INTAKE_MOTOR_INNER = 12;
    public static final int INTAKE_MOTOR_OUTER = 13;
    
    //solenoid ports on PCM
    public static final int SOLENOID_forward1 = 0;
    public static final int SOLENOID_reverse1 = 1;

    public enum GamePiece {
        CUBE,
        CONE,
    }

    public enum ScoringLocation {
        LOW,
        MID,
        MIDHIGH,
        HIGH,
        SUBSTATION,
    }

    public class Bindings {
      public static final int releaseGamePiece = 1;
      public static final int intakeGamePiece = 2;
      public static final int setGamePieceCube = 3;
      public static final int setGamePieceCone = 4;
      public static final int raiseArm = 9;
      public static final int lowerArm = 8;
      public static final int manualElevatorControl = 5;
      public static final int chargeStationBalance = 10;

      //measured in hall effect sensor rotations
      public static final int groundPickUp = 15; //for ground pick up and cube hybrid
      public static final int fullRetraction = 14;
      public static final int lowScoreElevatorSetpoint = 13; //for cone pickup with ability to lift
      public static final int midScoreElevatorSetpoint = 12;
      public static final int highScoreElevatorSetpoint = 11;
      public static final int doubleSubstationSetpoint = 16;
    }
}
