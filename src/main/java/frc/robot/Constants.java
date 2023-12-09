package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;

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
    public static final double GLOBAL_TIMESTEP = 0.02;
    public static final class OIConstants {
        public static final int kIOperatorControllerPort = 1;
        public static final int yAxis = 1;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
        public static final double kMaxAccel = 50;
        public static final double countsPerRev = 42;
        public static final int currentLimit = 25; //amps
    }

    public class ElevatorConstants {
      public static final double groundPickupCubeHybrid = 7.5;
      public static final double coneDrivingWithLift = 3;
      public static final double midElevatorGamepiece = 7.5;
      public static final double highElevatorConeSetpoint = 15.5;
      public static final double highElevatorCubeSetpoint = 14.5;
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

    //map buttons on operator joystick to elevator extension distances
    public static class Pairings {
        public static final Map<Integer, Double> bindingsToSetpoints = new HashMap<>()
            {{ put(Bindings.groundPickUp, ElevatorConstants.groundPickupCubeHybrid); }
            { put(Bindings.lowScoreElevatorSetpoint, ElevatorConstants.coneDrivingWithLift); }
            { put(Bindings.midScoreElevatorSetpoint, ElevatorConstants.midElevatorGamepiece); }
            { put(Bindings.doubleSubstationSetpoint, ElevatorConstants.doubleSubstationSetpoint); }
            { put(Bindings.fullRetraction, ElevatorConstants.fullRetractionSetpoint); }};

    }

    public enum CONTROL_TYPES {
        POSITION,
        TRAPEZOIDAL,
        SCURVE
    }

    public final static class EventMapper {
        public static EventMapper instance;
        private static HashMap<String, Command> eventMap;

        public static synchronized EventMapper getInstance() {
            if (instance == null) {
                instance = new EventMapper();
                eventMap = new HashMap<>();
            }
            return instance;
        }

        public static HashMap<String, Command> getEventMap() {
            return eventMap;
        }
    }
}
