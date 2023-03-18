// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {

    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset
    );

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset
    );

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset
    );

    // The gyro sensor
    public WPI_Pigeon2 m_gyro = new WPI_Pigeon2(
        Constants.DriveConstants.gyroID
    );
    private final PowerDistribution powerDistribution = new PowerDistribution();

    private double prevAngle = 0;
    private double rightAngGoal = 0;
    private double turnDir = 0;

    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;

    private SlewRateLimiter magLimiter = new SlewRateLimiter(
        OIConstants.kMagnitudeSlewRate
    );
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(
        OIConstants.kRotationalSlewRate
    );
    double prevTime = WPIUtilJNI.now() * 1e-6;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
        }
    );

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        powerDistribution.clearStickyFaults();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
            Rotation2d.fromDegrees(-m_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition(),
            }
        );

        double ang = m_gyro.getRotation2d().getDegrees();

        SmartDashboard.putNumber("delta heading", ang - prevAngle);

        prevAngle = ang;
        SmartDashboard.putNumber("heading", ang);

        SmartDashboard.putNumber("right stick angle", rightAngGoal);
        SmartDashboard.putNumber("turn direction", turnDir);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(-m_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition(),
            },
            pose
        );
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param xRot          The x-position of the right joystick, used as rotational speed in normal turning
     * @param yRot          the y-position of the right joystick, used for getting the direction to point to in alt-turning
     * @param altDrive      whether to use alternative driving
     * @param centerGyro    whether to center the gyro to the direction the robot is currently facing
     */

    public void drive(
        double xSpeed,
        double ySpeed,
        double xRot,
        double yRot,
        boolean altDrive,
        boolean centerGyro
    ) {
        if (centerGyro) zeroHeading();
        if (altDrive) {
            altDrive(xSpeed, ySpeed, xRot, yRot);
        } else {
            mainDrive(xSpeed, ySpeed, xRot);
        }
    }

    public void mainDrive(double xSpeed, double ySpeed, double xRot) {
        double rot = xRot * DriveConstants.kMaxAngularSpeed;
        move(xSpeed, ySpeed, rot);
    }

    public void altDrive(
        double xSpeed,
        double ySpeed,
        double xRot,
        double yRot
    ) {
        double rot = 0;
        //convert to degrees
        rightAngGoal = Math.atan2(xRot, yRot) * 180 / Math.PI;
        if (xRot != 0 || yRot != 0) {
            //convert to degrees
            double stickAng = Math.atan2(xRot, yRot) * 180 / Math.PI;
            //gets the difference in angle, then uses mod to make sure its from -180 to 180
            rot =
                Math.tanh(
                    ((m_gyro.getAngle() + stickAng + 180) % 360 - 180) /
                    DriveConstants.altTurnSmoothing
                ) *
                DriveConstants.kMaxAngularSpeed;
        }
        turnDir = rot;
        move(xSpeed, ySpeed, rot);
    }

    private void move(double xSpeed, double ySpeed, double rot) {
        move(xSpeed, ySpeed, rot, true);
    }

    private void move(
        double xSpeed,
        double ySpeed,
        double rot,
        boolean rateLimit
    ) {
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(
                Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)
            );

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (currentTranslationMag != 0.0) {
                directionSlewRate =
                    Math.abs(
                        OIConstants.kDirectionSlewRate / currentTranslationMag
                    );
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - prevTime;
            double angleDif = SwerveUtils.AngleDifference(
                inputTranslationDir,
                currentTranslationDir
            );
            if (angleDif < 0.45 * Math.PI) {
                currentTranslationDir =
                    SwerveUtils.StepTowardsCircular(
                        currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime
                    );
                currentTranslationMag =
                    magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    currentTranslationMag = magLimiter.calculate(0.0);
                } else {
                    currentTranslationDir =
                        SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
                    currentTranslationMag =
                        magLimiter.calculate(inputTranslationMag);
                }
            } else {
                currentTranslationDir =
                    SwerveUtils.StepTowardsCircular(
                        currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime
                    );
                currentTranslationMag = magLimiter.calculate(0.0);
            }
            prevTime = currentTime;

            xSpeedCommanded =
                currentTranslationMag * Math.cos(currentTranslationDir);
            ySpeedCommanded =
                currentTranslationMag * Math.sin(currentTranslationDir);
            currentRotation = rotLimiter.calculate(rot);
        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            currentRotation = rot;
        }

        // Adjust input based on max speed
        double xSpeedDelivered =
            xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered =
            ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle())
            )
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates,
            DriveConstants.kMaxSpeedMetersPerSecond
        );
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        );
        m_frontRight.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        );
        m_rearLeft.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        );
        m_rearRight.setDesiredState(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        );
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            DriveConstants.kMaxSpeedMetersPerSecond
        );
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public void getModuleStates() {
        System.out.println(m_frontLeft.getState());
        System.out.println(m_frontRight.getState());
        System.out.println(m_rearLeft.getState());
        System.out.println(m_rearRight.getState());
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
    }

    public double getPitch() {
        return m_gyro.getPitch();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        //builder.addDoubleProperty("Magnitude Commanded",  () -> magCommanded, null);
        //builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
        //addChild("Controller", m_controller);
    }
}
