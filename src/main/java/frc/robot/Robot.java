// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.VisionWindow;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

	private NetworkTableInstance inst;
	private NetworkTable USB2;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
		// register subsystems into a list to minimize redundancy if logging
		// inputs/outputs for all
		inst = NetworkTableInstance.create();

		// start a NT4 client
		inst.startClient4(Constants.NETWORK_TABLE_CLIENT);

		// connect to a roboRIO with team number TEAM
		inst.setServerTeam(Constants.TEAM_NUMBER);

		// connect to a specific host/port
		inst.setServer(Constants.NETWORK_TABLES_SERVER, NetworkTableInstance.kDefaultPort4);
		USB2 = inst.getTable(Constants.PhotonVisionConstants.PHOTON_NETWORK_TABLES_NAME).getSubTable(Constants.PhotonVisionConstants.PHOTON_CAMERA_NAME);
		//
    if (Constants.PhotonVisionConstants.OPEN_TAG_WINDOW) {
			VisionWindow.startWindow(this);
		}
  }

	public double[] getPose() {
    //checks if there is a current target
		if (USB2.getEntry("hasTarget").getBoolean(false)) {
      //returns the pose of the current target
			return USB2.getEntry("targetPose").getDoubleArray(Constants.PhotonVisionConstants.DEFAULT_POSE);
		}
    //there is either no detected tags or photonvision is not connected
    return Constants.PhotonVisionConstants.DEFAULT_POSE;
	}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}
}
