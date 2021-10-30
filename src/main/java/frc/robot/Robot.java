// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /** You can leave this in if you want, it only changes anything if you are using the autonomous part of the code, 
   * if you do remove it, just delete any errors it creates */
  private Command m_autonomousCommand;

  /**Creates an object for our DriveTrain class so we can call things from the class
   * make sure to include any variables needed for the parameters
   */
  public static DriveTrain m_driveTrain = new DriveTrain(Constants.leftFrontPort, Constants.leftBackPort, Constants.rightFrontPort, Constants.rightBackPort);
  /** Similar to the previous line, creates a RobotContainer object so we can call things from the robotcontainer class, 
   * what you name this is up to you  */
  private RobotContainer m_robotContainer;


  /** The values that we will use from the joystick */
  double xAxis;
  double yAxis;
  /** Values that will go in the parameters that are needed for the RobotDrive method */
  double leftSpeed;
  double rightSpeed;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
  public void autonomousInit() {
  
  }

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
  public void teleopPeriodic() {
    /** Defining the XAxis and yAxis objects as the x and y values from our joysticks on the controller */
    xAxis =   -m_robotContainer.xboxController.getRawAxis(1);
    yAxis = m_robotContainer.xboxController.getRawAxis(4);
    /** The math required to make the robot turn left and right properly according to the joystick configuration. */
    leftSpeed =   yAxis + xAxis;
    rightSpeed = yAxis - xAxis;
    /** Calling back the robotDrive method from the DriveTrain class so that it runs fifty times a second
     * uses the parameters leftSpeed and rightSpeed that we just defined
     */
    m_driveTrain.robotDrive(leftSpeed, rightSpeed);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
