/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.team696.robot.Constants.OperatorConstants;
import org.team696.robot.subsystems.Limelight;
import org.team696.robot.subsystems.ShooterHood;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final Logger logger = LogManager.getLogger(Robot.class);
  public static RobotContainer m_robotContainer;
  public static OperatorConstants m_robotConstants;
  private Command m_autonomousCommand;
  double last_hood_axis = -69.23; //arbitrary value for initialization later

  // public static double shootAngle;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    logger.info("Starting up...");
    m_robotContainer = new RobotContainer();
    m_robotConstants = new OperatorConstants();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    RobotContainer.limelight.pipeline(1);
    SmartDashboard.putBoolean("isOnTarget", m_robotContainer.turretSubsystem.onTarget());
    SmartDashboard.putNumber("Current RPM", m_robotContainer.shooter.getRPM());
    SmartDashboard.putNumber("Shoot Angle", m_robotContainer.shooterHood.shootAngleToServoAngle(m_robotContainer.shooterHood.servoAngle()));
    SmartDashboard.putNumber("Hood Axis", Math.round(m_robotContainer.operatorPanel.getRawAxis(1)*128));
    SmartDashboard.putNumber("Shoot Angle", m_robotContainer.shooterHood.servoAngleToShootAngle(m_robotContainer.shooterHood.servoAngle()));
    SmartDashboard.putBoolean("Up to Speed", m_robotContainer.shooter.isUpToSpeed());
    SmartDashboard.putNumber("Spindexer Current", m_robotContainer.spindexer.getCurrent());
    SmartDashboard.putNumber("Hood Angle", m_robotContainer.shooterHood.servoAngle());
    SmartDashboard.putNumber("Accelerator Velocity", m_robotContainer.shooter.getAcceleratorVelocity());
    SmartDashboard.putNumber("Right Joystick", m_robotContainer.driverController.getX(Hand.kRight));
    SmartDashboard.putNumber("Left Joystick", m_robotContainer.driverController.getY(Hand.kLeft));
    SmartDashboard.putNumber("Servo Angle ", m_robotContainer.climberServos.getServoAngle());


    // SmartDashboard.putNumber("spin pos", m_robotContainer.spindexer.getEncoderPosition());
    // SmartDashboard.putNumber("servo angle", m_robotContainer.shooterHood.servoAngle());
    // SmartDashboard.putNumber("Number of Balls", m_robotContainer.spindexer.getNumberOfFilledPockets());
    // SmartDashboard.putNumber("distance", Limelight.distanceFromTarget());
    // SmartDashboard.putNumber("shooter power", m_robotContainer.shooter.getShooterPower());
    // SmartDashboard.putNumber("index", Limelight.indexForDistance());
    // SmartDashboard.putNumber("AUTORPM", m_robotContainer.shooter.getAutoRPM());
    // SmartDashboard.putNumber("AUTOANGLE", m_robotContainer.shooter.getAutoAngle());
    // SmartDashboard.putNumber("SERVOAUTOANGLE", m_robotContainer.shooterHood.shootAngleToServoAngle(m_robotContainer.shooter.getAutoAngle()));
    // SmartDashboard.putNumber("AngleDistance", Limelight.angleDistance());
    // SmartDashboard.putNumber("Intake Current", m_robotContainer.intake.intakeCurrent());
    // RobotContainer.limelight.setLights(3);
    // SmartDashboard.putNumber("Roll", m_robotContainer.drivetrain.roll());

  }


  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    logger.info("Transitioning to disabled.");
    m_robotContainer.spindexer.setBrake(false);
    // RobotContainer.limelight.setLights(3);
  }

  @Override
  public void disabledPeriodic() {
    Limelight.setLights(1);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    logger.info("Transitioning to autonomous.");
    m_robotContainer.spindexer.setBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    
  }

  @Override
  public void teleopInit() {
    m_robotContainer.intake.leftIntDepMotor.getEncoder().setPosition(0);
    m_robotContainer.intake.rightIntDepMotor.getEncoder().setPosition(0);
    logger.info("Transitioning to teleop.");
    m_robotContainer.spindexer.setBrake(true);
    // SmartDashboard.putNumber("Target RPM", 3900);
    // SmartDashboard.putNumber("Target Angle", 30);

  }

  /**
   * This function is called periodically during operator control.
   */
  

  @Override
  public void teleopPeriodic() {
        // if(RobotContainer.operatorPanel.getRawButton(OperatorConstants.hoodAutoButton)){
        //   m_robotContainer.shooterHood.moveServoAngle(51);

          

        // }
        // else{
        //   double hood_axis = -Math.round(m_robotContainer.operatorPanel.getRawAxis(1)*128*1.5);
        //   if (last_hood_axis == -69.23){
        //     last_hood_axis = hood_axis;
        //   }
        //   double delta = hood_axis - last_hood_axis;
        
        //   SmartDashboard.putNumber("Hood Axis", Math.round(m_robotContainer.operatorPanel.getRawAxis(1)*128));

        //   if (delta + m_robotContainer.shooterHood.servoAngle() <= 51 && delta + m_robotContainer.shooterHood.servoAngle() >= 10){
        //     m_robotContainer.shooterHood.moveServoAngle(m_robotContainer.shooterHood.servoAngle()+delta);
          
           
        //   }
        //   last_hood_axis = hood_axis;


        // }
      // }
        

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
