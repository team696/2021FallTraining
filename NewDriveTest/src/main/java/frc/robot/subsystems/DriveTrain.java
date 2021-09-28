// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** YOU DO NOT HAVE TO WORRY ABOUT THIS SECTION, USING TAB WILL ADD ANYTHING NEEDED HERE AUTOMATICALLY */
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  /** Creates a differential drive which allows the two speed controller groups to work together */
  DifferentialDrive diffDrive;
  /**Creates a speed controller group which allows you to run multiple motors with one object */
  SpeedControllerGroup leftSide;
  SpeedControllerGroup rightSide;
  /**Creates the objects for each motor controller */
  CANSparkMax leftFront;
  CANSparkMax leftBack;
  CANSparkMax rightFront;
  CANSparkMax rightBack;

  
  public DriveTrain(int leftFrontPort, int leftBackPort, int rightFrontPort, int rightBackPort) {
    /** Defining all the motor controllers with the CANSparkMax(port, motor type) method */
    leftFront = new CANSparkMax(leftFrontPort, MotorType.kBrushless);
    leftBack = new CANSparkMax(leftBackPort, MotorType.kBrushless);
    rightBack = new CANSparkMax(rightBackPort, MotorType.kBrushless);
    rightFront = new CANSparkMax(rightFrontPort, MotorType.kBrushless);
    /**Defining the two speed controller groups using the SpeedControllerGroup(motor, motor) method */
    leftSide = new SpeedControllerGroup(leftFront, leftBack);
    rightSide = new SpeedControllerGroup(rightFront, rightBack);

    /**Defining the differential drive object using the DifferentialDrive(speedcontrollergroup, speedcontrollergroup) method */
    diffDrive = new DifferentialDrive(leftSide, rightSide);
  }

  /**Method we will use to actually control the robot, which need the parameters leftSpeed and rightSpeed, which will be defined in robot */
  public void robotDrive(double leftSpeed, double rightSpeed){
    /** A method tied to the DifferentialDrive object in the Wpilib libraries, basically lets us control the motors in a tank drive fashion  */
    diffDrive.tankDrive(leftSpeed, rightSpeed);
    /** Creates a minimum required output for the joystick to run the motor */
    diffDrive.setDeadband(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (50 times a second)
  }
}
