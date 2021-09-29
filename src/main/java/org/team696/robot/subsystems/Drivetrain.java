package org.team696.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

// import com.analog.adis16448.frc.ADIS16448_IMU;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import org.team696.MathUtils.MathUtils;
import org.team696.robot.Constants;
import org.team696.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  private static final Logger logger = LogManager.getLogger(Drivetrain.class);
  private static int iterationCounter = 0;
  private CANSparkMax leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive;
  private CANEncoder leftFrontEncoder, leftRearEncoder, rightFrontEncoder, rightRearEncoder;
  private CANPIDController leftPID, rightPID;
  // private ADIS16448_IMU imu;
  private AHRS navX;
  private double targetYaw; //In degrees
  private PIDController driveAssistPID = new PIDController(DrivetrainConstants.yawkP, DrivetrainConstants.yawkI, DrivetrainConstants.yawkD);

  private DrivetrainMode mode = DrivetrainMode.OpenLoop;

  public enum DrivetrainMode{
    None,
    OpenLoop,
    RateCommand
  }
  
  public Drivetrain() {
    super();
    logger.info("Instantiating drivetrain...");
    //Instantiate Sparks
    leftFrontDrive = new CANSparkMax(DrivetrainConstants.leftFrontCANID, MotorType.kBrushless);
    leftRearDrive = new CANSparkMax(DrivetrainConstants.leftRearCANID, MotorType.kBrushless);
    rightFrontDrive = new CANSparkMax(DrivetrainConstants.rightFrontCANID, MotorType.kBrushless);
    rightRearDrive = new CANSparkMax(DrivetrainConstants.rightRearCANID, MotorType.kBrushless);
    
    //Configure Sparks
    //TODO: get mode from OI on init?
    configLFSpark(mode);
    configLRSpark(mode);
    configRFSpark(mode); 
    configRRSpark(mode);

    //imu = new ADIS16448_IMU();
    navX = new AHRS();
    //TODO: check actual gyro output range
    driveAssistPID.enableContinuousInput(-180, 180);
    logger.info("Drivetrain instantiation complete.");
  }

  /**Set the drivetrain mode.
   * @param mode the mode to set
   */
  public void setMode(DrivetrainMode mode) {
    if(this.mode == mode){
      return;
    }
    logger.info(String.format("Setting drivetrain mode to %s", mode.name()));
    this.mode = mode;
    if(mode == DrivetrainMode.RateCommand){
      targetYaw = getRobotYaw();
    }
    configLFSpark(mode);
    configLRSpark(mode);
    configRFSpark(mode);
    configRRSpark(mode);
  }

  public void testMotor(double power){
    leftRearDrive.set(power);
  }
  
  public void resetEncoders(){
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    //If any sparks have reset, reinitialize them
    if(leftFrontDrive.getFault(FaultID.kHasReset)){
      logger.warn("Left front drive Spark has reset. Reinitializing.");
      configLFSpark(mode);
    }
    if(leftRearDrive.getFault(FaultID.kHasReset)){
      logger.warn("Left rear drive Spark has reset. Reinitializing.");
      configLRSpark(mode);
    }
    if(rightFrontDrive.getFault(FaultID.kHasReset)){
      logger.warn("Right front drive Spark has reset. Reinitializing.");
      configRFSpark(mode);
    }
    if(rightRearDrive.getFault(FaultID.kHasReset)){
      logger.warn("Right rear drive Spark has reset. Reinitializing.");
      configRRSpark(mode);
    }

    SmartDashboard.putNumber("Robot Speed", getRobotSpeed());
    SmartDashboard.putNumber("Current Yaw", getRobotYaw());
    SmartDashboard.putString("Drive Mode", mode.name());

    if(iterationCounter > DrivetrainConstants.slowTasksSpeed){
      iterationCounter = 0;

      //Perform slow tasks
      // logger.info(String.format("Left front drive motor temp: %f", leftFrontDrive.getMotorTemperature()));
      // logger.info(String.format("Left rear drive motor temp: %f", leftRearDrive.getMotorTemperature()));
      // logger.info(String.format("Right front drive motor temp: %f", rightFrontDrive.getMotorTemperature()));
      // logger.info(String.format("Right rear drive motor temp: %f", rightRearDrive.getMotorTemperature()));

    } else {
      iterationCounter++;
    }

  }

  /**
   * Drives robot using current mode.
   * @param speed Robot speed (-1, 1)
   * @param turn Turning-ness, (-1, 1), positive values are counterclockwise
   */
  public void arcadeDrive(double speed, double turn){
    switch(mode){
      case OpenLoop:
        driveOpenLoop(speed, turn);
        break;
      case RateCommand:
        driveClosedLoop(speed, turn);
        break;
      default:
        return;
    }
  }

  public double roll(){
    return navX.getPitch();
  }

  /**
   * Drives robot in open loop percent-output.
   * @param speed Robot speed (-1, 1)
   * @param turn Turning-ness, (-1, 1), positive values are counterclockwise
   */
  public void driveOpenLoop(double speed, double turn){
    double leftSpeed, rightSpeed;
    if(turn > 0){
      turn = turn*turn;
    } else{
      turn = -(turn*turn);
    }
    if(speed > 0){
      speed = speed*speed;
    } else{
      speed = -(speed*speed);
    }
    rightSpeed = speed + turn;
    leftSpeed = speed - turn;

    //Scale speeds to stay within range
    if(Math.abs(rightSpeed) > 1){
      double multiplier = (1 / Math.abs(rightSpeed));
      rightSpeed *= multiplier;
      leftSpeed *= multiplier;
    }
    if(Math.abs(leftSpeed) > 1){
      double multiplier = (1 / Math.abs(leftSpeed));
      rightSpeed *= multiplier;
      leftSpeed *= multiplier;
    }

    SmartDashboard.putNumber("Left Commanded Throttle", leftSpeed);
    SmartDashboard.putNumber("Right Commanded Throttle", rightSpeed);
    leftFrontDrive.set(leftSpeed);
    rightFrontDrive.set(rightSpeed);
  }

  /**
   * Implements closed-loop assisted driving.
   * 
   * @param speed Robot speed (-1, 1)
   * @param turn Turning-ness, (-1, 1), positive values are counterclockwise
   */
  public void driveClosedLoop(double speed, double turn){
    speed *= DrivetrainConstants.maxSpeed;
    turn *= DrivetrainConstants.maxTurnRate;

    targetYaw += turn;
    targetYaw = MathUtils.wrapAngle180(targetYaw);
    SmartDashboard.putNumber("Target Yaw", targetYaw);
    SmartDashboard.putNumber("Target Speed", speed);

    double currentYaw = getRobotYaw();
    double turnOutput = driveAssistPID.calculate(currentYaw, targetYaw);
    SmartDashboard.putNumber("Turn PID Output", turnOutput);
    double rightSpeed = speed + turnOutput; //In RPM
    double leftSpeed = speed - turnOutput; //In RPM

    //Scale speeds to stay within range
    if(Math.abs(rightSpeed) > DrivetrainConstants.maxSpeed){
      double multiplier = (DrivetrainConstants.maxSpeed / rightSpeed);
      rightSpeed *= multiplier;
      leftSpeed *= multiplier;
    }
    if(Math.abs(leftSpeed) > DrivetrainConstants.maxSpeed){
      double multiplier = (DrivetrainConstants.maxSpeed / leftSpeed);
      rightSpeed *= multiplier;
      leftSpeed *= multiplier;
    }

    SmartDashboard.putNumber("Left Commanded Speed", leftSpeed);
    SmartDashboard.putNumber("Right Commanded Speed", rightSpeed);
    leftPID.setReference(leftSpeed, ControlType.kVelocity);
    rightPID.setReference(rightSpeed, ControlType.kVelocity);
  }

  public void setTargetYaw(double yaw){
    targetYaw = yaw;
  }

  public double getRobotYaw(){
    //return MathUtils.wrapAngle180(imu.getAngle());
    return 0.0;
  }

  // public void calibrateIMU(){
  //   imu.configCalTime(DrivetrainConstants.IMUCalTime);
  //   imu.calibrate();
  // }

  /**
   * Gets the speed of a side of the robot.
   * @param isRight Set to false for the left side, true for the right side
   * @return Speed of the side of the robot in in/s
   */
  @SuppressWarnings("checkstyle:MagicNumber")
  public double getSpeed(boolean isRight){
    double frontSpeed, rearSpeed;

    if(isRight){
      frontSpeed = rightFrontEncoder.getVelocity(); //in RPM
      rearSpeed = rightRearEncoder.getVelocity(); //in RPM
    } else{
      frontSpeed = leftFrontEncoder.getVelocity(); //in RPM
      rearSpeed = leftRearEncoder.getVelocity(); //in RPM
    }

    //TODO: check to make sure front/rear speeds are similar
    
    double motorSpeedRPM = (frontSpeed + rearSpeed)/2.;
    double outputSpeedRPM = motorSpeedRPM / DrivetrainConstants.driveGearboxReduction; //Speed (RPM) of gearbox output shaft
    return outputSpeedRPM * (1./60.) * Math.PI * DrivetrainConstants.wheelDiameter; //Convert shaft RPM to in/s
  }

  /**
   * Gets robot center speed.
   * @return Robot speed in in/s
   */
  public double getRobotSpeed(){
    double leftSpeed = getSpeed(false);
    double rightSpeed = getSpeed(true);
    return (leftSpeed + rightSpeed)/2.;
  }

  private void configLFSpark(DrivetrainMode mode){
    logger.debug(String.format("Configuring left-front Spark for %s", mode.name()));
    leftFrontDrive.restoreFactoryDefaults();
    leftFrontDrive.setInverted(DrivetrainConstants.leftFrontIsInverted);
    leftFrontDrive.setSmartCurrentLimit(Constants.DrivetrainConstants.stallCurrentLimit, 
                                        Constants.DrivetrainConstants.freeCurrentLimit, 
                                        Constants.DrivetrainConstants.stallThresholdRPM);
    leftFrontDrive.setOpenLoopRampRate(Constants.DrivetrainConstants.openLoopRampRate);
    leftFrontEncoder = leftFrontDrive.getEncoder();
    leftPID = leftFrontDrive.getPIDController();
  }

  private void configLRSpark(DrivetrainMode mode){
    logger.debug(String.format("Configuring left-rear Spark for %s", mode.name()));
    leftRearDrive.restoreFactoryDefaults();
    leftRearDrive.setInverted(DrivetrainConstants.leftRearIsInverted);
    leftRearDrive.setSmartCurrentLimit(Constants.DrivetrainConstants.stallCurrentLimit, 
                                        Constants.DrivetrainConstants.freeCurrentLimit, 
                                        Constants.DrivetrainConstants.stallThresholdRPM);
    leftRearEncoder = leftRearDrive.getEncoder();
    leftRearDrive.follow(leftFrontDrive);
  }

  private void configRFSpark(DrivetrainMode mode){
    logger.debug(String.format("Configuring right-front Spark for %s", mode.name()));
    rightFrontDrive.restoreFactoryDefaults();
    rightFrontDrive.setInverted(DrivetrainConstants.rightFrontIsInverted);
    rightFrontDrive.setSmartCurrentLimit(Constants.DrivetrainConstants.stallCurrentLimit, 
                                        Constants.DrivetrainConstants.freeCurrentLimit, 
                                        Constants.DrivetrainConstants.stallThresholdRPM);
    rightFrontDrive.setOpenLoopRampRate(Constants.DrivetrainConstants.openLoopRampRate);
    rightFrontEncoder = rightFrontDrive.getEncoder();
    rightPID = rightFrontDrive.getPIDController();
  }
  private void configRRSpark(DrivetrainMode mode){
    logger.debug(String.format("Configuring right-rear Spark for %s", mode.name()));
    rightRearDrive.restoreFactoryDefaults();
    rightRearDrive.setInverted(DrivetrainConstants.rightRearIsInverted);
    rightRearDrive.setSmartCurrentLimit(Constants.DrivetrainConstants.stallCurrentLimit, 
                                        Constants.DrivetrainConstants.freeCurrentLimit, 
                                        Constants.DrivetrainConstants.stallThresholdRPM);
    rightRearEncoder = rightRearDrive.getEncoder();
    rightRearDrive.follow(rightFrontDrive);
  }
}
