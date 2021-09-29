/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team696.robot.Constants;
import org.team696.robot.Robot;
import org.team696.robot.RobotContainer;
import org.team696.robot.TrajectoryTable;
import org.team696.robot.Constants.OperatorConstants;
import org.team696.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private WPI_TalonFX leftShooterMotor;
  private WPI_TalonFX rightShooterMotor;

  private SimpleMotorFeedforward shooterFeedForward;

  private WPI_TalonSRX shooterAcceleratorMotor;

  public double shootRPM;


  // private LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  public Shooter() {

    leftShooterMotor = new WPI_TalonFX(Constants.ShooterConstants.leftShooterPort);
    rightShooterMotor = new WPI_TalonFX(Constants.ShooterConstants.rightShooterPort);

    leftShooterMotor.configFactoryDefault();

    leftShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.ShooterConstants.pidSlot, Constants.CANtimeout);
    leftShooterMotor.setSensorPhase(Constants.ShooterConstants.leftShooterSensorPhase);
    leftShooterMotor.setInverted(Constants.ShooterConstants.leftShooterInverted);
    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    leftShooterMotor.configNominalOutputForward(0);
    leftShooterMotor.configNominalOutputReverse(0);
    leftShooterMotor.configPeakOutputForward(1);
    leftShooterMotor.configPeakOutputReverse(-1);

    leftShooterMotor.configAllowableClosedloopError(Constants.ShooterConstants.pidSlot,
        Constants.ShooterConstants.allowableShooterError);

    // leftShooterMotor.configContinuousCurrentLimit(amps)

    leftShooterMotor.config_kP(Constants.ShooterConstants.pidSlot, Constants.ShooterConstants.shooterkP);
    leftShooterMotor.config_kI(Constants.ShooterConstants.pidSlot, Constants.ShooterConstants.shooterkI);
    leftShooterMotor.config_kD(Constants.ShooterConstants.pidSlot, Constants.ShooterConstants.shooterkD);
    leftShooterMotor.config_kF(Constants.ShooterConstants.pidSlot, Constants.ShooterConstants.shooterkF);

    rightShooterMotor.configFactoryDefault();
    rightShooterMotor.follow(leftShooterMotor);
    rightShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.ShooterConstants.pidSlot, Constants.CANtimeout);
    rightShooterMotor.setSensorPhase(Constants.ShooterConstants.rightShooterSensorPhase);
    rightShooterMotor.setInverted(Constants.ShooterConstants.rightShooterInverted);

    shooterFeedForward = new SimpleMotorFeedforward(Constants.ShooterConstants.shooterkSGain,
        Constants.ShooterConstants.shooterkVGain, Constants.ShooterConstants.shooterkAGain);



    shooterAcceleratorMotor = new WPI_TalonSRX(Constants.ShooterConstants.acceleratorPort);
    shooterAcceleratorMotor.configFactoryDefault();
    shooterAcceleratorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
        Constants.ShooterConstants.pidSlot, Constants.CANtimeout);
    shooterAcceleratorMotor.configNominalOutputForward(0);
    shooterAcceleratorMotor.configNominalOutputReverse(0);
    shooterAcceleratorMotor.configPeakOutputForward(1);
    shooterAcceleratorMotor.configPeakOutputReverse(-1);
    shooterAcceleratorMotor.setInverted(Constants.ShooterConstants.acceleratorInverted);
    shooterAcceleratorMotor.setSensorPhase(Constants.ShooterConstants.acceleratorSensorPhase);
    shooterAcceleratorMotor.config_kP(Constants.ShooterConstants.pidSlot, Constants.ShooterConstants.acceleratorkP);
    shooterAcceleratorMotor.config_kI(Constants.ShooterConstants.pidSlot, Constants.ShooterConstants.acceleratorkI);
    shooterAcceleratorMotor.config_kD(Constants.ShooterConstants.pidSlot, Constants.ShooterConstants.acceleratorkD);
    shooterAcceleratorMotor.config_kF(Constants.ShooterConstants.pidSlot, Constants.ShooterConstants.acceleratorkF);

    // shooterHoodServo.dead

  }

  public void setShooterkP(double kP){
    leftShooterMotor.config_kP(0, kP);
  }
  public void setShooterkI(double kI){
    leftShooterMotor.config_kP(0, kI);
  }
  public void setShooterkD(double kD){
    leftShooterMotor.config_kP(0, kD);
  }
  public void setShooterkF(double kF){
    leftShooterMotor.config_kF(0, kF);
  }


  public void setAcceleratorkP(double kP){
    shooterAcceleratorMotor.config_kP(0, kP);
  }
  public void setAcceleratorkI(double kI){
    shooterAcceleratorMotor.config_kP(0, kI);
  }
  public void setAcceleratorkD(double kD){
    shooterAcceleratorMotor.config_kP(0, kD);
  }
  public void setAcceleratorkF(double kF){
    shooterAcceleratorMotor.config_kF(0, kF);
  }


  public void setFeederVelocity(double velocity) {

    shooterAcceleratorMotor.set(ControlMode.Velocity, velocity);
  }

  public void setFeederPower(double power) {
    shooterAcceleratorMotor.set(ControlMode.PercentOutput, power);

  }

  public double getShooterSetpoint(){
    return leftShooterMotor.getClosedLoopTarget();
  }

  public double getShooterPower(){
    return leftShooterMotor.getMotorOutputPercent();
  }
  /**
   * Commands shooter in open-loop.
   * @param power Shooter power (0-1)
   */
  public void setShooterPower(double power) {

    leftShooterMotor.set(ControlMode.PercentOutput, power);
    // rightShooterMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Commands accelerator in open-loop.
   * @param power Accelerator power (0-1)
   */
  public void setAcceleratorPower(double power) {
    shooterAcceleratorMotor.set(ControlMode.PercentOutput, power);
    // rightShooterMotor.set(ControlMode.PercentOutput, power);
  }

  public void setAcceleratorPower(boolean state) {
    if(state){
    shooterAcceleratorMotor.set(ControlMode.PercentOutput, 1);
    }
    else{
      shooterAcceleratorMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  /**
   * Commands shooter velocity in TalonFX native units.
   * @param velocity Shooter motor target velocity in encoder units per 100 ms
   */
  public void setShooterVelocity(double velocity) {
    leftShooterMotor.set(TalonFXControlMode.Velocity, velocity);
  }

  /**
   * Commands accelerator velocity.
   * @param velocity Velocity in Talon SRX native units
   */
  public void setAcceleratorVelocity(double velocity){
    shooterAcceleratorMotor.set(ControlMode.Velocity, velocity);
  }

  public void runShooterFeedForward(double targetVelocity, double targetAcceleration) {
    leftShooterMotor.setVoltage(calculateFeedForwardValue(targetVelocity, targetAcceleration));
    // might use voltage depends which works better
    // leftShooterMotor.set(TalonFXControlMode.PercentOutput, calculateFeedForward);
  }


  







  /**
   * Computes Talon velocity setpoint from RPM.
   * Use this value to feed directly into a shooter velocity control method
   * @param RPM Flywheel speed in RPM
   * @return Velocity value in TalonFX integrated sensor units per 100 ms. 
   */
  @SuppressWarnings("checkstyle:magicnumber")
  public double shooterRPMToTalonVelocity(double RPM){
    double retval = ((RPM / 600) * (2048 / ShooterConstants.shooterGearRatio));
    return retval;
  }

  /**
   * Computes flywheel RPM from native units.
   * @param velocity Falcon ticks / 100 ms
   * @return Flywheel speed in RPM
   */
  @SuppressWarnings("checkstyle:magicnumber")
  public double talonVelocityToShooterRPM(double velocity){
    double retval = velocity/2048. * 600 * ShooterConstants.shooterGearRatio;
    return retval;
  }

  /**
   * 
   * @param targetVelocity     Tarrget Velocity in the same units of gains in
   *                           FeedForward constructor
   * @param targetAcceleration Target Velocity in the same units of gains in
   *                           FeedForward constructor
   * @return Value that feedForward calculates to pass into talon's .set()
   */
  public double calculateFeedForwardValue(double targetVelocity, double targetAcceleration) {
    return shooterFeedForward.calculate(targetVelocity, targetAcceleration);
  }

  public double getLeftShooterCurrent() {
    return leftShooterMotor.getSupplyCurrent();
  }

  public double getRightShooterCurrent() {
    return rightShooterMotor.getSupplyCurrent();
  }

  public double getAcceleratorCurrent() {
    return shooterAcceleratorMotor.getSupplyCurrent();
  }


  /**
   * 
   * @return Left TalonFX Velocity in Encoder Units per 100 ms
   */
  public double getLeftShooterVelocity() {
    return leftShooterMotor.getSelectedSensorVelocity();
  }
  /**
   * @return Right TalonFX Velocity in Encoder Units per 100 ms
   */
  public double getRightShooterVelocicty() {
    return rightShooterMotor.getSelectedSensorVelocity();
  }

  /**
   * @return Returns Accelerator/Feeder Velocity in Encoder Units per 100 ms
   */
  public double getAcceleratorVelocity() {
    return shooterAcceleratorMotor.getSelectedSensorVelocity();
  }
  // public double getAcceleratorRPM() {
  //   return shooterAcceleratorMotor.
  // }

  /**
   * Gets flywheel velocity
   * @return Flywheel velocity in RPM
   */
  public double getRPM(){
    return talonVelocityToShooterRPM(getLeftShooterVelocity());
  }


  // public double getAutoRPM(){
  //   return TrajectoryTable.TrajectoryTable[Limelight.indexForDistance()][1];
  // }

  // public double getAutoAngle(){

  //   if(TrajectoryTable.TrajectoryTable[Limelight.indexForDistance()][0]>14&&TrajectoryTable.TrajectoryTable[Limelight.indexForDistance()][0]<53){
  //   return TrajectoryTable.TrajectoryTable[Limelight.indexForDistance()][0];
  //   }
  //   else if(TrajectoryTable.TrajectoryTable[Limelight.indexForDistance()][0]<14){
  //     return 14;
  //   }
  //   else{
  //     return 53;
  //   }
  // }

  // public void testShooterRPM(){
  //   setShooterVelocity(shooterRPMToTalonVelocity(testtar));
  // }

  public double  getShootRPM(){
    if(RobotContainer.operatorPanel.getRawButton(10)){
      return 4350;
    }
    else{
      return 2800; 
    }
  }

  public boolean isUpToSpeed(){
    if(talonVelocityToShooterRPM(getLeftShooterVelocity())>0.9*getShootRPM()){
      return true; 
    }
    else{
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    

}
}
