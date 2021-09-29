/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import org.team696.TCA9548ADriver.TCA9548A;
import org.team696.robot.Constants.SpindexerConstants;
import org.team696.robot.subsystems.BallSensors;

/**
 * Add your docs here.
 */
public class Spindexer extends SubsystemBase {
  private static final Logger logger = LogManager.getLogger(Spindexer.class);

  private boolean isMoving;
  private int targetPocket = 1;
  private boolean isOnTarget;
  private static Boolean isJammed = false;
  private boolean[] ballOccupancy = new boolean[SpindexerConstants.nPockets];

  private int forwardTimer;
  private int backTimer;

  public enum SpindexerLoadingStates {
    FORWARD_TIMER, FORWARD_OK, BACK_TIMER, BACK_OK
  }
  private SpindexerLoadingStates spindexerStates = SpindexerLoadingStates.FORWARD_TIMER;

  // Stuff related to the driving motor
  private final CANSparkMax spindexerMotor = new CANSparkMax(SpindexerConstants.MotorCANID, MotorType.kBrushless);
  private final CANPIDController motorPID = spindexerMotor.getPIDController();
  private final CANEncoder motorEncoder = spindexerMotor.getEncoder();

  // Absolute encoder
  private final DigitalInput encoderDI = new DigitalInput(SpindexerConstants.EncoderPort);
  private final DutyCycle encoder = new DutyCycle(encoderDI);

  // Ball color sensors
  TCA9548A mux = new TCA9548A(Port.kOnboard, SpindexerConstants.ColorSensorsMuxAddress);
  private final BallSensors sensors = new BallSensors(mux,
      new byte[] { SpindexerConstants.ColorSensor1MuxChannel, SpindexerConstants.ColorSensor2MuxChannel,
          SpindexerConstants.ColorSensor3MuxChannel, SpindexerConstants.ColorSensor4MuxChannel,
          SpindexerConstants.ColorSensor5MuxChannel });

  // Kick motor
  private final CANSparkMax kickMotor = new CANSparkMax(SpindexerConstants.KickMotorCANID, MotorType.kBrushless);

  public Spindexer() {
    spindexerMotor.restoreFactoryDefaults();
    spindexerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    spindexerMotor.setClosedLoopRampRate(SpindexerConstants.closedLoopRampRate);
    spindexerMotor.setOpenLoopRampRate(SpindexerConstants.openLoopRampRate);
    motorPID.setP(SpindexerConstants.P);
    sensors.initSensors();
    kickMotor.setIdleMode(IdleMode.kBrake);
  }

  public int getTargetPocket() {
    return targetPocket;
  }

  public boolean isMoving() {
    return isMoving;
  }

  public boolean isOnTarget() {
    return isOnTarget;
  }

  /**
   * Sets brake mode of spindexer motor.
   * @param isBrake True for brake, false for coast
   */
  public void setBrake(boolean isBrake){
    if(isBrake){
      spindexerMotor.setIdleMode(IdleMode.kBrake);
    } else{
      spindexerMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Safely sets target pocket.
   * 
   * @param pocket Intended target pocket (bounds-checked)
   */
  public void setTargetPocket(int pocket) {
    if (1 <= pocket && pocket <= SpindexerConstants.nPockets) {
      // logger.info(String.format("Setting spindexer target to pocket %d.", pocket));
      targetPocket = pocket;
    }
    else{
      // logger.error(String.format("Received request to set spindexer target to pocket %d. Ignoring.", pocket));
    }
  }

  public double getMotorPosition() {
    return motorEncoder.getPosition();
  }

  public double getCurrent() {
    return spindexerMotor.getOutputCurrent();
  }

  public double getVelocity() {
    return motorEncoder.getVelocity();
  }

  /**Gets ball occupancy array.
   * @return Which pockets have balls
   */
  public boolean[] getBallOccupancy() {
    return ballOccupancy;
  }

  @Override
  public void periodic() {
    // check if jammed
    if (getCurrent() > SpindexerConstants.indexingJamDetectionCurrent) {
      isJammed = true;
      logger.warn("Spindexer jammed!");
    } else {
      isJammed = false;
    }

    // Check if moving
    isMoving = (motorEncoder.getVelocity() > SpindexerConstants.VelocityTolerance);

    // Check if on target
    if ((Math.abs(SpindexerConstants.PocketPositions[targetPocket - 1]
        - getEncoderPosition()) < SpindexerConstants.PositionTolerance) && !isMoving) {
      isOnTarget = true;
      // updateBallOccupancy();
    } else {
      isOnTarget = false;

      if(isJammed){
        // logger.warn(String.format("Jam while advancing to pocket %d. Reverting to previous pocket."));
        if (getVelocity() > 0) {
          if (targetPocket == 1) {
            setTargetPocket(5);
          } else {
            setTargetPocket(targetPocket - 1);
          }
        } else {
          if (targetPocket == 5) {
            setTargetPocket(1);
          } else {
            setTargetPocket(targetPocket + 1);
          }
        }
      }

      motorPID.setReference(getMotorPositionForPocket(targetPocket), ControlType.kPosition);

    }
  }

  /**
   * Gets the currently-indexed pocket.
   * 
   * @return Current pocket (1-5), or 0 if not on a pocket
   */
  public int getCurrentPocket() {
    if (isOnTarget) {
      return targetPocket;
    } else {
      // TODO: Need a better way to represent "not at a pocket"
      return 0;
    }
  }

  /**
   * Gets the position of the spindexer, based on the absolute encoder.
   * 
   * @return Encoder position on (0, 1)
   */
  public double getEncoderPosition() {
    double freq = encoder.getFrequency();
    // TODO: Check for invalid frequency (would suggest disconnected encoder)
    double ratio = encoder.getOutput();
    @SuppressWarnings("checkstyle:magicnumber")
    int highTime = (int) (((1. / freq) * ratio) * 1.e6); // Gets high time of signal in microseconds
    // Map microseconds to rotations
    return (double) (highTime - SpindexerConstants.RevTBEMinMicroseconds)
        / (double) (SpindexerConstants.RevTBEMaxMicroseconds - SpindexerConstants.RevTBEMinMicroseconds);
  }

  /**
   * Gets the required motor position to get to the given pocket ASAP.
   * 
   * @param pocket The index of the pocket to go to (1, ..., 5)
   * @return The required motor position, taking the current motor position into
   *         account
   */
  @SuppressWarnings("checkstyle:magicnumber")
  private double getMotorPositionForPocket(int pocket) {
    double currentPos = getEncoderPosition();
    double targetPos = SpindexerConstants.PocketPositions[pocket - 1];
    double toGo = computeWrappedDistance(currentPos, targetPos);
    if ((0.5 - Math.abs(toGo)) < 0.1) {
      toGo = Math.abs(toGo);
    }
    // System.out.printf("toGo: %f\n", toGo);
    return getMotorPosition() + (toGo * SpindexerConstants.MotorTurnsPerSpindexerTurn);
  }

  /**
   * Helper function to compute shortest wrapped angular distance.
   * 
   * @param from From point, on (0, 1)
   * @param to   To point, on (0, 1)
   * @return Shortest distance, on (-0.5, 0.5)
   */
  @SuppressWarnings("checkstyle:magicnumber")
  private double computeWrappedDistance(double from, double to) {
    // Adapted from https://stackoverflow.com/a/28037434
    double diff = (to - from + 0.5) % 1 - 0.5;
    return diff < -0.5 ? diff + 1 : diff;
  }

  /**
   * Finds the pocket closest to the current position.
   * @return Index of closest pocket, in the range (1, SpindexerConstants.nPockets)
   */
  public int findNearestPocket(){
    double currentPos = getEncoderPosition();
    int i; 
    int nearest=0; 
    double nearestDistance = 1;
    for(i=0; i<SpindexerConstants.nPockets; i++){
      double distance = Math.abs(computeWrappedDistance(currentPos, SpindexerConstants.PocketPositions[i]));
      if(distance < nearestDistance){
        nearestDistance = distance;
        nearest = i+1;
      }
    }
    return nearest;
  }

  /**
   * Updates the ballOccupancy array using sensor data. Uses the current position
   * of the spindexer to figure out which pocket is aligned with a given sensor.
   * 
   * @return None
   */
  private void updateBallOccupancy() {
    int offset = getCurrentPocket() - 1;
    boolean[] sensorReadings = sensors.getResults();
    int i;
    for (i = 0; i < SpindexerConstants.nPockets; i++) {
      ballOccupancy[(i + offset) % (SpindexerConstants.nPockets + 1)] = sensorReadings[i];
    }
  }

  /**
   * Finds the filled pocket closest to the current pocket.
   * 
   * @return The closest filled pocket, or 0 if no pockets are filled.
   */
  public int findNextFilledPocket() {
    int current = getCurrentPocket();
    int toCheckA;
    int toCheckB;
    int i;

    // Check adjacent
    for (i = 0; i < (SpindexerConstants.nPockets / 2); i++) {
      toCheckA = (current + i) % (SpindexerConstants.nPockets);
      toCheckB = (current - i) < 1 ? (current - i) + SpindexerConstants.nPockets : (current - i);

      if (ballOccupancy[toCheckA]) {
        return toCheckA;
      }
      if (ballOccupancy[toCheckB]) {
        return toCheckB;
      }
    }

    // TODO: need better way to indicate no occupied pockets
    return 0;
  }

  /**
   * Counts the total number of filled pockets.
   * 
   * @return Number of filled pockets
   */
  public int getNumberOfFilledPockets() {
    int result = 0;
    for (Boolean p : ballOccupancy) {
      if (p) {
        result++;
      }
    }
    return result;
  }

  /**
   * Turns kick motor on or off.
   * 
   * @param state
   */
  public void setKickMotor(double power) {
    // if (state) {
      kickMotor.set(power);
    // } else {
    //   kickMotor.set(0);
    // }
  }

  /**
   * This method runs the spindexer for loading and will change directions when
   * the ball gets jammed and goes above a threshold of current
   * 
   * @param power to set spindexer SparkMax to, Might need to change this to
   *              velocity pid control
   */
  public void spindexerLoadingAntiJam(double power) {

    switch (spindexerStates) {

    case FORWARD_TIMER:

      spindexerMotor.set(power);
      forwardTimer++;
      if (forwardTimer > SpindexerConstants.antiJamTimeout) {
        forwardTimer = 0;
       // logger.info("Transitioning to FORWARD_OK");
        spindexerStates = SpindexerLoadingStates.FORWARD_OK;
      }

      break;

    case FORWARD_OK:
      spindexerMotor.set(power);
      if (getCurrent() > SpindexerConstants.loadingJamDetectionCurrent) {
       // logger.info("Jam detected! Transitioning to BACK_TIMER.");
        spindexerStates = SpindexerLoadingStates.BACK_TIMER;
      }
      break;

    case BACK_TIMER:
      spindexerMotor.set(-power);
      backTimer++;
      if (backTimer > SpindexerConstants.antiJamTimeout) {
        backTimer = 0;
       // logger.info("Transitioning to BACK_OK");
        spindexerStates = SpindexerLoadingStates.BACK_OK;
      }
      break;

    case BACK_OK:
      spindexerMotor.set(-power);
      if (getCurrent() > SpindexerConstants.loadingJamDetectionCurrent) {
       // logger.info("Jam detected! Transitioning to FORWARD_TIMER.");
        spindexerStates = SpindexerLoadingStates.FORWARD_TIMER;
      }
      break;

    default:
      break;

    }

  }

  public void goToNotAPocket() {
    motorPID.setReference(0.867, ControlType.kPosition);
  }
}
