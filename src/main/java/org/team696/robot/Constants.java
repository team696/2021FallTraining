/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int CANtimeout = 10;

  public static final int timeBetweenPockets = 50;

  public static final class SpindexerConstants {
    public static final int nPockets = 5;
    public static final int MotorCANID = 20;
    public static final int EncoderPort = 0;
    public static final double P = 0.08;
    public static final double closedLoopRampRate = 0.5;
    public static final double openLoopRampRate = 0.25;
    public static final int RevTBEMinMicroseconds = 1;
    public static final int RevTBEMaxMicroseconds = 1024;
    // public static final double[] PocketPositions = {0.9335, 0.1398, 0.3431,
    // 0.5386, 0.7351};
    // public static final double[] PocketPositions = {0.9689, 0.1689, 0.3689,
    // 0.5689, 0.7689};
    public static final double[] PocketPositions = { 0.964, 0.173, 0.375, 0.580, 0.775 };
    public static final double PositionTolerance = 0.005; // Used to calcuate isOnTarget
    public static final double VelocityTolerance = 0.01; // used to calculate isMoving
    public static final double MotorTurnsPerSpindexerTurn = 62.5;
    public static final byte ColorSensorsMuxAddress = 0x70;
    public static final byte ColorSensor1MuxChannel = 0;
    public static final byte ColorSensor2MuxChannel = 1;
    public static final byte ColorSensor3MuxChannel = 2;
    public static final byte ColorSensor4MuxChannel = 3;
    public static final byte ColorSensor5MuxChannel = 7;

    public static final double HueMax = 1;
    public static final double HueMin = 0;
    public static final double SatMax = 1;
    public static final double SatMin = 0;
    public static final double ValueMax = 1;
    public static final double ValueMin = 0;

    public static final int KickMotorCANID = 21;
    public static final double KickMotorSpeed = 1.;
    public static final double KickMotorReverseSpeed = -0.1;

    public static final double loadingDrumPower = 0.22;
    public static final double continuousShootDrumPower = 0.55;
    public static final double stopDrumPower = 0;

    public static final double timeBetweenIndex = 1;

    public static final double indexingJamDetectionCurrent = 15;
    public static final double loadingJamDetectionCurrent = 10;
    public static final double antiJamTimeout = 50;
  }

  public static final class ShooterConstants {
    public static final int leftShooterHoodServo = 2;
    public static final int rightShooterHoodServo = 3;

    public static final int leftShooterPort = 42;
    public static final boolean leftShooterInverted = false;
    public static final boolean leftShooterSensorPhase = false;
    public static final int rightShooterPort = 43;
    public static final boolean rightShooterInverted = true;
    public static final boolean rightShooterSensorPhase = false;
    public static final int pidSlot = 0;
    public static double shooterkP = 0.6;
    public static double shooterkI = 0.0;
    public static double shooterkD = 0.0;
    public static double shooterkF = 0.06;
    public static final int allowableShooterError = 10;

    // TODO: run simple motor characterization routine to deterine feedforward gains
    public static final double shooterkSGain = 0;
    public static final double shooterkVGain = 0;
    public static final double shooterkAGain = 0;

    public static final double shooterHoodAngleMaxLimit = 115;
    public static final double shooterHoodAngleMinLimit = 175;

    public static final int acceleratorPort = 41;
    public static final double acceleratorPower = 0.3;
    public static double acceleratorkP = 0;
    public static double acceleratorkI = 0;
    public static double acceleratorkD = 0;
    public static double acceleratorkF = 0;
    public static final int allowableAcceleratorError = 100;
    public static final boolean acceleratorInverted = true;
    public static final boolean acceleratorSensorPhase = false;

    public static final double shooterGearRatio = 2.5;

    public static final double autoLineTargetRPM = 3900;
    public static final double autoLineTargetAngle = 30;

    public static final double trenchRunTargetRPM = 5742.27;
    public static final double trenchRunTargetAngle = 18.6;

  }

  public static final class TurretConstants {
    public static final int turretMotorPort = 40;
    // public static final boolean turret

    public static final boolean turretMotorInverted = false;
    public static final boolean turretMotorSensorPhase = true;

    public static final int forwardSoftLimit = -370;
    public static final int reverseSoftLimit = -570;

    public static final int turretCenterPos = -540;

    public static final double positionTolerance = 0.6;

    public static final double peakOutput = 0.75;

    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.001;
  }


  public static final class LimelightConstants {
    public static final int camTranPipeline = 1;
    public static final int loadingpipeline = 1;
  }

  public static final class OperatorConstants {
    public static final int operatorPanelPort = 0;
    public static final int driverJoystickPort = 1;

    // Driver gamepad mappings
    public static final int driveModeButton = 4;

    // Operator panel mappings
    public static final int ATCRevButton = 1;
    public static final int intakeOnButton = 3;
    public static final int intakeOffButton = 4;
    public static final int intakeOutButton = 5;
    public static final int fitUnderTrenchButton = 6;
    public static final int intakeInButton = 6;
    public static final int ATCForButton = 7;
    public static final int fireButton = 8;
    public static final int hoodAutoButton = 9;
    public static final int shooterAutoButton = 10;
    public static final int colorWheelPositionControl = 11;
    public static final int climberUpButton = 12;
    public static final int colorWheelDeploy = 13;
    public static final int turretAutoButton = 14;
    public static final int shooterManualButton = 15;
    public static final int spinUpButton = 16;
    public static final int climberDownButton = 17;
    public static final int colorWheelRotationControl = 18;
    public static final int driveAssistButton = 19;

    public static final int shooterManualAxis = 3;
    public static final int turretManualAxis = 2;
    public static final int hoodManualAxis = 1;

    public static final int ATCRevLED = 0;
    public static final int limelightLockLED = 1;
    public static final int limelightCaptureLED = 2;
    public static final int ATCForLED = 3;
    public static final int fireLED = 4;
    public static final int realsenseCaptureLED = 5;
    public static final int realsenseLockLED = 6;
    public static final int carabinerLatchLED = 7;
    public static final int climberUpLimLED = 8;
    public static final int climberDownLimLED = 9;
    public static final int colorWheelDeployLED = 10;
  }

  public static final class DrivetrainConstants {
    public static final int leftFrontCANID = 10;
    public static final int leftRearCANID = 11;
    public static final int rightFrontCANID = 12;
    public static final int rightRearCANID = 13;

    public static final boolean leftFrontIsInverted = true;
    public static final boolean leftRearIsInverted = true;
    public static final boolean rightFrontIsInverted = false;
    public static final boolean rightRearIsInverted = false;

    public static final double wheelDiameter = 6.; // In inches. Used for calculating speed.
    public static final double driveGearboxReduction = (60. / 10.) * (30. / 18.);

    public static final int IMUCalTime = 8;
    public static final double yawkP = 1.;
    public static final double yawkI = 0.;
    public static final double yawkD = 0.;

    public static final double maxSpeed = 184.6; // In in/s
    public static final double maxTurnRate = 5; // In degrees/iteration

    public static final int stallCurrentLimit = 80;
    public static final int freeCurrentLimit = 80;
    public static final int stallThresholdRPM = 1000;

    public static final double openLoopRampRate = 0.5;

    public static final double driveControllerDeadband = 0.05;

    public static final int slowTasksSpeed = 50;
  }

  public static final class IntakeConstants {
    public static final int IntakeMotorPort = 50;
    public static final boolean IntakeInverted = false;

    public static final double intakePower = 0.7;
    public static final double outtakePower = -1;
    public static final double stopIntakePower = 0;
  }
}
