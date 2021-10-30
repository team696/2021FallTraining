/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import org.team696.MathUtils.MathUtils;
import org.team696.robot.Constants.IntakeConstants;
import org.team696.robot.Constants.OperatorConstants;
import org.team696.robot.Constants.SpindexerConstants;
// import org.team696.robot.commands.ATCForCommand;
// import org.team696.robot.commands.ATCRevCommand;
import org.team696.robot.commands.AutoIndexKickUp;
import org.team696.robot.commands.Drive;
import org.team696.robot.commands.DriveTimer;
import org.team696.robot.commands.FireCommand;
import org.team696.robot.commands.IntakeTimerCommand;
import org.team696.robot.commands.OmniKickDown;
import org.team696.robot.commands.OmniKickUp;
import org.team696.robot.commands.OmniKickUpTimer;
import org.team696.robot.commands.ShooterCommand;
import org.team696.robot.commands.ShooterHoodCommand;
import org.team696.robot.commands.ShooterPowerCommand;
import org.team696.robot.commands.ShooterPrep;
// import org.team696.robot.commands.SpinToPocket;
import org.team696.robot.commands.SpindexerLoading;
import org.team696.robot.commands.TurretLockOn;
import org.team696.robot.commands.TurretManual;
import org.team696.robot.subsystems.Drivetrain;
import org.team696.robot.subsystems.Intake;
import org.team696.robot.subsystems.Limelight;
import org.team696.robot.subsystems.Shooter;
import org.team696.robot.subsystems.ShooterHood;
import org.team696.robot.subsystems.Spindexer;
import org.team696.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subystem declarations
  private static final Logger logger = LogManager.getLogger(RobotContainer.class);
  public final Drivetrain drivetrain = new Drivetrain();
  public final Spindexer spindexer = new Spindexer();
  public final Shooter shooter = new Shooter();
  public final ShooterHood shooterHood = new ShooterHood();
  public final TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static final Limelight limelight = new Limelight();
  public final Intake intake = new Intake();

  // Joysticks
  public final XboxController driverController = new XboxController(OperatorConstants.driverJoystickPort);
  public static final Joystick operatorPanel = new Joystick(OperatorConstants.operatorPanelPort);

  // Buttons

  // make hood/shooter auto/man to change from 2 distances
  // make intake in/out be for color wheel height
  private final JoystickButton driveMode = new JoystickButton(driverController, OperatorConstants.driveModeButton);

  private final JoystickButton spinUpButton = new JoystickButton(operatorPanel, OperatorConstants.spinUpButton);
  private final JoystickButton shooterAutoButton = new JoystickButton(operatorPanel, OperatorConstants.shooterAutoButton);
  private final JoystickButton shooterManualButton = new JoystickButton(operatorPanel, OperatorConstants.shooterManualButton);
  private final JoystickButton fireButton = new JoystickButton(operatorPanel, OperatorConstants.fireButton);
  private final JoystickButton ATCForButton = new JoystickButton(operatorPanel, OperatorConstants.ATCForButton);
  private final JoystickButton ATCRevButton = new JoystickButton(operatorPanel, OperatorConstants.ATCRevButton);

  private final JoystickButton turretAutoButton = new JoystickButton(operatorPanel, OperatorConstants.turretAutoButton);
  private final JoystickButton hoodAutoButton = new JoystickButton(operatorPanel, OperatorConstants.hoodAutoButton);

  private final JoystickButton intakeOnButton = new JoystickButton(operatorPanel, OperatorConstants.intakeOnButton);
  private final JoystickButton intakeOffButton = new JoystickButton(operatorPanel, OperatorConstants.intakeOffButton);

  private final JoystickButton fitUnderTrenchButton = new JoystickButton(operatorPanel, OperatorConstants.fitUnderTrenchButton);

  private final JoystickButton continuousButton = new JoystickButton(operatorPanel, OperatorConstants.colorWheelDeploy);
  private final JoystickButton omniReverse = new JoystickButton(operatorPanel, OperatorConstants.colorWheelRotationControl);
  private final JoystickButton automaticShootPrep = new JoystickButton(operatorPanel, OperatorConstants.driveAssistButton);
  private final JoystickButton automaticIndex =  new JoystickButton(operatorPanel, OperatorConstants.colorWheelPositionControl);

  public double getDriveSpeed() {
    // Math to correct for nonlinearities in the controller should happen here.
    double jsVal = -driverController.getY(Hand.kLeft);
    if (jsVal > 0) {
      return MathUtils.deadBand(jsVal, Constants.DrivetrainConstants.driveControllerDeadband);
    } else {
      return -MathUtils.deadBand(-jsVal, Constants.DrivetrainConstants.driveControllerDeadband);
    }
  }

  public double getDriveTurn() {
    // Math to correct for nonlinearities in the controller should happen here.
    double jsVal = -driverController.getX(Hand.kRight);
    if (jsVal > 0) {
      return MathUtils.deadBand(jsVal, Constants.DrivetrainConstants.driveControllerDeadband);
    } else {
      return -MathUtils.deadBand(-jsVal, Constants.DrivetrainConstants.driveControllerDeadband);
    }
  }

  public double turretManual() {
    return operatorPanel.getRawAxis(1);
  }

  public RobotContainer() {
    Command driveCommand = new Drive(() -> getDriveSpeed(), () -> getDriveTurn(), drivetrain);
    drivetrain.setDefaultCommand(driveCommand);
    configureButtonBindings();
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private void configureButtonBindings() {
    // rpm is 3900
    spinUpButton.whenPressed(new ShooterCommand(shooter, shooter.shootRPM, true));
    spinUpButton.whenReleased(new ShooterPowerCommand(shooter, 0, false));

    turretAutoButton.whileHeld(new TurretLockOn(turretSubsystem, limelight));
    turretAutoButton.whenReleased(new TurretManual(turretSubsystem, limelight));

    SpindexerLoading loadingOff = new SpindexerLoading(spindexer, intake, SpindexerConstants.stopDrumPower, IntakeConstants.stopIntakePower);
    intakeOnButton.whileHeld(
        new SpindexerLoading(spindexer, intake, SpindexerConstants.loadingDrumPower, IntakeConstants.intakePower));
    intakeOnButton.whenReleased(loadingOff);
    intakeOffButton.whileHeld(
        new SpindexerLoading(spindexer, intake, SpindexerConstants.stopDrumPower, IntakeConstants.outtakePower));
    intakeOffButton.whenReleased(loadingOff);

    // maybe have this button cancel the human player loading sequence
    continuousButton.whileHeld(new SpindexerLoading(spindexer, intake, SpindexerConstants.continuousShootDrumPower,
        IntakeConstants.stopIntakePower));
    continuousButton.whenReleased(loadingOff);

    automaticShootPrep.whenPressed(new ShooterPrep(shooter, turretSubsystem));
    automaticShootPrep.whenReleased(new ShooterPowerCommand(shooter, 0, false));
    automaticShootPrep.whenReleased(new TurretManual(turretSubsystem, limelight));

    automaticIndex.whenPressed(new AutoIndexKickUp(spindexer, intake, SpindexerConstants.continuousShootDrumPower, 0));
    automaticIndex.whenReleased(loadingOff);

    fireButton.whenPressed(new FireCommand(shooter, spindexer, SpindexerConstants.KickMotorSpeed));
    fireButton.whenReleased(new FireCommand(shooter, spindexer, SpindexerConstants.stopDrumPower));

    // OmniKickUp reverseOmni = new OmniKickUp(spindexer, SpindexerConstants.KickMotorReverseSpeed);
    // OmniKickDown reverseOmni = new OmniKickDown(spindexer, -0.1, shooter, -0.5);
    omniReverse.whenPressed(new OmniKickDown(spindexer, -0.1, shooter, -0.5));
    omniReverse.whenReleased(new OmniKickDown(spindexer, 0.0, shooter, 0.0));

    // ATCForCommand ATCfor = new ATCForCommand(spindexer);
    // ATCRevCommand ATCrev = new ATCRevCommand(spindexer);

    // ATCForButton.whenPressed(ATCfor);
    // ATCForButton.cancelWhenPressed(ATCrev);

    // ATCRevButton.whenPressed(ATCrev);
    // ATCRevButton.cancelWhenPressed(ATCfor);

    // intakeOnButton.whenReleased(ATCfor);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @SuppressWarnings("checkstyle:magicnumber")
  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(

        new ParallelCommandGroup(
            new TurretLockOn(turretSubsystem, limelight),
            new ShooterCommand(shooter, 3900, true),

            new SequentialCommandGroup(
                  // new ShooterHoodCommand(shooterHood, 51), 
                          // new IntakeTimerCommand(intake, 50);
                // new SpinToPocket(spindexer, 1),
                //   new OmniKickUpTimer(spindexer, true, 25), 
                // new SpinToPocket(spindexer, 2),
                //   new OmniKickUpTimer(spindexer, true, 25), 
                // new SpinToPocket(spindexer, 3),
                //   new OmniKickUpTimer(spindexer, true, 25)
              )
          ),

        new DriveTimer(drivetrain, -0.5, 0, 50), 
        new DriveTimer(drivetrain, 0, 0.7, 40),
        new DriveTimer(drivetrain, 0.5, 0, 25),
        new ParallelCommandGroup(
          new DriveTimer(drivetrain, 0.5, 0, 35), 
          new IntakeTimerCommand(intake, 35)
          )
        

    );

  }

  public boolean getDriveMode() {
    return driveMode.get();
  }

  public void setLimelightCaptureLED(boolean state) {
    operatorPanel.setOutput(OperatorConstants.limelightCaptureLED, state);
  }

  public void setLimelightLockLED(boolean state) {
    operatorPanel.setOutput(OperatorConstants.limelightLockLED, state);
  }
}
