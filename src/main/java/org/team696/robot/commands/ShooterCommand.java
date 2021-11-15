/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.team696.robot.RobotContainer;
import org.team696.robot.Constants.IntakeConstants;
import org.team696.robot.Constants.OperatorConstants;
import org.team696.robot.Constants.ShooterConstants;
import org.team696.robot.subsystems.Intake;
import org.team696.robot.subsystems.Shooter;
import org.team696.robot.subsystems.ShooterHood;

public class ShooterCommand extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */
  private static final Logger logger = LogManager.getLogger(ShooterCommand.class);
  private Shooter shooter;
  private Intake intake;
  private ShooterHood shooterHood;


  double RPM;
  boolean state;
  double position;
  double angle;
  double timer;
  public ShooterCommand(Shooter shooter, double RPM, boolean state, Intake intake, double position, ShooterHood shooterHood, double angle) {

    this.shooter = shooter;
    this.state = state;
    this.RPM = RPM;
    this.intake = intake;
    this.position = position;
    this.shooterHood = shooterHood;
    this.angle = angle;

    addRequirements(shooter);
    addRequirements(intake);
    addRequirements(shooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    logger.info(String.format("Spinning up shooter to %f RPM", RPM));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
    // if(RobotContainer.operatorPanel.getRawButton(OperatorConstants.shooterAutoButton)){
    shooter.setShooterVelocity(shooter.shooterRPMToTalonVelocity(shooter.getShootRPM()));
    // }
    // else if(RobotContainer.operatorPanel.getRawButton(OperatorConstants.shooterManualButton)){
    //   // shooter.setShooterVelocity(shooter.shooterRPMToTalonVelocity(RPM+1000*RobotContainer.operatorPanel.getRawAxis(OperatorConstants.shooterManualAxis)));
    //   shooter.setShooterPower(shooter.shooterRPMToTalonVelocity(ShooterConstants.trenchRunTargetRPM));
    // }
    // else{
    //   shooter.setShooterPower(0);
    // }
  
    shooter.setAcceleratorPower(state);
    intake.moveIntake(position);
    intake.MoveRightIntake(position);

    shooterHood.moveServoAngle(angle);


    //TODO: make another command to run off of feedforward gains
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.info("Done with shooter command.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
