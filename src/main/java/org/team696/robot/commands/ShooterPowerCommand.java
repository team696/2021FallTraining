/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.Constants.IntakeConstants;
import org.team696.robot.subsystems.Intake;
import org.team696.robot.subsystems.Shooter;
import org.team696.robot.subsystems.ShooterHood;

public class ShooterPowerCommand extends CommandBase {
  /**
   * Creates a new ShooterPowerCommand.
   */
  private Shooter shooter;
  private Intake intake;
  private ShooterHood shooterHood;
  private double angle;
  private double power;
  private boolean state;
  private double position;
  public ShooterPowerCommand(Shooter shooter, double power, boolean state, Intake intake, double position, ShooterHood shooterHood , double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.intake = intake;
    this.state = state;
    this.power = power;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterPower(power);
    shooter.setAcceleratorPower(state);
    intake.MoveRightIntake(position);
    intake.moveIntake(position);
    shooterHood.moveServoAngle(angle);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
