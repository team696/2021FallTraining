/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.subsystems.Shooter;

public class ShooterPowerCommand extends CommandBase {
  /**
   * Creates a new ShooterPowerCommand.
   */
  private Shooter shooter;
  private double power;
  private boolean state;
  public ShooterPowerCommand(Shooter shooter, double power, boolean state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.state = state;
    this.power = power;
    addRequirements(shooter);
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
