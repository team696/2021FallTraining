/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.subsystems.Shooter;

public class ShooterTimerCommand extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */
  private Shooter shooter;

  double RPM;
  boolean state;
  int x = 0;
  public ShooterTimerCommand(Shooter shooter, double RPM, boolean state) {

    this.shooter = shooter;
    this.state = state;
    this.RPM = RPM;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterVelocity(shooter.shooterRPMToTalonVelocity(RPM));
    shooter.setAcceleratorPower(state);
    System.out.println("shooting");
    x++;

    //TODO: make another command to run off of feedforward gains
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    x = 0;
    shooter.setShooterPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x>150;
  }
}
