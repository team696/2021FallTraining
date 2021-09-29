/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import org.team696.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTimer extends CommandBase {
  private Drivetrain drivetrain; 
  /**
   * Creates a new DriveTimer.
   */
  int timer;
  int x = 0;
  double speed;
  double turn;
  public DriveTimer(Drivetrain drivetrain, double speed, double turn, int timer) {
    this.drivetrain = drivetrain;
    this.timer = timer;
    this.speed = speed;
    this.turn = turn;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(speed, turn);
    x++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
    x = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x>timer;
  }
}
