/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import org.team696.robot.RobotContainer;
import org.team696.robot.Constants.TurretConstants;
import org.team696.robot.subsystems.Limelight;
import org.team696.robot.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurretManual extends CommandBase {
  /**
   * Creates a new TurretManual.
   */
  TurretSubsystem turretSubsystem;
  Limelight limelight;
  double position;
  public TurretManual(TurretSubsystem turretSubsystem, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.turretSubsystem = turretSubsystem;
    this.limelight = limelight;
    this.position = position;
    addRequirements(turretSubsystem);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // limelight.setLights(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turretSubsystem.moveToPosition(TurretConstants.turretCenterPos+position*100);
    // System.out.println("turret manual");
    // System.out.println(position);
    if(RobotContainer.operatorPanel.getRawAxis(0)>0.05){
      limelight.setLights(1);
    }
    else{
      limelight.setLights(1);
    }
    turretSubsystem.openLoop(-RobotContainer.operatorPanel.getRawAxis(0));
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
