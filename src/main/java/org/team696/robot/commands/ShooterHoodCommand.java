/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team696.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team696.robot.RobotContainer;
import org.team696.robot.Constants.OperatorConstants;
import org.team696.robot.subsystems.Shooter;
import org.team696.robot.subsystems.ShooterHood;

public class ShooterHoodCommand extends CommandBase {
  /**
   * Creates a new ShooterHoodCommand.
   */
  private ShooterHood shooterHood;
  private double angle;

  private double timer; 
  public ShooterHoodCommand(ShooterHood shooterHood, double angle) {

    this.shooterHood  = shooterHood;
    this.angle = angle;
    addRequirements(shooterHood);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    timer++;
    System.out.println("adjusting hood");
    // if(RobotContainer.operatorPanel.getRawButton(OperatorConstants.shooterAutoButton)){
      shooterHood.moveServoAngle(angle);
    // }
    //   else if(RobotContainer.operatorPanel.getRawButton(OperatorConstants.shooterManualButton)){
    //     // shooter.setShooterVelocity(shooter.shooterRPMToTalonVelocity(RPM+1000*RobotContainer.operatorPanel.getRawAxis(OperatorConstants.shooterManualAxis)));
    //     shooterHood.moveServoAngle(shooterHood.shootAngleToServoAngle(18));

    //   }
    //   else{
    //     // shooter.setShooterPower(0);
    //     shooterHood.moveServoAngle(shooterHood.shootAngleToServoAngle(40));

    //   }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer>150;
  }
}
