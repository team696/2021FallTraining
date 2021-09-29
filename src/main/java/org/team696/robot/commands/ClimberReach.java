/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// package org.team696.robot.commands;

// import org.team696.robot.subsystems.ClimberSubsystem;
// import org.team696.robot.RobotContainer;
// import org.team696.robot.Constants.OperatorConstants;
// import org.team696.robot.subsystems.ClimberServos;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class ClimberReach extends CommandBase {
//   private ClimberSubsystem climberSubsystem;
//   private ClimberServos servoSubsystem;
//   /**
//    * Creates a new ClimberPushDown.
//    */

//    boolean servoIsOpen = false;

//   public ClimberReach(ClimberSubsystem climberSubsystem, ClimberServos servoSubsystem) {
//     this.climberSubsystem = climberSubsystem;
//     this.servoSubsystem = servoSubsystem;
//     addRequirements(climberSubsystem);
//     addRequirements(servoSubsystem);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   // servoSubsystem.moveClimberServos(180);

//   if (RobotContainer.operatorPanel.getRawButton(OperatorConstants.climberDownButton)){
//        climberSubsystem.setClimberPower();

//   }
//   else{
//     climberSubsystem.setClimberPower(0);
//   }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     //ClimberSubsystem.climberState = ClimberStates.AT_HEIGHT;
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     //if sensor trips return tru
//     // return climberSubsystem.isLatchedOn();
//     return false;
//   }
// }
