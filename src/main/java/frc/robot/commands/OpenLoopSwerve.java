/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OpenLoopSwerve extends Command {
  public OpenLoopSwerve() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.swervy);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.oi.xbox1.getAButton()) {
      Robot.swervy.FR.setMotorA(Robot.oi.getXbox1().getY(GenericHID.Hand.kLeft));
      Robot.swervy.FR.setMotorA(Robot.oi.getXbox1().getY(GenericHID.Hand.kRight));
    } else {
      Robot.swervy.convertDesiredWheelMotionToMotorOutputs(Robot.swervy.convertIntentToDesiredAnglesAndSpeeds(Robot.swervy.convertJoystickToIntentAxes()));
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
