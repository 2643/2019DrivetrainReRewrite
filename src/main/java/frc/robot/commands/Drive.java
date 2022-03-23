// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drive extends CommandBase {
  double leftSpeed;
  double rightSpeed;
  public static double slowSpeed = 1;
  /** Creates a new Drive. */
  public Drive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(slowSpeed <= 1){
      if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS)) > Constants.JOYSITCK_DEADBAND) {
        leftSpeed = (RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_LEFT_AXIS)* slowSpeed);
      }
      else{
        leftSpeed = 0;
      }
      if (Math.abs(RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS)) > Constants.JOYSITCK_DEADBAND) {
        rightSpeed = (RobotContainer.driveStick.getRawAxis(Constants.JOYSTICK_RIGHT_AXIS)* slowSpeed);
      }
      else {
        rightSpeed = 0;
      }
    }
    RobotContainer.m_DriveTrain.setLeftSpeed(leftSpeed);
    RobotContainer.m_DriveTrain.setRightSpeed(rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
