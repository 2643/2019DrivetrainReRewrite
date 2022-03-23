// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Drive;

public class DriveTrain extends SubsystemBase {
  public static int lFrontMotorPort = 29;   
  public static int lBackMotorPort = 21;    
  public static int rFrontMotorPort = 26;   
  public static int rBackMotorPort = 38;
  public static TalonSRX LeftFrontMotor = new TalonSRX(lFrontMotorPort);
  public static TalonSRX LeftBackMotor = new TalonSRX(lBackMotorPort);
  public static TalonSRX RightFrontMotor = new TalonSRX(rFrontMotorPort);
  public static TalonSRX RightBackMotor = new TalonSRX(rBackMotorPort);
  /** Creates a new DriveTrain. */

  ShuffleboardTab TalonFXTab = Shuffleboard.getTab("2019");
  NetworkTableEntry lol2 = TalonFXTab.add("Slow Speed(From 0-1)", 1).getEntry();
  public DriveTrain() 
  {
        LeftFrontMotor.configContinuousCurrentLimit(32, 0);
        LeftFrontMotor.configPeakCurrentLimit(35, 0);
        LeftFrontMotor.configPeakCurrentDuration(80, 0);
        LeftFrontMotor.enableCurrentLimit(true);

        LeftBackMotor.follow(LeftFrontMotor);
        RightBackMotor.follow(RightFrontMotor);

        RightFrontMotor.configContinuousCurrentLimit(32, 0);
        RightFrontMotor.configPeakCurrentLimit(35, 0);
        RightFrontMotor.configPeakCurrentDuration(80, 0);
        RightFrontMotor.enableCurrentLimit(true);

  }

  public void setLeftSpeed(double speed) {
    LeftFrontMotor.set(ControlMode.PercentOutput, -speed);
}

public void setRightSpeed(double speed) {
  RightFrontMotor.set(ControlMode.PercentOutput, speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Drive.slowSpeed = lol2.getDouble(1);

  }
}
