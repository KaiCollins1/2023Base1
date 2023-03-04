// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  private final MotorControllerGroup leftMotors =
      new MotorControllerGroup(
          new Spark(DriveConstants.leftBackPort),
          new Spark(DriveConstants.leftFrontPort));

  // The motors on the right side of the drive.
  private final MotorControllerGroup rightMotors =
      new MotorControllerGroup(
          new Spark(DriveConstants.rightBackPort),
          new Spark(DriveConstants.rightFrontPort));

  SlewRateLimiter fwdLimiter = new SlewRateLimiter(.5);
  SlewRateLimiter rotLimiter = new SlewRateLimiter(.5);

  private double m_fwd = 0;
  private double m_rot = 0;
  @Override
  public void setDefaultCommand(Command command) {
      // TODO Auto-generated method stub
      super.setDefaultCommand(command);
  }

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotors.setInverted(true);
  }

  public void arcadeDrive(double fwd, double rot) {
    m_fwd = fwd;
    m_rot = rot;
  }

  @Override
  public void periodic(){
    m_drive.arcadeDrive(m_fwd, m_rot);
    SmartDashboard.putNumber("m_fwd", m_fwd);
    SmartDashboard.putNumber("m_rot", m_rot);
  }
}
