// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.lang.Math;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TelopDriveCommand extends CommandBase {

  final DriveSubsystem m_driveSubsystem;
  final ControllerSubsystem m_controllerSubsystem;
  private double m_fwd = 0;
  private double m_rot = 0;

  /** Creates a new TelopDriveCommand. */
  public TelopDriveCommand(DriveSubsystem driveSubsystem, ControllerSubsystem controllerSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_controllerSubsystem = controllerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, controllerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("commandInit?", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //squares input in second term. first term re-multiplies the negative if there was one bc squaring removes the negative
    m_fwd = (m_controllerSubsystem.controllerY() > 0 ? 1 : -1) * (-Math.pow(m_controllerSubsystem.controllerY(), 2));
    m_rot = -(m_controllerSubsystem.controllerX() > 0 ? 1 : -1) * (Math.pow(m_controllerSubsystem.controllerX(), 2));
    m_driveSubsystem.arcadeDrive(m_fwd, m_rot);
    SmartDashboard.putNumber("debugFwd", m_fwd);
    SmartDashboard.putNumber("debugRot", m_rot);
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
