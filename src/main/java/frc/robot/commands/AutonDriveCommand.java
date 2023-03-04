// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class AutonDriveCommand extends CommandBase {
  /** Creates a new AutonDriveCommand. */
  DriveSubsystem m_driveSubsystem;
  GyroSubsystem m_gyroSubsystem;
  double m_fwd;
  double m_rot;
  double m_time;
  double m_angChange;
  Timer m_timer = new Timer();

  public AutonDriveCommand(DriveSubsystem driveSubsystem, GyroSubsystem gyroSubsystem, double fwd, double rot, double time) {
    m_driveSubsystem = driveSubsystem;
    m_gyroSubsystem = gyroSubsystem;
    m_fwd = fwd;
    m_rot = rot;
    m_time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem, m_gyroSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_rot == 0){
      m_angChange = m_gyroSubsystem.getAngle();
      m_driveSubsystem.arcadeDrive(m_fwd, 0);
      if(Math.abs(m_gyroSubsystem.getAngle()-m_angChange) > .5){
        m_driveSubsystem.arcadeDrive(0, m_gyroSubsystem.getAngle()-m_angChange > 0 ? .2 : -.2);
      }
    }else {
      m_driveSubsystem.arcadeDrive(m_fwd, m_rot);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.get() > m_time){
      return true;
    }else{
      return false;
    }
  }
}
