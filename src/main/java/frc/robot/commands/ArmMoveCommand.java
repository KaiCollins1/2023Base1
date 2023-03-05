// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveCommand extends CommandBase {

  ArmSubsystem m_armSubsystem;
  /** Creates a new ArmMoveCommand. */
  public ArmMoveCommand(ArmSubsystem armSubsystem) {
    m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if(m_armSubsystem.m_controller.getPOV() == 180 ? true : false){
        m_armSubsystem.setArmSpeed(-.2 * ArmConstants.kArmInverted);
      }else if(m_armSubsystem.m_controller.getPOV() == 0 ? true : false){
        m_armSubsystem.setArmSpeed(.35 * ArmConstants.kArmInverted);
      }else {
        m_armSubsystem.setArmSpeed(.17);
      }

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
