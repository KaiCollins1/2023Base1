// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.allyGator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.allyGator.subsystems.ArmSubsystem;
import frc.allyGator.subsystems.DriveSubsystem;

/** Add your docs here. */
public class AutonRoutines {
    private static DriveSubsystem m_driveSubsystem;
    private static ArmSubsystem m_armSubsystem;

    void AutonRoutines(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem){
        m_driveSubsystem = driveSubsystem;
        m_armSubsystem = armSubsystem;
    }

    public static CommandBase scoreMobilityTurnArmUpCommand() {
        return m_armSubsystem.armDownCommand().alongWith(
        m_driveSubsystem.pauseCommand(3)
      ).withTimeout(3).andThen(
        m_driveSubsystem.autonDriveCommand(-0.75, 0, 2.8)
      ).andThen(
        m_driveSubsystem.autonDriveCommand(.1, 180, 5).alongWith(
        m_armSubsystem.armUpCommand().withTimeout(5)
      ));
    }
}
