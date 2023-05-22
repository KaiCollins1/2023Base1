// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.allyGator;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.allyGator.subsystems.ArmSubsystem;
import frc.allyGator.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  CommandXboxController m_controller = new CommandXboxController(0);
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  SendableChooser<CommandBase> m_chooser = new SendableChooser<>();

  // private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
  // private GenericEntry autonDelay = tab.add("Auton Delay", 1.5).getEntry();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default commands
    CommandScheduler.getInstance().setDefaultCommand(m_driveSubsystem, m_driveSubsystem.arcadeDriveCommand(
      () -> m_controller.getLeftY(), 
      () -> m_controller.getRightX(),
      () -> m_controller.getRightTriggerAxis()
      ));
    CommandScheduler.getInstance().setDefaultCommand(m_armSubsystem, m_armSubsystem.armDefaultHoldCommand());

    m_controller.leftBumper().whileTrue(m_armSubsystem.armDownCommand());
    m_controller.rightBumper().whileTrue(m_armSubsystem.armUpCommand());

    //sendableChooser here
   
  m_chooser.setDefaultOption(
    "S: Score, Mobility, Turn+armUp", 
    //Drop arm and satisfy motor watchdog for 3 sec
    //drive backwards straight for 2.8 sec at 75% speed
    //turn 180 and then lift arm up
    m_armSubsystem.armDownCommand().alongWith(
      m_driveSubsystem.pauseCommand(Constants.kAutonDelay)
    ).withTimeout(Constants.kAutonDelay).andThen(
      m_driveSubsystem.autonDriveCommand(-0.75, 0, 2.5)
    ).andThen(
      m_driveSubsystem.autonDriveCommand(0, 180, 2).alongWith(
      m_armSubsystem.armUpCommand().withTimeout(2)
    ))
  );

    //score and satisfy motor watchdog
    //tilt chSt
    //climb over chSt
    //rotate 180
    //climb on chSt
    //dock
    //put arm up
    m_chooser.addOption("M: Score, Mobility, Dock",
    m_armSubsystem.armDownCommand().alongWith(
      m_driveSubsystem.pauseCommand(Constants.kAutonDelay)
    ).withTimeout(Constants.kAutonDelay)
    .andThen(
      m_driveSubsystem.chStMobilityCommand(true)
    ).andThen(
      m_driveSubsystem.dockChStCommand(true, 180)
    ).andThen(
      m_driveSubsystem.autonDriveCommand(0, 90, 2).alongWith(
      m_armSubsystem.armUpCommand().withTimeout(2)
    ))
  );

    //score and satisfy motor watchdog
    //tilt chSt
    //climb over chSt
    //rotate 180
    //put arm up
  m_chooser.addOption("M: Score, Mobility, Turn+armUp",
    m_armSubsystem.armDownCommand().alongWith(
      m_driveSubsystem.pauseCommand(Constants.kAutonDelay)
    ).withTimeout(Constants.kAutonDelay)
    .andThen(
      m_driveSubsystem.chStMobilityCommand(true)
    ).andThen(
      m_driveSubsystem.autonDriveCommand(0, 180, 2).alongWith(
      m_armSubsystem.armUpCommand().withTimeout(2)
    ))
  );

    //Drop arm and satisfy motor watchdog for 3 sec
    m_chooser.addOption(
      "ANY: Score", 
      m_driveSubsystem.pauseCommand(Constants.kAutonDelay).
      alongWith(m_armSubsystem.armDownCommand()).
      withTimeout(Constants.kAutonDelay).andThen(
        m_driveSubsystem.autonDriveCommand(0, 180, 2).alongWith(
        m_armSubsystem.armUpCommand().withTimeout(2)
      ))
    );

    // m_chooser.addOption("TEST turn 180", 
    // m_driveSubsystem.autonDriveCommand(0, 180, 30).withTimeout(30)
    // );

    // m_chooser.addOption("TEST tiltChSt", m_driveSubsystem.tiltChStCommnad(true, 0));

    // m_chooser.addOption("TEST chStMobility", m_driveSubsystem.chStMobilityCommand(true));
    
    // m_chooser.addOption("TEST Score, engage", 
    //   m_armSubsystem.armDownCommand().alongWith(
    //     m_driveSubsystem.pauseCommand(1.5)
    //   ).withTimeout(1.5)
    //   .andThen(
    //     m_driveSubsystem.engageChStCommand(true, 0)
    //   )
    // );

    // m_chooser.addOption("TEST Score, Mobility, Engage",
    //   m_armSubsystem.armDownCommand().alongWith(
    //     m_driveSubsystem.pauseCommand(1.5)
    //   ).withTimeout(1.5)
    //   .andThen(
    //     m_driveSubsystem.chStMobilityCommand(true)
    //   ).andThen(
    //     m_driveSubsystem.engageChStCommand(true, 180)
    //   )
    // );
    
    //put m_chooser on the dashboard becasue we need to be able to select auton
    SmartDashboard.putData(m_chooser);
  }

  public void calibrate(){
    m_driveSubsystem.zeroHeading();
  }
  
  //this basically doesn't help use but whatever
  public void updateSchedulerTelemetry() {
    SmartDashboard.putData(m_driveSubsystem);
    SmartDashboard.putData(m_armSubsystem);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // return the selected auton command.
    return m_chooser.getSelected();
  }
}
