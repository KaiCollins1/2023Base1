// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.allyGator.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.allyGator.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  public final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public final MedianFilter rateFilter = new MedianFilter(DriveConstants.kMedianFilterRange);
  public final MedianFilter angleFilter = new MedianFilter(DriveConstants.kMedianFilterRange);
  public final MedianFilter pitchFilter = new MedianFilter(DriveConstants.kMedianFilterRange);
  public final MedianFilter yawFilter = new MedianFilter(DriveConstants.kMedianFilterRange);
  public final MedianFilter rollFilter = new MedianFilter(DriveConstants.kMedianFilterRange);

  private final MotorControllerGroup leftMotors =
      new MotorControllerGroup(
          new Spark(DriveConstants.leftBackPort),
          new Spark(DriveConstants.leftFrontPort));

  // The motors on the right side of the drive.
  private final MotorControllerGroup rightMotors =
      new MotorControllerGroup(
          new Spark(DriveConstants.rightBackPort),
          new Spark(DriveConstants.rightFrontPort));

  SlewRateLimiter fwdLimiter = new SlewRateLimiter(1.2);
  SlewRateLimiter rotLimiter = new SlewRateLimiter(1.2);
  

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotors.setInverted(true);
    zeroHeading();
    gyro.setAngleAdjustment(DriveConstants.kAngleOffset);
  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getAngle(){
    return angleFilter.calculate(gyro.getAngle() % 360)*(DriveConstants.kGyroReversed ? -1 : 1);
  }

  public double getPitch(){
    return pitchFilter.calculate(gyro.getRoll()+DriveConstants.kPitchOffset)*(DriveConstants.kGyroReversed ? -1 : 1);
  }

  public boolean climbingChargeStation() {
    return Math.abs(getPitch()) > 10;
  }

  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot){
    return run(
      () -> m_drive.arcadeDrive(
        -DriveConstants.kMaxDriveSpeed*fwd.getAsDouble(),
        -DriveConstants.kMaxDriveSpeed*rot.getAsDouble())).withName("arcadeDrive");
  }

  public CommandBase autonDriveCommand(double speed, double timeout){
    PIDController controller = new PIDController(DriveConstants.kP, 0, 0);
    return run(
      ()->m_drive.arcadeDrive(
        speed,
        -0.2 * controller.calculate(getAngle(), 0)
      )
    ).withTimeout(timeout).withName("autonDrive");
  }

  public CommandBase dockChStationCommnad(double timeout) {
    return autonDriveCommand(
      .75,
      10
    ).until(()->climbingChargeStation()).withTimeout(timeout).withName("dockChStation");
  }
  
  public CommandBase enableChStationCommand(double timeout) {
    PIDController controller = new PIDController(.1, 0, 0);
    controller.setTolerance(DriveConstants.kPitchTolerance, 5);
    return run(
      ()->m_drive.arcadeDrive(
        -.63*controller.calculate(getPitch(), 2.5),
        0
      )
    ).until(controller::atSetpoint).withTimeout(timeout).withName("enableChStation");
  }
  
  public CommandBase autonEnableCommand() {
    return dockChStationCommnad(10).andThen(enableChStationCommand(10)).withName("autonEnableChStationNoScore");
  }
  
  @Override
  public void periodic(){
    SmartDashboard.putNumber("angle", getAngle());
    SmartDashboard.putNumber("rate", gyro.getRate());
    SmartDashboard.putNumber("pitch", getPitch());
  }
}
