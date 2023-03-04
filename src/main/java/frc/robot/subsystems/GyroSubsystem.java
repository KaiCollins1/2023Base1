// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;

import com.kauailabs.navx.frc.AHRS;

public class GyroSubsystem extends SubsystemBase {
  public final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public final MedianFilter rateFilter = new MedianFilter(DriveConstants.kMedianFilterRange);
  public final MedianFilter angleFilter = new MedianFilter(DriveConstants.kMedianFilterRange);
  public final MedianFilter pitchFilter = new MedianFilter(DriveConstants.kMedianFilterRange);
  public final MedianFilter yawFilter = new MedianFilter(DriveConstants.kMedianFilterRange);
  public final MedianFilter rollFilter = new MedianFilter(DriveConstants.kMedianFilterRange);
  /** Creates a new GyroSubsystem. */
  public GyroSubsystem() {
    zeroHeading();
  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getRate(){
    return rateFilter.calculate(gyro.getRate())*(Constants.DriveConstants.kGyroReversed ? -1 : 1);
  }

  public double getAngle(){
    return angleFilter.calculate(gyro.getAngle() % 360)*(Constants.DriveConstants.kGyroReversed ? -1 : 1);
  }

  public double getPitch(){
    return pitchFilter.calculate(gyro.getPitch())*(Constants.DriveConstants.kGyroReversed ? -1 : 1);
  }
  public double getRoll(){
    return rollFilter.calculate(gyro.getRoll())*(Constants.DriveConstants.kGyroReversed ? -1 : 1);
  }
  public double getYaw(){
    return yawFilter.calculate(gyro.getYaw())*(Constants.DriveConstants.kGyroReversed ? -1 : 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("angle", getAngle());
    SmartDashboard.putNumber("yaw", getYaw());
    SmartDashboard.putNumber("roll", getRoll());
    SmartDashboard.putNumber("pitch", getPitch());
    SmartDashboard.putNumber("rate", getRate());
  }
}
