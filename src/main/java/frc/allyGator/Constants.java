// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.allyGator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    //differential drive
    public static final int rightBackPort = 2;
    public static final int rightFrontPort = 5;
    public static final int leftBackPort = 3;
    public static final int leftFrontPort = 4;
    public static final double kMaxTurnSpeed = .8;
    
    //gyro
    public static final double kPitchOffset = 0;
    public static final double kAngleOffset = 0;
    public static final boolean kGyroReversed = false;
    public static final int kMedianFilterRange = 5;
    public static final double kP = .1;
    public static final double kPitchTolerance = 1;
    //public static final double kTurnToleranceDeg = 5;
    //public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    public static final double kEnabledP = .05;
  }

  public static final class ArmConstants{
    public static final int klowerLimitSwitchPort = 2;
    public static final int kArmMotorPort = 0;
    public static final int kArmInverted = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double kAutonDelay = 1.5;

}
