// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.allyGator.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.allyGator.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  public final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public final MedianFilter warningFilter = new MedianFilter(DriveConstants.kMedianFilterRange);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(
    new Spark(DriveConstants.leftBackPort),
    new Spark(DriveConstants.leftFrontPort)
  );

  // The motors on the right side of the drive.
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(
    new Spark(DriveConstants.rightBackPort),
    new Spark(DriveConstants.rightFrontPort)
  );

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(){
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward
    rightMotors.setInverted(true);

    zeroHeading();

    //TODO test if setting the adjustment to the first reading of the gyro leads to more consistancy
    gyro.setAngleAdjustment(DriveConstants.kAngleOffset);
  }

  public void zeroHeading(){
    gyro.reset();
  }

  //gets angle, and also accounts for weird values using the MedianFilter angleFilter
  //modulus by 360 becasue we don't care if the robot has done 30 clockwise rotations
  public double getAngle(){
    return (gyro.getAngle() % 360) * (DriveConstants.kGyroReversed ? -1 : 1);
  }

  /*
  because of the orientation of the gyro, the actual pitch from the robots perspective is the roll from the gyro's perspecitve
  this is why I call the method getPitch(), because you're getting the pitch of the robot
  but I use gyro.getRoll() because the gyro reads it as roll
  search up how to determine which is roll, pitch, and yaw on the navX documentation.
  */
  public double getPitch(){
    return (gyro.getRoll()+DriveConstants.kPitchOffset)*(DriveConstants.kGyroReversed ? -1 : 1);
  }

  //climbing ChSt? then this returns true
  public boolean climbingChSt(){
    return Math.abs(getPitch()) > 10;
  }
  public boolean onFloor(){
    return Math.abs(getPitch()) < 0.75;
  }

  //Teleop Commands
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot, DoubleSupplier slowDown){
    /*
    gets a doublesupplier to constantly be able to update values.
    Inverts both fwd and rot becasue of x y inversion on sticks

    Slowdown is applied via a scalar. 
    slowDown is 0.0 to 1.0 (the right trigger), and then gets scaled to 25%
    this is then subtracted from 1 to get a value of 1.0 for no press, and 0.75 for full press
    this allows the driver to make the robot drive slower when required for precision/ChSt scaling
    */
    return run(
      () -> m_drive.arcadeDrive(
        -(1 - (.25*slowDown.getAsDouble())) * fwd.getAsDouble(),
        -DriveConstants.kMaxTurnSpeed*rot.getAsDouble()
      )
    ).withName("arcadeDrive");

  }

  //Auton Commands
  public CommandBase autonDriveCommand(double speed, double angle, double timeout){
    PIDController controller = new PIDController(0.4, 0.02, 0.055);

    //allows the driving to account for an angle mistake, or to turn to a specific angle
    return run(
      ()->m_drive.arcadeDrive(
        speed,
        MathUtil.clamp(-0.2 * controller.calculate(getAngle(), angle), -0.8, 0.8)
      )
    ).withTimeout(timeout).withName("autonDrive");
  }

  //drives untill ya hit the ChSt then wait for one second to let the ChSt chill then drive up it a bit
  public CommandBase tiltChStCommnad(boolean goingReverse){
    return autonDriveCommand(
      0.75 * (goingReverse ? -1 : 1), 
      0, 
      10
    ).until(
      ()->climbingChSt()
    ).withTimeout(5)
    .andThen(pauseCommand(1))
    .andThen(autonDriveCommand(0.75 * (goingReverse ? -1 : 1), 0, 1))
    .withName("tiltChSt");
  }

  public CommandBase pauseCommand(double time){
    return run( () -> m_drive.arcadeDrive(0,0)).withTimeout(time);
  }

  public CommandBase engageChStCommand(boolean goingReverse){
    PIDController controller = new PIDController(0.1, 0, 0.01);
    /*
    sets the controller to only consider itself at the goal
    when the position is within xx degrees of the goal
    and the velocity is less than xx degrees/sec
    */
    controller.setTolerance(1, .2);

    return 
    tiltChStCommnad(goingReverse)
    .andThen(
      autonDriveCommand(
        MathUtil.clamp(controller.calculate(getAngle(), 0), -0.6, 0.6),
        0, 
        15)
      .until(controller::atSetpoint).withName("enableChSt")
    );
  }

  public CommandBase chStMobilityCommand(boolean goingReverse){
    return tiltChStCommnad(goingReverse)
    .withTimeout(5)
    .andThen(
      autonDriveCommand(.4 * (goingReverse ? -1: 1), 0, 10)
      .until(() -> onFloor())
    )
    .andThen(autonDriveCommand(.4 * (goingReverse ? -1: 1), 0, .5)
    );
  }
  
  @Override
  public void periodic(){
    //update telemetry
    SmartDashboard.putNumber("angle", getAngle());
    SmartDashboard.putNumber("rate", gyro.getRate());
    SmartDashboard.putNumber("pitch", getPitch());
    SmartDashboard.putBoolean("rateAbove", warningFilter.calculate(Math.abs(gyro.getRate())) > 0.5 ? true : false);
  }
}