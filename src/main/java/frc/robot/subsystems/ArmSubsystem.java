// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  DigitalInput lowerLimit = new DigitalInput(ArmConstants.klowerLimitSwitchPort);
  DigitalInput upperLimit = new DigitalInput(ArmConstants.kupperLimitSwitchPort);
  Spark armMotor = new Spark(ArmConstants.kArmMotorPort);
  double armSpeed = 0;

  SlewRateLimiter armLimiter = new SlewRateLimiter(.7);

  public ArmSubsystem() {
  }

  public boolean lowerSwitched(){
    return lowerLimit.get();
  }

  public boolean upperSwitched(){
    return upperLimit.get();
  }

  public void setArmSpeed(double speed){
    armSpeed = armLimiter.calculate(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armMotor.set(armSpeed);
    SmartDashboard.putNumber("ArmSpeed", armSpeed);
    SmartDashboard.putBoolean("UpperLimit", upperSwitched());
    SmartDashboard.putBoolean("LowerLimit", lowerSwitched());
  }

  @Override
  public void setDefaultCommand(Command command) {
      // TODO Auto-generated method stub
      super.setDefaultCommand(command);
  }
  
}
