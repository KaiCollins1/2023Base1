// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  DigitalInput lowerLimit = new DigitalInput(ArmConstants.klowerLimitSwitchPort);
  Spark armMotor = new Spark(ArmConstants.kArmMotorPort);

  public ArmSubsystem() {
    
  }

  public CommandBase armDefaultHoldCommand(){
    // if(lBumper.getAsBoolean()){
    //   return run(()->armMotor.set(.35)).withName("armUp");
    // }else if(rBumper.getAsBoolean()){
    //   return run(()->armMotor.set(-.2)).withName("armDown");
    // }else{
    //   return run(()->armMotor.set(.17)).withName("armHold");
    // }
    return run(()->armMotor.set(.17)).withName("armHold");
  }

  public CommandBase armUpCommand(){
    return run(()->armMotor.set(.35)).withName("armUp");
  }

  public CommandBase armDownCommand(){
    return run(()->armMotor.set(-.2)).withName("armDown");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LowerLimit", lowerLimit.get());
  }
  
}
