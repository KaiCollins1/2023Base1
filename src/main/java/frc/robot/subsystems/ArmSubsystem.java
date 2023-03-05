// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  DigitalInput lowerLimit = new DigitalInput(ArmConstants.klowerLimitSwitchPort);
  DigitalInput upperLimit = new DigitalInput(ArmConstants.kupperLimitSwitchPort);
  Spark armMotor = new Spark(ArmConstants.kArmMotorPort);

  SlewRateLimiter armLimiter = new SlewRateLimiter(.7);

  public XboxController m_controller;
  
  public ArmSubsystem(XboxController controller) {
    m_controller = controller;
  }

  public boolean lowerSwitched(){
    return lowerLimit.get();
  }

  public boolean upperSwitched(){
    return upperLimit.get();
  }

  public CommandBase armDefaultMovementCommand(BooleanSupplier lBumper, DoubleSupplier lTrigger){
    if(lBumper.getAsBoolean()){
      return run(()->armMotor.set(.35)).withName("armUp");
    }else if(lTrigger.getAsDouble() > .5){
      return run(()->armMotor.set(-.2)).withName("armDown");
    }else{
      return run(()->armMotor.set(.17)).withName("armHold");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("UpperLimit", upperSwitched());
    SmartDashboard.putBoolean("LowerLimit", lowerSwitched());
  }
  
}
