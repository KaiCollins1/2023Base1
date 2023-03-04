// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControllerSubsystem extends SubsystemBase {
  /** Creates a new ControllerSubsystem. */
  public ControllerSubsystem() {}

  private XboxController controller = new XboxController(0);

  public double controllerX(){
    return controller.getRightX();
  }
  public double controllerY(){
    return controller.getLeftY();
  }
  public boolean getUpDpad(){
    return (controller.getPOV() == 0 ? true : false);
  }
  public boolean getDownDpad(){
    return (controller.getPOV() == 180 ? true : false);
  }
  public boolean getAButton(){
    return controller.getAButton();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("contX", controllerX());
    SmartDashboard.putNumber("contY", controllerY());
  }
}
