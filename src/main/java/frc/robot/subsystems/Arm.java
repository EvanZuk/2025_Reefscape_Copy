// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final TalonFX arm1 = new TalonFX(100);
  private final TalonFX arm2 = new TalonFX(101);

  public MotorControllerGroup armMotors = new MotorControllerGroup(arm1, arm2);

  public Arm() {
    //armMotors = new TalonFX(100);
    armMotors.setInverted(true);
  }

  public double getPosition() {
    return arm1.getPosition().getValue();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Roller Moter Encoder", rollerMoter.getSelectedSensorPosition());
  }
}
