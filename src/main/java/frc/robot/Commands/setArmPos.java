// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class setArmPos extends Command {
  /** Creates a new setArmPos. */

  private final Arm arm;
  private final double pos;
  PIDController pid = new PIDController(0.65, 0.4, 0.1);

  public setArmPos(double pos, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.pos = pos;
   addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.armMotors.set(pid.calculate(arm.getPosition(), pos));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
