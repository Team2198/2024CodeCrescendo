// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class intakeOverride extends Command {
  /** Creates a new intakeOverride. */
  IntakeSub intake;
  DoubleSupplier speed;
  BooleanSupplier on;
  DoubleSupplier rollSpeed;
  public intakeOverride(IntakeSub intake, DoubleSupplier speed, BooleanSupplier on) {
    this.intake = intake;
    this.speed = speed;
    this.on = on;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public intakeOverride(IntakeSub intake, DoubleSupplier speed, DoubleSupplier rollerSpeed) {
    this.intake = intake;
    this.speed = speed;
    this.rollSpeed = rollerSpeed;



    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeOverride(speed.getAsDouble()*-1);
    intake.overrideRoller(rollSpeed.getAsDouble()*-1);
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
