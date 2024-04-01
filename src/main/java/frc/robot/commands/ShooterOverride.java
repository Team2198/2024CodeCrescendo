// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.Shooter;

public class ShooterOverride extends Command {
  /** Creates a new intakeOverride. */
  Shooter shooter;
  DoubleSupplier speed;
  BooleanSupplier on;
  DoubleSupplier rollSpeed;
  public ShooterOverride(Shooter shooter, DoubleSupplier speed, BooleanSupplier on) {
    this.shooter = shooter;
    this.speed = speed;
    this.on = on;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

  

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.intakeOverride(speed.getAsDouble()*-1);
    //intake.overrideRoller(rollSpeed.getAsDouble()*-1);
    shooter.setHood(speed.getAsDouble()*12);
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
