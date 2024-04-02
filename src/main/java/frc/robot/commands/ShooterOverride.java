// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class ShooterOverride extends Command {
  /** Creates a new intakeOverride. */
  Shooter shooter;
  DoubleSupplier speed;
  BooleanSupplier on;
  DoubleSupplier rollSpeed;
  DoubleSupplier speed2;
  DoubleSupplier speed3;
  public ShooterOverride(Shooter shooter, DoubleSupplier speed, DoubleSupplier speed2, DoubleSupplier speed3,BooleanSupplier on) {
    this.shooter = shooter;
    this.speed = speed;
    this.on = on;
    this.speed3 = speed3;
    this.speed2 = speed2;
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
    
    if (Math.abs(speed.getAsDouble())>0.08){
      shooter.setHood(speed.getAsDouble()*-12);
    }
    else{
      shooter.setHood(0);
    }
    
    shooter.setIndexer(speed2.getAsDouble()*-12);
    shooter.shootMotor(speed3.getAsDouble()*12);
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
