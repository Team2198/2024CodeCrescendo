// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.Shooter;
public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  Shooter shooter;
  IntakeSub intake;
  boolean shoot;
  double speed = 0;
  boolean isDone = false;
  boolean amp=false;
  public ShooterCommand(Shooter shooter, IntakeSub intake, boolean shoot) {
    this.shooter = shooter;
    this.intake = intake;
    this.shoot = shoot;
    addRequirements(shooter, intake);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ShooterCommand(Shooter shooter, IntakeSub intake, boolean shoot, boolean isDone) {
    this.shooter = shooter;
    this.intake = intake;
    this.shoot = shoot;
    this.isDone = isDone;
    this.speed = 0.6;

    addRequirements(shooter, intake);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ShooterCommand(Shooter shooter, IntakeSub intake, boolean shoot, boolean isDone, boolean amp) {
    this.shooter = shooter;
    this.intake = intake;
    this.shoot = shoot;
    this.isDone = isDone;
    this.speed = 0.45;
    this.amp = amp;
    addRequirements(shooter, intake);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if (amp){
    shooter.shootMotor(0.38);
   }

   else if (shoot){
    shooter.shootMotor(0.8);
   }
   else{
    intake.shooterPass();
   }
   
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (isDone){
      intake.overrideRoller(0);
      shooter.shootMotor(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
