// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeSub;

public class climbCommand extends Command {
  /** Creates a new intakeOverride. */
  Climber climber;
  DoubleSupplier speed;
  
  DoubleSupplier speedTwo;  
  public climbCommand(Climber climber, DoubleSupplier speed, DoubleSupplier speedTwo) {
    this.climber = climber;
    this.speed = speed;
    this.speedTwo = speedTwo;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

  

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftDub = speed.getAsDouble();
    double rightDub = speedTwo.getAsDouble();

    if (Math.abs(leftDub)<0.12){
      leftDub = 0;

    }

    if (Math.abs(rightDub)<0.12){
      rightDub = 0;

    }



    climber.setLeft(leftDub);
    climber.setRight(-rightDub);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
