// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSub;
import edu.wpi.first.math.controller.PIDController;
public class turningCommand extends Command {
  /** Creates a new turningCommand. */
  DriveSub drive;
  double goal = 0;
  
  public turningCommand(DriveSub driveS) {
    
    drive = driveS;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goal = drive.getHeading()+90;
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.robotRelative(0, 0, 0);
    
  }

  @Override
  public boolean isFinished(){
    return drive.turnToAngle(goal);
  }

  // Returns true when the command should end.
  
}
