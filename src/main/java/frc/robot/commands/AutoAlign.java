// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSub;

public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */

  private DriveSub drive; 
  private double robotTurnGoal; 

  public AutoAlign(DriveSub driveRobot) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.drive = driveRobot;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double limelightData = drive.getLimelight();
    this.robotTurnGoal =  drive.getHeading() + limelightData;

    SmartDashboard.putNumber("LimelightX", limelightData);
    SmartDashboard.putNumber("Robot turn goal", robotTurnGoal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    drive.robotRelative(0, 0, 0);
    boolean turnState = drive.atSetpoint();
    SmartDashboard.putBoolean("Robot turned goal yet", turnState);

    return turnState;
  }
}
