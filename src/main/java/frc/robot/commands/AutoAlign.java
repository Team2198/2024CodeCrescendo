// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSub;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */

  private DriveSub drive; 
  private double robotTurnGoal; 

  public AutoAlign(DriveSub driveRobot) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.drive = driveRobot;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double limelightData = drive.getLimelight();
    this.robotTurnGoal =  drive.getHeading() + limelightData;

    SmartDashboard.putNumber("LimelightX", limelightData);
    SmartDashboard.putNumber("Robot turn goal", robotTurnGoal);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean turnState = drive.turnToAngle(robotTurnGoal);
    SmartDashboard.putBoolean("Robot turned goal yet", turnState);
    
    return turnState;
  }
}
