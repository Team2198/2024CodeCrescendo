// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionHelper extends SubsystemBase {
  /** Creates a new VisionHelper. */
  public static Pose2d visionPose;
  public static Pose2d robotPose;
  public static double currentNoteYaw; 
  
  public VisionHelper() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public static void setVisionPose(Pose2d pose){
    visionPose = pose;
  }

  public static void setRobotPose(Pose2d pose){
    robotPose = pose;
  }

  public static Pose2d getVisionPose(){
    return visionPose;
  }

  public static Pose2d getRobotPose(){
    return robotPose;
  }

  public static double getNoteYaw(){
    return currentNoteYaw;
  }
  
  public static void updateNoteYaw(double currentYaw){
    currentNoteYaw = currentYaw;
  }






}
