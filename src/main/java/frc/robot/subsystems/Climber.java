// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax leftMotor = new CANSparkMax(10, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(11, MotorType.kBrushless);
  RelativeEncoder leftEncoder = leftMotor.getEncoder();
  RelativeEncoder rightEncoder = rightMotor.getEncoder();
  public Climber() {
   
   leftMotor.setIdleMode(IdleMode.kBrake);
   rightMotor.setIdleMode(IdleMode.kBrake); 
   leftMotor.setSmartCurrentLimit(40);
   rightMotor.setSmartCurrentLimit(40);
   leftMotor.burnFlash();
   rightMotor.burnFlash();
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  public void setRight(double speed){
    
    rightMotor.set(speed);
  }

  public void setLeft(double speed){
    leftMotor.set(speed);
  }


  
}
