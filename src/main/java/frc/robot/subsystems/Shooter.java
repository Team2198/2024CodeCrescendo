// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax shooterMotorOne = new CANSparkMax(12, MotorType.kBrushless);
  CANSparkMax shooterMotorTwo = new CANSparkMax(13, MotorType.kBrushless);

  CANSparkMax hoodMotor = new CANSparkMax(0, MotorType.kBrushless);
  RelativeEncoder hoodAngle = hoodMotor.getEncoder();
  PIDController hoodController = new PIDController(0, 0, 0);
  public Shooter() {
    shooterMotorOne.setSmartCurrentLimit(40);
    shooterMotorTwo.setSmartCurrentLimit(40);
    shooterMotorOne.burnFlash();
    shooterMotorTwo.burnFlash();
    
    //shooterMotorOne.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  

  public void shootMotor(double speed){
    shooterMotorOne.set(-speed);
    shooterMotorTwo.set(speed);
    
  }

  public void setHoodAngle(double angle){
    hoodMotor.setVoltage(hoodController.calculate(getHoodAngle(), angle));
    SmartDashboard.putNumber("angle voltage app", hoodController.calculate(getHoodAngle(), angle));
    
  }

  public void setHood(double voltage){
    hoodMotor.set(voltage);
  }

  public double getHoodAngle(){
    SmartDashboard.putNumber("hood angle", hoodAngle.getPosition()/80);
    return hoodAngle.getPosition()/80;
  }

  
}
