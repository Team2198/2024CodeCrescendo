// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkFlex shooterMotorOne = new CANSparkFlex(6, MotorType.kBrushless);
  CANSparkFlex shooterMotorTwo = new CANSparkFlex(7, MotorType.kBrushless);
  CANSparkFlex indexer = new CANSparkFlex(13, MotorType.kBrushless);
  CANSparkFlex hoodMotor = new CANSparkFlex(5, MotorType.kBrushless);
  RelativeEncoder hoodAngle = hoodMotor.getEncoder();
  PIDController hoodController = new PIDController(0.27, 0, 0);
  public Shooter() {
    hoodMotor.setInverted(true);
    indexer.setInverted(true);
    shooterMotorOne.setSmartCurrentLimit(40);
    shooterMotorTwo.setSmartCurrentLimit(40);
    shooterMotorOne.burnFlash();
    shooterMotorTwo.burnFlash();
    hoodAngle.setPosition(0);
    hoodController.setTolerance(1);
    //shooterMotorOne.setInverted(true);
  }

  @Override
  public void periodic() {
    getHoodAngle();
    //setHoodAngle(90);
    // This method will be called once per scheduler run
  }

  
  

  

  public void shootMotor(double voltage){
    //indexer.setVoltage(voltage);
    shooterMotorOne.setVoltage(voltage);
    shooterMotorTwo.setVoltage(voltage);
    //shooterMotorOne.set(-speed);
    //shooterMotorTwo.set(speed);
    
  }

  public void setHoodAngle(double angle){
    hoodMotor.setVoltage(hoodController.calculate(getHoodAngle(), angle)*12);
    SmartDashboard.putNumber("angle voltage app", hoodController.calculate(getHoodAngle(), angle));
    SmartDashboard.putBoolean("hood angle ok", hoodController.atSetpoint());
  }

  public double getHoodAngle(){
    SmartDashboard.putNumber("hood angle", hoodAngle.getPosition()/320*360);
    return hoodAngle.getPosition()/320*360;
  }

  public void setHood(double voltage){
    hoodMotor.setVoltage(voltage);
  }

  public void setIndexer(double voltage){
    indexer.setVoltage(voltage);
  }
  public Command runIndexer(DoubleSupplier speed){
    return this.run(()->this.indexer.set(speed.getAsDouble()));
  }

  public Command setShooterAngle(double angle){
    return this.run(()->this.setHoodAngle(angle));
  }

  public Command overrideHood(double speed){
    return this.runOnce(()->this.setHood(0));
  }
  
}
