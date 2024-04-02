// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
public class IntakeSub extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  
  boolean nodeIn = false;
  
  //ColorMatch colorMatcher = new ColorMatch();s
  //Color NodeColor = new Color(0.92, 0.203, 0.91);
  Color detectedColor;
  double distance;

  Spark led = new Spark(0);
 // PWMSparkMax led2 = new PWMSparkMax(0);
  CANSparkMax m_arm = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax m_intake = new CANSparkMax(8, MotorType.kBrushless);
  RelativeEncoder myEncoder = m_arm.getEncoder();
  DigitalInput beamBreak = new DigitalInput(0);
  double beamCounter = 0;
  double counter = -0.99;

  /** Creates a new colorSensor. */
  public IntakeSub() {
    
    m_arm.setSmartCurrentLimit(40);
    m_intake.setSmartCurrentLimit(40);
    m_intake.burnFlash();
    m_arm.burnFlash();
    //colorMatcher.addColorMatch(NodeColor);
    myEncoder.setPosition(0);
    

    
    
  }



  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //nodeVisible();
    //led.set(0.93);   
    maintainBeamBreak();  
    SmartDashboard.putBoolean("beam break detected", !beamBreak.get());          
    SmartDashboard.putBoolean("node in 2", detectNode());
    if (!detectNode() && !nodeVisible()){
     led.set(-0.85);
    }
    SmartDashboard.putNumber("beam counter", beamCounter);

    

    //nodeVisible();
    //nodeVisible();
    /* SmartDashboard.putNumber("led voltage", led.get());
    SmartDashboard.putBoolean("node in: ", nodeIn);
    SmartDashboard.putNumber("Detected red 2: ", mySensor.getRed());
    SmartDashboard.putNumber("Detected green: ", mySensor.getGreen());
    SmartDashboard.putNumber("Detected blue: ", mySensor.getBlue());
    
    SmartDashboard.putBoolean("Detected sensor", mySensor.isConnected()); */
    //nodeIndicator();
     
    SmartDashboard.putNumber("angle", getAngle());   
    //led.set(-0.83);
    SmartDashboard.putNumber("current color", counter);
  }     


  public void maintainBeamBreak(){
    if (!beamBreak.get()){
      beamCounter+=1;
      
      SmartDashboard.putNumber("last break", beamCounter);
      
      //SmartDashboard.putBoolean("node in 2", false);
    }

    else{
      
      beamCounter=0;
    }
  }

  public boolean detectNode(){

            
    //distance = mySensor.getProximity();          
    //(mySensor.getRed() > mySensor.getBlue() && mySensor.getRed()>mySensor.getGreen()){
    
      
    if (beamCounter >= 3){
      SmartDashboard.putBoolean("node in 2", true);
      return true;
    }
      
      
      
      
      
    else{         
      
                                         
      SmartDashboard.putBoolean("node in 2", false);
      
      return false;     
    }   
    //return false;

  } 

  public boolean nodeVisible(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    double nodeVisible = tv.getDouble(0);
    if (nodeVisible==1 && !detectNode()){
      led.set(-0.81);
      return true;
    }
    return false;         
  }         

  public void nodeIndicator(){
    /* if(nodeIn){
      blinkinPWM.set(0.77); // green
    }else{
      blinkinPWM.set(0.61); // re d
    } */
  }

  public void setLed(){
    if (detectNode()){
      led.set(-0.83);
    }
  }

  public double getAngle(){
    return (myEncoder.getPosition() / 66)*360;
  }

  public boolean extendIntake(){
    double myAngle = getAngle();
    
    if(myAngle <= 190){
    m_arm.set(0.8);
    
    }
    else if(myAngle <=200){
      m_arm.set(0);
    }
    else{
      m_arm.set(0);
      return true;
    }
    return false;
    
  }


  public boolean extendIntakeTwo(double speed){
    double myAngle = getAngle();
    
    if(myAngle <= 190){
    m_arm.set(speed);
    }
    else if(myAngle <=195){
      m_arm.set(0.1);
    }
    else{
      m_arm.set(0);
      return true;      
    }
    return false;
    
  }

  public void takeNode(){
    
    if(detectNode()){
      
      m_intake.set(0);
      SmartDashboard.putBoolean("detected node",detectNode());  
    }
    else{ 
      m_intake.set(0.75);
    }
    
  }
  public void shooterPass(){
    m_intake.set(-0.9);
  }

  public boolean retractIntake(){
    
    m_intake.set(0);
    setLed();
    if(getAngle() <= 20){
      m_arm.set(0);
      return true;
    } else{          
      
      m_arm.set(-0.5);
      return false;
    } 
  }

  public Command extendIntakeOverride(){
  return this.run(()->this.extendIntake());
  } 

  public void overrideRoller(double d) {
    // TODO Auto-generated method stub
    
   
      m_intake.set(d);
    
     
  }

public void intakeOverride(double d) {
    // TODO Auto-generated method stub
    m_arm.set(d);
}

public Command reverseNote(){
  return this.run(()->this.overrideRoller(-0.8));
  } 

  public Command stuckNote(){
    return this.run(()->this.extendIntakeTwo(0.1));
  }

  public Command climbExtend(){
    return this.run(()->this.extendIntakeTwo(0.8));
  }

  public Command shooterStuck(){
    return this.runOnce(()->this.overrideRoller(0.3));
  }


   public Command overideRollerCommand(double speed){
    return this.runOnce(()->this.overrideRoller(speed));
  }


  public double counter(){
    //counter += 0.01;
    return counter;
  }
  
  public void player(){
    //led.set(counter);
  }
  public void colorWorks(){
    SmartDashboard.putString("working color "+counter, "batee5a");
  }

}