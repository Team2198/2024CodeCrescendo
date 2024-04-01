// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class IntakeCom extends Command {

  IntakeSub intake;
  boolean extend;
  boolean retractBool = false;
  boolean all;
  boolean override = false;
  boolean auto = false;
  BooleanSupplier manual;
  /** Creates a new sensorCom. */
  public IntakeCom(IntakeSub intakey, boolean extender, boolean override, boolean auto) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakey;
    
    extend = extender;
    this.override = override;
    this.auto = auto;
    manual = ()->false;
    addRequirements(intake);
  }

  public IntakeCom(IntakeSub intakey, boolean extender, boolean override) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakey;
    extend = extender;
    this.override = override;
    manual = ()->false;
    addRequirements(intake);
  
  }

  public IntakeCom(IntakeSub intakey, boolean extender, boolean override, BooleanSupplier manual) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakey;
    extend = extender;
    this.override = override;
    this.manual = manual;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    retractBool = intake.detectNode();
    SmartDashboard.putBoolean("retract bool", retractBool);
    /* if(!extend && !takeIn){
      intakee.retractIntake();
    }
    if(takeIn){
      intakee.takeNode();
    }  */
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOverride(0);
    intake.overrideRoller(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     
    
    if(extend){
        SmartDashboard.putBoolean("in command", true);
      

        boolean finish = intake.extendIntake();
        
        if (auto){
          intake.takeNode();
        }

        else if (manual.getAsBoolean()){
          intake.overrideRoller(0.75);
          return false;
        }

        else if (finish){
          intake.takeNode();
        }

        
        
        if (override){
          return finish;
        }
        return finish&&intake.detectNode();
      

      
      
    }
    else{
      return intake.retractIntake();
    }  
  
  } 
}