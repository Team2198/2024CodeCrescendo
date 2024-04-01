// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCom;
import frc.robot.commands.ParralelAuto;
import frc.robot.commands.intakeOverride;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterOverride;
import frc.robot.commands.climbCommand;
import frc.robot.commands.colorTest;
import frc.robot.commands.turningCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.Shooter;

//import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.intakeOverride;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Shooter shoot = new Shooter();
  Climber climber = new Climber();  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  DriveSub drive = new DriveSub();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
 private final CommandXboxController m_driverController =
      new CommandXboxController(0);
 
  private final CommandXboxController op_driverController =
      new CommandXboxController(1);
  
  
  IntakeSub intake = new IntakeSub();
  ParralelAuto autoSegment = new ParralelAuto(drive, shoot, intake);
  Command middleOneNote = Commands.sequence(drive.setWheelAngleCommand(0).withTimeout(1),drive.setRobotOffset(0),drive.zeroEncodersCommand(),autoSegment.shooterAuto());
  
  Command driveBack = Commands.sequence(drive.setWheelAngleCommand(0).withTimeout(1),drive.setRobotOffset(0),drive.zeroEncodersCommand(), autoSegment.driveBack(1.397, 0));
  Command middleTwoNote = Commands.sequence(drive.setWheelAngleCommand(0).withTimeout(1),drive.setRobotOffset(0),drive.zeroEncodersCommand(),autoSegment.shooterAuto(), autoSegment.intakeAndDrive(1.397, 0), autoSegment.driveBack(0,0), autoSegment.shooterAuto());
  Command middleTwoNoteLeave = Commands.sequence(drive.setWheelAngleCommand(0).withTimeout(1),drive.setRobotOffset(0),drive.zeroEncodersCommand(),autoSegment.shooterAuto(), autoSegment.intakeAndDrive(1.397, 0), autoSegment.driveBack(0,0), autoSegment.shooterAuto(), autoSegment.driveBackTesting(1.143+0.152, 0));
  //Command middleOneNote = Commands.sequence(drive.setRobotOffset(0), drive.zeroEncodersCommand(),new AutoDrive(drive, Units.inchesToMeters(0.762), 0));
  Command leftSideOneLeave = Commands.sequence(drive.setWheelAngleCommand(120).withTimeout(1),drive.setRobotOffset(60),drive.zeroEncodersCommand(),autoSegment.shooterAuto(),new AutoDrive(drive, -(1.143+0.152), 120));
  Command leftSideOne = Commands.sequence(drive.setRobotOffset(60),drive.zeroEncodersCommand(),autoSegment.shooterAuto());
  Command noDriveMiddle = Commands.sequence(drive.setRobotOffset(0),drive.zeroEncodersCommand(),autoSegment.shooterAuto());
  Command driveBackWall = Commands.sequence(drive.setWheelAngleCommand(0).withTimeout(1),drive.setRobotOffset(0),drive.zeroEncodersCommand(), autoSegment.driveBack(2.083, 0));

  Command rightSideOneLeave = Commands.sequence(drive.setWheelAngleCommand(-120).withTimeout(1),drive.setRobotOffset(-60),drive.zeroEncodersCommand(),autoSegment.shooterAuto(),new AutoDrive(drive, -(1.143+0.152), -120));
  Command rightSideOne = Commands.sequence(drive.setRobotOffset(-60),drive.zeroEncodersCommand(),autoSegment.shooterAuto());
  
  Command middleTwoNoteTesting = Commands.sequence(drive.setRobotOffset(0),autoSegment.alignShoot(0), autoSegment.intakeAndDriveTesting(1.397, 0), autoSegment.driveBack(0,0).withTimeout(2.5));
  Command middleThreeNoteTesting = Commands.sequence(drive.setRobotOffset(0),autoSegment.alignShoot(90),autoSegment.driveBackTesting(Units.inchesToMeters(-62), 90).withTimeout(2.5),drive.setWheelAngleCommand(0).withTimeout(0.1),autoSegment.intakeAndDriveTesting(Units.inchesToMeters(82), 0), drive.setWheelAngleCommand(90).withTimeout(0.1), autoSegment.driveBackTesting(Units.inchesToMeters(64), 90).withTimeout(1.5), drive.setWheelAngleCommand(0).withTimeout(.1), autoSegment.driveBackTestingTwo(Units.inchesToMeters(-78), 0),autoSegment.shooterAuto());
  Command middleThreeNoteTestingRed = Commands.sequence(drive.setRobotOffset(0),autoSegment.alignShoot(90),autoSegment.driveBackTesting(Units.inchesToMeters(62), 90).withTimeout(2.5),drive.setWheelAngleCommand(0).withTimeout(0.1),autoSegment.intakeAndDriveTesting(Units.inchesToMeters(82), 0), drive.setWheelAngleCommand(90).withTimeout(0.1), autoSegment.driveBackTesting(Units.inchesToMeters(-64), 90).withTimeout(1.5), drive.setWheelAngleCommand(0).withTimeout(.1), autoSegment.driveBackTesting(Units.inchesToMeters(-78), 0),autoSegment.shooterAuto());      
  Command middleThreeNote = Commands.sequence(middleTwoNoteTesting, middleThreeNoteTesting);
  Command strafeMiddle = Commands.sequence(drive.setRobotOffset(0),autoSegment.alignShoot(52),autoSegment.intakeAndDriveTesting(Units.inchesToMeters(5), 90));
  //autoSegment.intakeAndDriveTesting(Units.inchesToMeters(73), 52), autoSegment.intakeAndDriveTesting(Units.inchesToMeters(5), 0),autoSegment.driveBackTesting(Units.inchesToMeters(-77),48), autoSegment.alignShoot(0));

  SendableChooser<Command> m_chooser = new SendableChooser<>();          
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */      
  public RobotContainer() {
    m_chooser.setDefaultOption("Middle one note", middleOneNote);
    m_chooser.addOption("middle two note", middleTwoNote);
    m_chooser.addOption("middle two note leave", middleTwoNoteLeave);
    m_chooser.addOption("right side one note leave", rightSideOneLeave);
    m_chooser.addOption("drive back", driveBack);
    m_chooser.addOption("right side one note", rightSideOne);
    m_chooser.addOption("left side one note", leftSideOne);
    m_chooser.addOption("left side one note leave", leftSideOneLeave);  
    m_chooser.addOption("drive back from the wall", driveBackWall);
    m_chooser.addOption("three note auto testing", middleThreeNote);
    m_chooser.addOption("middle three note BLUE", middleThreeNote);
    Shuffleboard.getTab("Auto chooser").add(m_chooser);

    //NamedCommands.registerCommand("shooter", new shooterCommand(shoot));
    // Configure the trigger bindings
    //SmartDashboard.putData("Auto chooser", m_chooser);
    
         
    
    

    //SmartDashboard.putData(m_chooser);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //intake.setDefaultCommand(new intakeOverride(intake, ()->m_driverController.getRightY(), ()->m_driverController.getLeftY()));
    
    //m_driverController.rightBumper().onTrue(new ShooterCommand(shoot, intake,true).andThen(new WaitCommand(0.5)).andThen(new ShooterCommand(shoot, intake, false)).andThen(new WaitCommand(1)).andThen(new ShooterCommand(shoot, intake, false, true)));
    //m_driverController.leftBumper().onTrue(new ShooterCommand(shoot, intake,true, false,true).andThen(new WaitCommand(0.5)).andThen(new ShooterCommand(shoot, intake, false)).andThen(new WaitCommand(1)).andThen(new ShooterCommand(shoot, intake, false, true)));
    
    //intake.setDefaultCommand(new IntakeCom(intake, ()->m_driverController.getHID().getYButton()));
    //m_driverController.leftBumper().onTrue(new AutoDrive(drive,1.2, 0));
    
    //op_driverController.a().onTrue(new ShooterCommand(shoot, intake,true).andThen(new WaitCommand(0.5)).andThen(new ShooterCommand(shoot, intake, false)).andThen(new WaitCommand(1)).andThen(new ShooterCommand(shoot, intake, false, true)));
    //m_driverController.a().whileTrue(new IntakeCom(intake, false, true));
    //m_driverController.rightTrigger().onTrue(new AutoAlign(drive, false));
    //m_driverController.leftTrigger().onTrue(new AutoAlign(drive, true));
    //m_driverController.a().whileTrue(new IntakeCom(intakee, true, false, false, false));
   // climber.setDefaultCommand(new climbCommand(climber, ()->m_driverController.getLeftY(), ()->m_driverController.getRightY()));
    //m_driverController.rightTrigger().onTrue(intake.extendIntakeOverride());
    //drive.setDefaultCommand(new DriveCommand(drive,()->m_driverController.getLeftY(), ()->m_driverController.getLeftX(), ()->m_driverController.getRightX(), ()->m_driverController.getHID().getAButton()));
    //m_driverController.b().onTrue(new ShooterCommand(shoot, intake,true, false,true).andThen(new WaitCommand(0.5)).andThen(new ShooterCommand(shoot, intake, false)).andThen(new WaitCommand(1)).andThen(new ShooterCommand(shoot, intake, false, true)));
    //m_driverController.x().toggleOnTrue(new IntakeCom(intake, false, false));
    
    //m_driverController.y().toggleOnTrue(new IntakeCom(intake, true,false).andThen(new IntakeCom(intake,false,false)));
    //m_driverController.leftTrigger().onTrue(intake.stuckNote());
    //m_driverController.y().toggleOnTrue(new IntakeCom(intake, true,false).andThen(new IntakeCom(intake,false,false)));
    
    //m_driverController.a().onTrue(new colorTest(intake, true, false));
    //m_driverController.b().onTrue(new colorTest(intake, false, true));
    //intake.setDefaultCommand(new intakeOverride(intake, ()->m_driverController.getRightY(), ()->m_driverController.getLeftY()));
    //actual buttons
    //m_driverController.a().onTrue(drive.visionReset());
    shoot.setDefaultCommand(new ShooterOverride(shoot, ()->m_driverController.getLeftY(), ()->true));
    //drive.setDefaultCommand(new DriveCommand(drive,()->m_driverController.getLeftY(), ()->m_driverController.getLeftX(), ()->m_driverController.getRightX(), ()->m_driverController.getHID().getAButton(), ()->op_driverController.getHID().getLeftBumper()));
    //drive.setDefaultCommand(new DriveCommand(drive,()->m_driverController.getLeftY(), ()->m_driverController.getLeftX(), ()->m_driverController.getRightX(), ()->m_driverController.getHID().getAButton(), ()->op_driverController.getHID().getLeftBumper()));
    /* m_driverController.b().whileTrue((new IntakeCom(intake, true, true, ()->true)));
    m_driverController.b().onFalse(new IntakeCom(intake, false, false));
    m_driverController.x().toggleOnTrue(new IntakeCom(intake, false, false));
    drive.setDefaultCommand(new DriveCommand(drive,()->m_driverController.getLeftY(), ()->m_driverController.getLeftX(), ()->m_driverController.getRightX(), ()->m_driverController.getHID().getAButton(), ()->op_driverController.getHID().getLeftBumper()));
    m_driverController.y().toggleOnTrue(new IntakeCom(intake, true,false).andThen(new IntakeCom(intake,false,false)));
    m_driverController.leftBumper().onTrue(new turningCommand(drive, 60));
    m_driverController.rightBumper().onTrue(new TurningCommand(drive, -60));
    m_driverController.leftTrigger().onTrue(intake.stuckNote());
    op_driverController.a().onTrue(new ShooterCommand(shoot, intake,true).andThen(new WaitCommand(0.9)).andThen(new ShooterCommand(shoot, intake, false)).andThen(new WaitCommand(1.3)).andThen(new ShooterCommand(shoot, intake, false, true)));
    op_driverController.b().onTrue(new ShooterCommand(shoot, intake,true, false,true).andThen(new WaitCommand(0.5)).andThen(new ShooterCommand(shoot, intake, false)).andThen(new WaitCommand(1)).andThen(new ShooterCommand(shoot, intake, false, true)));
    op_driverController.rightBumper().onTrue(intake.extendIntakeOverride());
    climber.setDefaultCommand(new climbCommand(climber, ()->op_driverController.getLeftY(), ()->op_driverController.getRightY()));
    op_driverController.y().onTrue(intake.reverseNote());
    
    op_driverController.rightTrigger().whileTrue(intake.shooterStuck());
    op_driverController.rightTrigger().onFalse(intake.overideRollerCommand(0)); */
             

    
  }       
   
  /**
   * .      
   *      
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new PathPlannerAuto("New Auto");
    
    //return auto;  
    return m_chooser.getSelected();
    //return middleTwoNote;
  }
}
