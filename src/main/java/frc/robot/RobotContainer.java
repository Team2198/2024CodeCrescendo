// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCom;
import frc.robot.commands.ParralelAuto;
import frc.robot.commands.intakeOverride;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.turningCommand;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AutoAlign;

//import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  DriveSub drive;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  ParralelAuto autoSegment;
  Command auto;
  IntakeSub intake = new IntakeSub();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //NamedCommands.registerCommand("shooter", new shooterCommand(shoot));
    // Configure the trigger bindings
    drive = new DriveSub();
    autoSegment = new ParralelAuto(drive, shoot);
    auto = Commands.sequence(autoSegment.intakeAndDrive(Units.inchesToMeters(77.8-27), 0));

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
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    //drive.setDefaultCommand(new DriveCommand(drive,()->m_driverController.getLeftY(), ()->m_driverController.getLeftX(), ()->m_driverController.getRightX(), ()->m_driverController.x().getAsBoolean()));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    intake.setDefaultCommand(new intakeOverride(intake, ()->m_driverController.getRightY(), ()->m_driverController.getLeftY()));
    m_driverController.b().whileTrue(new ShooterCommand(shoot));
    m_driverController.y().whileTrue(new IntakeCom(intake, true, false));
    m_driverController.x().whileTrue(new IntakeCom(intake, false, false));
    m_driverController.a().whileTrue(new IntakeCom(intake, false, true));
    m_driverController.rightTrigger().whileTrue(new AutoAlign(drive));

     //m_driverController.a().whileTrue(new IntakeCom(intakee, true, false, false, false));
    
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
    
    return auto;
  }
}
