package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.testing.AutoDriveTesting;
public class ParralelAuto {
    DriveSub drive;
    Shooter shoot;
    IntakeSub intake;
    public ParralelAuto(DriveSub drive, Shooter shoot, IntakeSub intake){
        this.drive = drive;
        this.shoot = shoot;
        this.intake = intake;
        
    }

    public Command intakeAndDrive(double distance, double angle) {
        return Commands.race(new AutoDrive(drive, distance, angle), new IntakeCom(intake, true, false, true));
        //return new AutoDrive(drive, distance, angle);
    }


    public Command driveBack(double distance, double angle){
        return Commands.parallel(new IntakeCom(intake, false, false), new AutoDrive(drive, distance, angle).withTimeout(3.5));
    }


    public Command shooterAuto(){
        return Commands.sequence(new ShooterCommand(shoot, intake, true), new WaitCommand(0.75),new ShooterCommand(shoot, intake, false), new WaitCommand(1), new ShooterCommand(shoot, intake, false, true));
    }

    public Command shooterAutoTwo(){
        return Commands.sequence(new ShooterCommand(shoot, intake, true), new WaitCommand( 1.2),new ShooterCommand(shoot, intake, false), new WaitCommand(1), new ShooterCommand(shoot, intake, false, true));
    }

    public Command alignShoot(double angle){
        return Commands.parallel(drive.setWheelAngleCommand(angle).withTimeout(3), shooterAuto());

    }

    public Command driveBackTesting(double distance, double angle){
        return Commands.parallel(new IntakeCom(intake, false, false), new AutoDriveTesting(drive, distance, angle).withTimeout(3.5));
    }

    public Command intakeSeq(){
        return Commands.sequence(new IntakeCom(intake, false, false), shooterAutoTwo());
    }

    public Command driveBackTestingTwo(double distance, double angle){
        return Commands.parallel(new AutoDriveTesting(drive, distance, angle).withTimeout(3.5), intakeSeq());
    }


    public Command shooterAutoTesting(){
        return Commands.sequence(new ShooterCommand(shoot, intake, true), new WaitCommand(0.75),new ShooterCommand(shoot, intake, false), new WaitCommand(1), new ShooterCommand(shoot, intake, false, true));
    }

    public Command intakeAndDriveTesting(double distance, double angle) {
        return Commands.race(new AutoDriveTesting(drive, distance, angle),new IntakeCom(intake, true, false, true));
        //return new AutoDrive(drive, distance, angle);
        //new IntakeCom(intake, true, false, true));
    }


}
