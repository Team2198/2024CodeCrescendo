package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.Shooter;

public class ParralelAuto {
    DriveSub drive;
    Shooter shoot;
    public ParralelAuto(DriveSub drive, Shooter shoot){
        this.drive = drive;
        this.shoot = shoot;
    }

    public Command intakeAndDrive(double distance, double angle) {
        return Commands.parallel(new AutoDrive(drive, distance, angle), new shooterCommand(shoot).withTimeout(2));
    }
}
