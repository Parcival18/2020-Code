package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.*;


public class Baila extends SequentialCommandGroup {
    public Baila(Launcher launcher, AutoIntake autoIntake, DriveTrain drive, LimeLight limey) {
    addCommands(
        new InstantCommand(launcher::AMidShoot, launcher),

        new InstantCommand(drive::zeroHeading, drive),

        new InstantCommand(()-> limey.setPipeline(1)),

        new RunCommand(drive::LimeLightAim, drive)
          .withTimeout(2),

        new InstantCommand(autoIntake::GoDown, autoIntake),

        new RunCommand(autoIntake::SuckIt, autoIntake)
            .withTimeout(3),

        new RunCommand(()-> drive.TurnAngleAuto(90))
            //.beforeStarting(drive::zeroHeading, drive)
            .withTimeout(1.4),

        new RunCommand(()-> drive.TurnAngleAuto(90))
            .beforeStarting(drive::zeroHeading, drive)
            .withTimeout(1.4),

        new InstantCommand(autoIntake::IntakeIt, autoIntake),
        
        new RunCommand(()-> drive.DriveDistance(.7), drive)
            .beforeStarting(drive::zeroHeading, drive)
            .beforeStarting(drive::resetEncoders, drive)
            .withTimeout(2),

        new InstantCommand(autoIntake::StopIt, autoIntake),

        new InstantCommand(autoIntake::GoUp, autoIntake),

        new RunCommand(()-> drive.TurnAngleAuto(90), drive)
            .beforeStarting(drive::zeroHeading, drive)
            .withTimeout(2),

        new RunCommand(()-> drive.TurnAngleAuto(90), drive)
            .beforeStarting(drive::zeroHeading, drive)
            .withTimeout(2),

        new RunCommand(drive::LimeLightAim, drive)
            .withTimeout(1),
        
        new RunCommand(autoIntake::SuckIt, autoIntake)
    );
  }
}