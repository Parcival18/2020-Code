
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.*;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class AutoLimelightFordward extends SequentialCommandGroup {
  public AutoLimelightFordward(Launcher launcher, AutoIntake autoIntake, DriveTrain drive, LimeLight limey) {
    addCommands(

        new InstantCommand(()-> limey.setPipeline(1)),

        new RunCommand(drive::LimeLightAim, drive)
          .withTimeout(2),

        new InstantCommand(launcher::AMidShoot, launcher),
          new WaitCommand(3),

        new RunCommand(autoIntake::ASuckIt, autoIntake)
          .withTimeout(2.5),

        new RunCommand(() -> drive.teleopDrive(.5, 0), drive)
          .withTimeout(3)
    );
  }
}