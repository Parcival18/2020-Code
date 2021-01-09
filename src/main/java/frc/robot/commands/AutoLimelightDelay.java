
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class AutoLimelightDelay extends SequentialCommandGroup {
  public double Delay = SmartDashboard.getNumber("Lime Delay", 0);
  public AutoLimelightDelay(Launcher launcher, AutoIntake autoIntake, DriveTrain drive, LimeLight limey) {
    addCommands(
        new WaitCommand(Delay),

        new InstantCommand(()-> limey.setPipeline(1)),

        new RunCommand(drive::LimeLightAim, drive)
          .withTimeout(2),

        new InstantCommand(launcher::AMidShoot, launcher),
          new WaitCommand(3),

        new RunCommand(autoIntake::SuckIt, autoIntake)
          .withTimeout(3),

        new RunCommand(() -> drive.teleopDrive(-.5, 0), drive)
          .withTimeout(1)
    );
  }
}