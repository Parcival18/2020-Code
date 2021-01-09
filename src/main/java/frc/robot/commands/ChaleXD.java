
package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.*;

import frc.robot.Constants.*;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class ChaleXD extends SequentialCommandGroup {
  DifferentialDriveKinematics DifferentialDriveKinematics = new DifferentialDriveKinematics(
      0.6541986514531234);

  public DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(0, 0), DifferentialDriveKinematics, 10);
  
  final TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

  final Trajectory bruhTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, -0, new Rotation2d(0)), //It's in radians-- degree value * pi/180
    List.of(
        new Translation2d(.5, 0)
    ),
    new Pose2d(1, 0, new Rotation2d(38)),
    //2.0, -1.0
    // Pass config
    config
);
  public ChaleXD(Launcher launcher, AutoIntake autoIntake, DriveTrain drive, LimeLight limey) {
    addCommands(

        new InstantCommand(()-> limey.setPipeline(1)),

        new RunCommand(drive::LimeLightAim, drive)
          .withTimeout(2),

        new InstantCommand(launcher::AMidShoot, launcher),
          new WaitCommand(2),

        new RunCommand(autoIntake::SuckIt, autoIntake)
          .withTimeout(3),
        
        new InstantCommand(autoIntake::GoDown, autoIntake),
          new WaitCommand(1),

        new InstantCommand(autoIntake::IntakeIt, autoIntake),
          new WaitCommand(1),

        //new InstantCommand(drive::resetOdometry, drive),

        new RamseteCommand(
        bruhTrajectory, 
        drive::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter), 
        DriveConstants.kDriveKinematics, 
        drive::getWheelSpeeds, 
        new PIDController(DriveConstants.kPDriveVel, 0, 0), 
        new PIDController(DriveConstants.kPDriveVel, 0, 0), 
        drive::tankDriveVolts, 
        drive)
    );
  }
}