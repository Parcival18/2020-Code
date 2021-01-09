package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;

import frc.robot.trajectories.ProTrajectories;;

public class StraightF extends SequentialCommandGroup {
    /**
     * Creates a new Autonomous.
     */
    public StraightF(ProTrajectories proTrajectories, Leds leds) {
    addCommands(
        new RunCommand(leds::red, leds).withTimeout(3),
        proTrajectories.getRamsete(proTrajectories.centerRightAutoBackwards)
        .raceWith(new RunCommand(leds::rainbow, leds))
    );
  }
}