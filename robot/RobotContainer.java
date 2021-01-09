/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;



/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final LimeLight limeLight = new LimeLight();
  private final Lift lift = new Lift();
  private final DriveTrain driveTrain = new DriveTrain(lift, limeLight);
  private final BrakeClimb brakeClimb = new BrakeClimb();
  private final Leds leds = new Leds();
  private final Raptor raptor = new Raptor();
  private final AutoIntake autoIntake = new AutoIntake();
  private final Launcher launcher = new Launcher();

  private final Command justMove = new RunCommand(()-> driveTrain.teleopDrive(.5, 0)).withTimeout(1);
  private final Command midAutoF = new MidAutoFordward(launcher, autoIntake, driveTrain);
  private final Command midAutoB = new MidAutoBackward(launcher, autoIntake, driveTrain);
  private final Command autoLimey = new AutoLimelight(launcher, autoIntake, driveTrain, limeLight);
  private final Command chaleXD = new ChaleXD(launcher, autoIntake, driveTrain, limeLight);
  private final Command baila = new Baila(launcher, autoIntake, driveTrain, limeLight);
  private final Command LimeyDelay = new AutoLimelightDelay(launcher, autoIntake, driveTrain, limeLight);

  SendableChooser<Command> chooseBro = new SendableChooser<>();

  Timer A = new Timer();

  Joystick Gamepad0 = new Joystick(0);
  Joystick Gamepad1 = new Joystick(1);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(
      new RunCommand(() -> driveTrain
         // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
      .teleopDrive(-Gamepad0.getRawAxis(1),
                   Gamepad0.getRawAxis(4)), driveTrain));

    lift.setDefaultCommand(
      new RunCommand(() -> lift
                     // Left Trigger and Right Trigger Drive Lift
      .manualLift(-Gamepad0.getRawAxis(2) + Gamepad0.getRawAxis(3)), lift));
    
      leds.setDefaultCommand(new RunCommand(leds::rainbow, leds));
      autoIntake.setDefaultCommand(new RunCommand(autoIntake::BallOBall, autoIntake));
      limeLight.setPipeline(0); 

      chooseBro.setDefaultOption("Just Move", justMove);

      Shuffleboard.getTab("Autonomous").add(chooseBro);
      chooseBro.addOption("Move Fordward", midAutoF);
      chooseBro.addOption("Move Backward", midAutoB);
      chooseBro.addOption("Limey Auto", autoLimey);
      chooseBro.addOption("Chale", chaleXD);
      chooseBro.addOption("Magic Dance", baila);
      chooseBro.addOption("Delay in AutoLimey", LimeyDelay);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Gamepad0
    new JoystickButton(Gamepad0, 1)
      .whenHeld(new LimeLightAim(lift, driveTrain, limeLight));
    new JoystickButton(Gamepad0, 2)
      .whenPressed(new InstantCommand(brakeClimb::liftBrakeOff, brakeClimb))
      .whenPressed(new RunCommand(leds::red, leds))
      .whenReleased(new InstantCommand(brakeClimb::liftBrakeOn, brakeClimb))
      .whenReleased(new RunCommand(leds::rainbow, leds));
    new JoystickButton(Gamepad0, 3)
      .whenPressed(new InstantCommand(()-> limeLight.setPipeline(0)));
    new JoystickButton(Gamepad0, 4)
      .whileHeld(new RunCommand(()-> driveTrain.TurnAngleManual(90)))
      .whenReleased((new InstantCommand(driveTrain::zeroHeading, driveTrain)));
    new JoystickButton(Gamepad0, 5)
      .whenPressed(new InstantCommand(raptor::openClimbHook, raptor));
    new JoystickButton(Gamepad0, 6)
      .whenPressed(new InstantCommand(raptor::closeClimbHook, raptor))
      .whenPressed(new RunCommand(leds::green, leds));
    new JoystickButton(Gamepad0, 7)
      .whileHeld(new RunCommand(()-> driveTrain.driveStraight(.4)));
    new JoystickButton(Gamepad0, 9)
      .whileHeld(new RunCommand(()-> driveTrain.DriveDistance(1)));
    new JoystickButton(Gamepad0, 10)
      .whenPressed(new InstantCommand(driveTrain::resetEncoders, driveTrain));

    //Gamepad1
    new JoystickButton(Gamepad1, 3)
      .whenPressed(new InstantCommand(autoIntake::GoDown, autoIntake))
      .whenReleased(new InstantCommand(autoIntake::GoUp, autoIntake));
    new JoystickButton(Gamepad1, 5)
      .whenPressed(new RunCommand(autoIntake::SpitIt, autoIntake))
      .whenReleased(new InstantCommand(autoIntake::StopIt, autoIntake));
    new JoystickButton(Gamepad1, 6)
      .whenPressed(new RunCommand(autoIntake::SuckIt, autoIntake))
      .whenReleased(new InstantCommand(autoIntake::StopIt, autoIntake));
    new JoystickButton(Gamepad1, 7)
      .whenPressed(new InstantCommand(autoIntake::IntakeIt, autoIntake))
      .whenReleased(new InstantCommand(autoIntake::NoInte, autoIntake));
    new JoystickButton(Gamepad1, 8)
      .whenPressed(new InstantCommand(autoIntake::Defend, autoIntake))
      .whenReleased(new InstantCommand(autoIntake::NoInte, autoIntake));
    new JoystickButton(Gamepad1, 9)
      .whenPressed(new InstantCommand(raptor::OpenMouth, raptor))
      .whenReleased(new InstantCommand(raptor::CloseMouth, raptor)); 

    //Gamepad1 POV
    new POVButton(Gamepad1, 180)//Down
      .whenPressed(new RunCommand(launcher::ShortShoot, launcher))
      .whenPressed(new InstantCommand(raptor::OpenMouth, raptor))
      .whenPressed(new RunCommand(leds::green, leds));
    new POVButton(Gamepad1, 270)//left
      .whenPressed(new RunCommand(launcher::MidShoot, launcher))
      .whenPressed(new InstantCommand(()-> limeLight.setPipeline(1)))
      .whenPressed(new InstantCommand(raptor::CloseMouth, raptor))
      .whenPressed(new RunCommand(leds::yellow, leds));
    new POVButton(Gamepad1, 0)//Up
      .whenPressed(new RunCommand(launcher::LongShoot, launcher))
      .whenPressed(new InstantCommand(()-> limeLight.setPipeline(2)))
      .whenPressed(new InstantCommand(raptor::CloseMouth, raptor))
      .whenPressed(new RunCommand(leds::red, leds));
    new POVButton(Gamepad1, 90)//right
      .whenPressed(new RunCommand(launcher::StopShoot, launcher))
      .whenPressed(new InstantCommand(raptor::CloseMouth, raptor))
      .whenPressed(new RunCommand(leds::rainbow, leds)); 
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    return chooseBro.getSelected();
  }
}
