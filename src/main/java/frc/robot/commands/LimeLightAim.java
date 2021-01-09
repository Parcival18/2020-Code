/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.*;

public class LimeLightAim extends CommandBase {
  /**
   * Creates a new LimeLightAim.
   */
  Lift LLlift ;
  LimeLight LLlimey;
  DriveTrain LLDrive;

  private double kP = 0.03;
  private double kF = 0.15;


  public LimeLightAim(Lift lift, DriveTrain dTrain, LimeLight limeLight ) {
    // Use addRequirements() here to declare subsystem dependencies.
    LLlift = lift;
    LLDrive = dTrain;
    LLlimey = limeLight; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double error = LLlimey.getHorizontalOffset();
    kF = Math.copySign(kF, error);
    SmartDashboard.putNumber("kF", kF);
    SmartDashboard.putNumber("Error", error);

    double outF = kF;             //Feed forward output
    double outP = kP * error;     //Proportional output

    double outputTurn = outF + outP; //+ outI + outD;

    LLDrive.teleopDrive(0, outputTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LLDrive.teleopDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
