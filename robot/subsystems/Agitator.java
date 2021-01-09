/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {
  TalonSRX LeftAgitator = new TalonSRX(20);//left
  TalonSRX RightAgitator = new TalonSRX(21);//Right
  PigeonIMU pidgey = new PigeonIMU(RightAgitator);

  /**
   * Creates a new Agitator.
   */
  public Agitator() {
    RightAgitator.configFactoryDefault();
    LeftAgitator.configFactoryDefault();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveVoltage(double x, double y)
    {
        double speed = x + y;           //sets the speed of the left wheels
        double speed1 = x - y;          //sets the speed of the right wheels

        LeftAgitator.set(ControlMode.PercentOutput, speed);        //sets the LeftMaster to speed
        
        RightAgitator.set(ControlMode.PercentOutput, -speed1);     //sets the RightMaster to speed1
        
    }

  public double getAngle(){
    double[] ypr_deg = new double[3];
    pidgey.getYawPitchRoll(ypr_deg);
		return ypr_deg[0];
  }

  public void resetGyro(){
    pidgey.setYaw(0.0);
  }

public Object DriveAgi(double d, double rawAxis) {
	return null;
}  
}
