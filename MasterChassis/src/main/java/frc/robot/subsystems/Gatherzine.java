/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gatherzine extends SubsystemBase 
{
  private TalonSRX gatherer;
  private TalonSRX upperMag;
  private TalonSRX lowerMag;  
  
  /**
   * Creates a new Gatherer and magazine combination.
   * 
   */
  public Gatherzine() 
  {
    gatherer = new TalonSRX(Constants.gatherMotorID);
    upperMag = new TalonSRX(Constants.upperMagMotorID);
    lowerMag = new TalonSRX(Constants.lowerMagMotorID);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    if(RobotContainer.io.getButtonXPressed())
    {
      gatherer.set(ControlMode.Velocity, 1500);
      upperMag.set(ControlMode.Velocity, 1500);
      lowerMag.set(ControlMode.Velocity, 1500);
    }

  }
}
