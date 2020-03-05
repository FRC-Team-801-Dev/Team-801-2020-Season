/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gatherzine extends SubsystemBase 
{
  private CANSparkMax gatherMotor;
  private CANEncoder gatherEncoder;
  
  private CANSparkMax magazineMotor;
  private CANEncoder magazineEncoder;
  
  
  /**
   * Creates a new Gatherer and magazine combination.
   * 
   */
  public Gatherzine() 
  {
    gatherMotor = new CANSparkMax(Constants.gatherMotorID, MotorType.kBrushless);
    gatherEncoder = gatherMotor.getEncoder();
    gatherMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);
    
    magazineMotor = new CANSparkMax(Constants.magazineMotorID, MotorType.kBrushless);
    magazineEncoder = magazineMotor.getEncoder();
    magazineMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);
  }
  
  public void gather()
  {

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
