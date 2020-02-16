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

public class LifterWinch extends SubsystemBase 
{
  private CANPIDController lifterPID;
  private CANPIDController winchPID;
  private CANSparkMax lifterMotor;
  private CANSparkMax winchMotor;
  private CANEncoder lifterEncoder;
  private CANEncoder winchEncoder;

  /**
   * Creates a new LifterWinch.
   */
  public LifterWinch(int lifterMotorID, int winchMotorID) 
  {
    lifterMotor = new CANSparkMax(lifterMotorID, MotorType.kBrushless);
    lifterPID = lifterMotor.getPIDController();
    lifterEncoder = lifterMotor.getEncoder();
    lifterPID.setP(Constants.DRIVE_P);
    lifterPID.setI(Constants.DRIVE_I);
    lifterPID.setD(Constants.DRIVE_D);
    lifterPID.setIZone(Constants.DRIVE_IZ);
    lifterPID.setFF(Constants.DRIVE_FF);
    lifterPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);

    lifterMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);

    winchMotor = new CANSparkMax(winchMotorID, MotorType.kBrushless);
    winchPID = winchMotor.getPIDController();
    winchEncoder = winchMotor.getEncoder();
    winchPID.setP(Constants.DRIVE_P);
    winchPID.setI(Constants.DRIVE_I);
    winchPID.setD(Constants.DRIVE_D);
    winchPID.setIZone(Constants.DRIVE_IZ);
    winchPID.setFF(Constants.DRIVE_FF);
    winchPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);
    
    winchMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);
    }


    //TODO: set to go by rotations rather than speed, as well as correct rotation count.  
    public void sendUpLifter()
    {
      lifterPID.setReference(4000, ControlType.kDutyCycle);
    }
    public void liftUpRobot()
    {
      lifterPID.setReference(4000, ControlType.kDutyCycle);
      winchPID.setReference(4000, ControlType.kDutyCycle);
    }




  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    if(RobotContainer.io.getButtonXPressed())
    {
      sendUpLifter();
    }
    if(RobotContainer.io.getButtonYPressed())
    {
      liftUpRobot();
    }
  }
}
