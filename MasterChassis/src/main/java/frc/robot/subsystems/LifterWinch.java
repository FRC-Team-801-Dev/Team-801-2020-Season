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
import com.revrobotics.EncoderType;

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
  public LifterWinch() 
  {
    lifterMotor = new CANSparkMax(Constants.lifterMotorID, MotorType.kBrushless);
    lifterPID = lifterMotor.getPIDController();
    lifterEncoder = lifterMotor.getEncoder(EncoderType.kHallSensor, 42);
    lifterPID.setP(Constants.DRIVE_P);
    lifterPID.setI(Constants.DRIVE_I);
    lifterPID.setD(Constants.DRIVE_D);
    lifterPID.setIZone(Constants.DRIVE_IZ);
    lifterPID.setFF(Constants.DRIVE_FF);
    lifterPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);

    lifterMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);

    winchMotor = new CANSparkMax(Constants.winchMotorID, MotorType.kBrushless);
    winchPID = winchMotor.getPIDController();
    winchEncoder = winchMotor.getEncoder();
    winchPID.setP(Constants.DRIVE_P);
    winchPID.setI(Constants.DRIVE_I);
    winchPID.setD(Constants.DRIVE_D);
    winchPID.setIZone(Constants.DRIVE_IZ);
    winchPID.setFF(Constants.DRIVE_FF);
    winchPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);
    
    winchMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);

    lifterEncoder = lifterMotor.getEncoder();
    // for the Neo 550 motor built in encoder we need to do the external gear reductions math in the setPositionConversionFactor
    // 10 to 1 for the height conversion (Not Included in Math) and 10 to 1 for the gearbox on the motor.
    lifterEncoder.setPositionConversionFactor(2 * Math.PI / 10);  // encoder will return radians
    }
    
  //TODO: set to go by position rather than speed, as well as correct encoder count.
  // Will have 4 preset points AND manual control. 
  public void sendUpLifter()
  {
    //Lifter Motor must rotate 400 times to go 4 inches on lead screw, and 40 inches in height.
    lifterPID.setReference(400, ControlType.kPosition);
  }

  public void liftUpRobot()
  {
    //Winch goes to max retraction of cable and lifter arm comes all the way back to the limit switch. 
    lifterPID.setReference(400, ControlType.kPosition);
    winchPID.setReference(400, ControlType.kPosition);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
