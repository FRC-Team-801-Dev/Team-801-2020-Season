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

public class Shooter extends SubsystemBase
{
  private CANPIDController shooterPID;
  private CANPIDController breachPID;
  private CANSparkMax shooterMotor;
  private CANSparkMax breachMotor;
  private CANEncoder shooterEncoder;
  private CANEncoder breachEncoder;
  /**
   * Creates a new Shooter.
   */
  public Shooter() 
  {
    shooterMotor = new CANSparkMax(Constants.shooterMotorID, MotorType.kBrushless);
    shooterPID = shooterMotor.getPIDController();
    shooterEncoder = shooterMotor.getEncoder();
    shooterPID.setP(Constants.DRIVE_P);
    shooterPID.setI(Constants.DRIVE_I);
    shooterPID.setD(Constants.DRIVE_D);
    shooterPID.setIZone(Constants.DRIVE_IZ);
    shooterPID.setFF(Constants.DRIVE_FF);
    shooterMotor.setInverted(Constants.SHOOTER_INVERTED);
    shooterPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);

    shooterMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);

    breachMotor = new CANSparkMax(Constants.breachMotorID, MotorType.kBrushless);
    breachPID = breachMotor.getPIDController();
    breachEncoder = breachMotor.getEncoder();
    breachPID.setP(Constants.DRIVE_P);
    breachPID.setI(Constants.DRIVE_I);
    breachPID.setD(Constants.DRIVE_D);
    breachPID.setIZone(Constants.DRIVE_IZ);
    breachPID.setFF(Constants.DRIVE_FF);
    breachPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);
    
    breachMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);
    breachMotor.setInverted(Constants.BREACH_INVERTED);
  }

  //TODO: find correct speed
  public void enableShooter()
  {
    shooterPID.setReference(1.0, ControlType.kDutyCycle);
    //breachPID.setReference(Constants.BREACH_DOWNSPEED, ControlType.kDutyCycle);
  }

  public void popUp()
  {
    breachPID.setReference(Constants.BREACH_UPSPEED, ControlType.kDutyCycle);
  }

  public void holdDown()
  {
    breachPID.setReference(Constants.BREACH_DOWNSPEED, ControlType.kDutyCycle);
  }

  public void stop()
  {
    shooterPID.setReference(0, ControlType.kDutyCycle);
    breachPID.setReference(0, ControlType.kDutyCycle);
  }

  public boolean isReady()
  {
    return shooterEncoder.getVelocity() > Constants.SHOOT_VELOCITY;
  }
}
