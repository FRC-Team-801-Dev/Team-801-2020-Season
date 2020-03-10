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
  }

  //TODO: find correct speed
  public void activateShooter()
  {
    shooterPID.setReference(4000, ControlType.kDutyCycle);
  }
  //TODO: set to go by rotations rather than speed and fix control setup
  public void launchBall()
  {
    breachPID.setReference(4000, ControlType.kDutyCycle);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
