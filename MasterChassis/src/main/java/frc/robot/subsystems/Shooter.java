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
  private CANPIDController boltPID;
  private CANSparkMax shooterMotor;
  private CANSparkMax boltMotor;
  private CANEncoder shooterEncoder;
  private CANEncoder boltEncoder;
  /**
   * Creates a new Shooter.
   */
  public Shooter(int shooterMotorID, int boltMotorID) 
  {
    shooterMotor = new CANSparkMax(shooterMotorID, MotorType.kBrushless);
    shooterPID = shooterMotor.getPIDController();
    shooterEncoder = shooterMotor.getEncoder();
    shooterPID.setP(Constants.DRIVE_P);
    shooterPID.setI(Constants.DRIVE_I);
    shooterPID.setD(Constants.DRIVE_D);
    shooterPID.setIZone(Constants.DRIVE_IZ);
    shooterPID.setFF(Constants.DRIVE_FF);
    shooterPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);

    shooterMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);

    boltMotor = new CANSparkMax(boltMotorID, MotorType.kBrushless);
    boltPID = boltMotor.getPIDController();
    boltEncoder = boltMotor.getEncoder();
    boltPID.setP(Constants.DRIVE_P);
    boltPID.setI(Constants.DRIVE_I);
    boltPID.setD(Constants.DRIVE_D);
    boltPID.setIZone(Constants.DRIVE_IZ);
    boltPID.setFF(Constants.DRIVE_FF);
    boltPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);
    
    boltMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);
  }

  //TODO: find correct speed
  public void activateShooter()
  {
    shooterPID.setReference(4000, ControlType.kDutyCycle);
  }
  //TODO: set to go by rotations rather than speed and fix control setup
  public void launchBall()
  {
    boltPID.setReference(4000, ControlType.kDutyCycle);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    if(RobotContainer.io.getButtonXPressed())
    {
      activateShooter();
    }
    if(RobotContainer.io.getButtonYPressed())
    {
      launchBall();
    }
    
  }
}
