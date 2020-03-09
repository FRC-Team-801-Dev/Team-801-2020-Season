/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmWinch extends SubsystemBase 
{
  private CANPIDController armPID;
  private CANPIDController winchPID;
  private CANSparkMax armMotor;
  private CANSparkMax winchMotor;
  private CANEncoder armEncoder;
  private CANEncoder winchEncoder;

  private boolean armZeroFlag;

  private CANDigitalInput m_reverseLimit;

  /**
   * Creates a new LifterWinch.
   */
  public ArmWinch() 
  {
    // Arm Settings
    armMotor = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
    armMotor.setInverted(Constants.ARM_INVERT);
    armMotor.setIdleMode(Constants.ARM_IDLEMODE);
    armMotor.setSmartCurrentLimit(Constants.ARM_MAX_CURRENT_STALL, Constants.ARM_MAX_CURRENT_RUN);

    armPID = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder(EncoderType.kHallSensor, 42);
    armPID.setP(Constants.ARM_P);
    armPID.setI(Constants.ARM_I);
    armPID.setD(Constants.ARM_D);
    armPID.setIZone(Constants.ARM_IZ);
    armPID.setFF(Constants.ARM_FF);
    armPID.setOutputRange(Constants.ARM_MIN_OUTPUT, Constants.ARM_MAX_OUTPUT);

    // for the Neo 550 motor built in encoder we need to do the external gear reductions math in the setPositionConversionFactor
    // 10 to 1 for the gearbox on the motor.
    armEncoder.setPositionConversionFactor(1);  // encoder will now return lead screw rotations
    armZeroFlag = true;

    // limit switch is zero point (fully retracted)
    m_reverseLimit = armMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_reverseLimit.enableLimitSwitch(true);


    // Winch Settings
    winchMotor = new CANSparkMax(Constants.winchMotorID, MotorType.kBrushless);
    winchPID = winchMotor.getPIDController();
    winchEncoder = winchMotor.getEncoder();
    winchPID.setP(Constants.WINCH_P);
    winchPID.setI(Constants.WINCH_I);
    winchPID.setD(Constants.WINCH_D);
    winchPID.setIZone(Constants.WINCH_IZ);
    winchPID.setFF(Constants.WINCH_FF);
    winchPID.setOutputRange(Constants.WINCH_MIN_OUTPUT, Constants.WINCH_MAX_OUTPUT);
    
    winchMotor.setSmartCurrentLimit(Constants.WINCH_MAX_CURRENT_STALL, Constants.WINCH_MAX_CURRENT_RUN);

    }
    

  public void sendArmHeight(double rotations) // in lead screw rotations...
  {
    //Lifter Motor must rotate 400 times to go 4 inches on lead screw, and 40 inches in height.
    armPID.setReference(rotations, ControlType.kPosition);

  }

  // sets arm to fully retracted and zeros the encoder
  public void resetArm()
  {
    armZeroFlag = true;
    // run till the limit switch stops it...  using 500 because that should be more than the 
    // lead screw length.
    armPID.setReference(Constants.ARM_POSITION_RESET, ControlType.kPosition);  

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    // test if at the fully retracted position and reset the encoder to zero
    // flag keeps from banging the encoder every scheduler run.
    if(m_reverseLimit.get() && armZeroFlag)
    {
      armEncoder.setPosition(0);
      armZeroFlag = false;
      armPID.setReference(0, ControlType.kPosition);
    }
  }
}
