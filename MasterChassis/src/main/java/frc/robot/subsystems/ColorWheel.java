/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
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

public class ColorWheel extends SubsystemBase 
{
  private CANPIDController spinnerPID;
  private CANSparkMax spinnerMotor;;
  private CANEncoder spinnerEncoder;

  
  private boolean spinnerZeroedFlag;

  private CANDigitalInput m_reverseLimit;

  /**
   * Creates a new color wheel spinner gizmo subsystem.
   */
  public ColorWheel() 
  {
    spinnerMotor = new CANSparkMax(Constants.colorWheelMotorID, MotorType.kBrushless);
    spinnerPID = spinnerMotor.getPIDController();
    spinnerEncoder = spinnerMotor.getEncoder(EncoderType.kHallSensor, 42);
    spinnerPID.setP(Constants.DRIVE_P);
    spinnerPID.setI(Constants.DRIVE_I);
    spinnerPID.setD(Constants.DRIVE_D);
    spinnerPID.setIZone(Constants.DRIVE_IZ);
    spinnerPID.setFF(Constants.DRIVE_FF);
    spinnerPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);

    spinnerMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL, Constants.DRIVE_MAX_CURRENT_RUN);

  
    // for the Neo 550 motor built in encoder we need to do the external gear reductions math in the setPositionConversionFactor
    // 8 to 1 for the drive wheel to color wheel and 10 to 1 for the for the gearbox on the motor. 80 motor rotations = 1 color wheel rotation
    spinnerEncoder.setPositionConversionFactor(1);  // encoder will return rotations
    //575 motor shaft rotations to lift/bring down
    spinnerZeroedFlag = false;

    // limit switch is zero point (fully retracted)
    m_reverseLimit = spinnerMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    m_reverseLimit.enableLimitSwitch(true); 
  }

  public void rotateColorWheel(double rotations)
  {
    //convert color wheel rotations to motorshaft rotations
    spinnerPID.setReference(-Constants.COLORWHEEL_ROTATION_COUNT * 8, ControlType.kPosition);
  }

  public void sendSpinnerMaxHeight() // in motor rotations...
  {
    spinnerPID.setReference(Constants.SPINNER_MAX_HEIGHT, ControlType.kPosition);
  }

  // sets arm to fully retracted and zeros the encoder
  public void resetSpinner()
  {
    spinnerZeroedFlag = false;
    // run till the limit switch stops it... 
    spinnerPID.setReference(Constants.SPINNER_MIN_HEIGHT, ControlType.kPosition);  
  }

  public void spinnerResetting() 
  {
    // This method will be called once per scheduler run
    // test if at the fully retracted position and reset the encoder to zero
    // flag keeps from banging the encoder every scheduler run.
    if(m_reverseLimit.get() && !spinnerZeroedFlag)
    {
      spinnerEncoder.setPosition(0);
      spinnerZeroedFlag = true;
      spinnerPID.setReference(0, ControlType.kPosition);
    }
  }

  public double getHeight()
  {
      return spinnerEncoder.getPosition();
  }

  public boolean getSpinnerZeroedFlag()
  {
      return this.spinnerZeroedFlag;
  }
    
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
