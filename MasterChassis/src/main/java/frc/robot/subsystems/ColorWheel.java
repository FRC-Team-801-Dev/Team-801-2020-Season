/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheel extends SubsystemBase 
{
  private CANPIDController spinnerPID;
  private CANSparkMax spinnerMotor;;
  private CANEncoder spinnerEncoder;

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
    spinnerEncoder.setPositionConversionFactor(1);  // encoder will return radians

    }
    
  //TODO: set to go by position rather than speed, as well as correct encoder count.
  // Will have 4 preset points AND manual control. 
  public void rotateColorWheel()
  {
    //Lifter Motor must rotate 400 times to go 4 inches on lead screw, and 40 inches in height.
    spinnerPID.setReference(400, ControlType.kPosition);
  }

  public void spin()
  {
    //Winch goes to max retraction of cable and lifter arm comes all the way back to the limit switch. 
    spinnerPID.setReference(400, ControlType.kPosition);

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
