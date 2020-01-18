package frc.robot.components;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveMotor
{
    private CANPIDController sparkPID;
    private CANSparkMax sparkMotor;
    private CANEncoder sparkEncoder;

    /**
     * 
     * @param motorID of the Spark Max
     * @param motorIndex of the motor index
     */
    public DriveMotor(int motorID)
    {
        sparkMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        sparkPID = sparkMotor.getPIDController();
        sparkEncoder = sparkMotor.getEncoder();
    }

    public void SetMotorSpeed(double motorSpeed)
    {
        sparkMotor.set(motorSpeed);
    }
    
}