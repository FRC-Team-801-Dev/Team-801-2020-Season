
package frc.robot.components;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

public class RPMMotor
{

    private double currentRPM = 0.0;
    private double desiredRPM = 0.0;

    private CANPIDController sparkPID;
    private CANSparkMax sparkMotor;
    private CANEncoder sparkEncoder;

    /**
     * 
     * @param motorID of the Spark Max
     * @param motorIndex of the motor index
     */
    public RPMMotor(int motorID)
    {
        sparkMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        sparkPID = sparkMotor.getPIDController();
        sparkEncoder = sparkMotor.getEncoder();
    }

    /**
     * A method to initialize the PID
     * @param P
     * @param I
     * @param D
     * @param IZ
     * @param FF
     * @param minOutput
     * @param maxOutput
     */
    public void setPIDConstants(double P, double I, double D, double IZ, double FF, double minOutput, double maxOutput)
    {
        sparkPID.setP(P);
        sparkPID.setI(I);
        sparkPID.setD(D);
        sparkPID.setIZone(IZ);
        sparkPID.setFF(FF);
        sparkPID.setOutputRange(minOutput, maxOutput);
    }

    /**
     * Configures the all-important current limits for the motors
     * @param stallCurrent
     * @param freeCurrent
     */
    public void setCurrentLimits(int stallCurrent, int freeCurrent)
    {
        sparkMotor.setSmartCurrentLimit(stallCurrent, freeCurrent);
    }

     /**
     * 
     * @param rpm desired RPMs of the motor shaft
     */
    public void setDesiredRPM(double rpm)
    {
        desiredRPM = rpm;
        sparkPID.setReference(desiredRPM, ControlType.kVelocity);
    }

    /**
     * s
     * @return motor shaft velocity in RPM
     */
    public double getCurrentRPM()
    {
        currentRPM = sparkEncoder.getVelocity();
        return currentRPM;
    }

    
}