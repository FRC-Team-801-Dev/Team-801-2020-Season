/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class IO 
{
    public static  XboxController driver = new XboxController(0);
//    public static Joystick driver = new Joystick(0);
//    public static Joystick manipulator = new Joystick(1);
    public static  XboxController manipulator = new XboxController(1);

    /**
     * @return the horizontal axis value from the left driver controller
     */
    public double getDriverLeftX()
    {
        return deadbandFilter(driver.getX(Hand.kLeft), 0.1);
    }

    /**
     * @return the vertical axis value from the left joystick on the driver controller
     */
    public double getDriverLeftY()
    {
        return deadbandFilter(driver.getY(Hand.kLeft), 0.1);
    }

    /**
     * @return the horizontal axis value from the right joystick on the driver controller
     */
    public double getDriverRightX()
    {
        return deadbandFilter(driver.getX(Hand.kRight), 0.1); 
    }

    /**
     * @return the vertical axis value from the right joystick on the driver controller
     */
    public double getDriverRightY()
    {
        return deadbandFilter(driver.getY(Hand.kRight), 0.1); 
    }


    /**
     * Reads the driver controller first joystick's horizontal value and applies an exponential function based on the exponent provided
     * @param exponent determines how steep the exponential function is
     */
    public double getDriverExpoLeftX(double exponent)
    {
        return getExponential(getDriverLeftX(), exponent);
    }

    /**
     * Reads the driver controller first joystick's vertical value and applies an exponential function based on the exponent provided
     * @param exponent determines how steep the exponential function is
     */
    public double getDriverExpoLeftY(double exponent)
    {
        return getExponential(getDriverLeftY(), exponent);
    }

    /**
     * Reads the driver controller second joystick's horizontal value and applies an exponential function based on the exponent provided
     * @param exponent determines how steep the exponential function is
     */
    public double getDriverExpoRightX(double exponent)
    {
        return getExponential(getDriverRightX(), exponent);
    }

    /**
     * Reads the driver controller second joystick's vertical value and applies an exponential function based on the exponent provided
     * @param exponent determines how steep the exponential function is
     */
    public double getDriverExpoRightY(double exponent)
    {
        return getExponential(getDriverRightY(), exponent);
    }

  
    /**
     * @return the horizontal axis value from the left driver controller
     */
    public double getManipulatorLeftX()
    {
        return deadbandFilter(manipulator.getX(Hand.kLeft), 0.1);
    }

    /**
     * @return the vertical axis value from the left joystick on the driver controller
     */
    public double getManipulatorLeftY()
    {
        return deadbandFilter(manipulator.getY(Hand.kLeft), 0.1);
    }

    /**
     * @return the horizontal axis value from the right joystick on the driver controller
     */
    public double getManipulatorRightX()
    {
        return deadbandFilter(manipulator.getX(Hand.kRight), 0.1); 
    }

    /**
     * @return the vertical axis value from the right joystick on the driver controller
     */
    public double getManipulatorRightY()
    {
        return deadbandFilter(manipulator.getY(Hand.kRight), 0.1); 
    }
   
    private double deadbandFilter(double value, double deadband)
    {
        return Math.abs(value) > deadband ? value : 0;
    }

    
    /**
     * This function takes a joystick input and applies an exponential scaling
     */
    private double getExponential(double stickInput, double exponent)
    {
        double stickOutput;

        // stickOutput = e^(exponent*|stickInput|) - 1
        stickOutput = Math.exp(Math.abs(stickInput) * exponent) - 1; // Creates an exponential function starting at 0 and with a steepness based on the exponent
        stickOutput /= Math.exp(exponent) - 1; // Scales it back so that at input of 1.0, the output is 1.0
        stickOutput *= Math.signum(stickInput); // Reapplies polarity of the input

        return stickOutput;
    }

}