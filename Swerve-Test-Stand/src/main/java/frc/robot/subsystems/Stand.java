/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.components.SwervePod;
import frc.robot.utilities.Utils;


/**
 * Add your docs here.
 */
public class Stand extends SubsystemBase
{
    private SwervePod pod;
    //private RollingAverage averageHeading;

    private static double desiredHeading; 
    private static double currentHeading;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    // Store what the last hue of the first pixel is
    private int m_rainbowFirstPixelHue;
    private int m_tracePosition = 0;
    private boolean traceFlag = false;


    public Stand()
    {
    pod = new SwervePod(Constants.DRIVE_POD_ID, Constants.TURN_POD_ID, 0);
    //averageHeading = new RollingAverage(3);
    

    desiredHeading = currentHeading = pod.getCurrentAngle();


    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(20);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    }

    public void teleopPeriodic()
    {
        // Always call to process PID for turn motor
        pod.processPod();

        //double x = Robot.m_oi.getDriverX();
        double speed = Utils.magnitude(RobotContainer.io.getDriverLeftX(), RobotContainer.io.getDriverLeftY());
        double x_r = RobotContainer.io.getDriverExpoRightX(2.5); // Rotation (x)

        //pod.setDesiredRPM(Utils.map(y, -1, 1, -8, 8));
        //System.out.println(theta);
        pod.setDesiredRPM(speed);
    
        if (Math.abs( x_r ) > .02) // puts some deadband on the input
        {
          desiredHeading = desiredHeading + (x_r / 2);
          // keep heading a positive angle
          if (desiredHeading < 0) 
          {
            desiredHeading += ( 2 * Math.PI );
          }
          if (desiredHeading >= ( 2 * Math.PI))
          {
            desiredHeading = desiredHeading % ( 2 * Math.PI);
          }
        }


        if(RobotContainer.io.getButtonAPressed())
        {
          desiredHeading = 0;
        }

        pod.setDesiredAngle(desiredHeading);

        if(RobotContainer.io.getButtonXPressed())
        {
          pod.resetEncoder();
        }


        if(RobotContainer.io.getButtonYPressed())
        {
          launchTrace();
        }


        Color detectedColor = m_colorSensor.getColor();

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        double IR = m_colorSensor.getIR();
    
        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */

        // Calculate hue from RGB
        double hue = -1;
        double max_v = Utils.max(detectedColor.red, detectedColor.green, detectedColor.blue);
        double delta = max_v - Utils.min(detectedColor.red, detectedColor.green, detectedColor.blue);

        if (max_v == detectedColor.red)
        {
          hue = (detectedColor.green - detectedColor.blue)/delta % 6;
        } else if (max_v == detectedColor.green)
        {
          hue = (detectedColor.blue - detectedColor.red)/delta + 2;
        } else if (max_v == detectedColor.blue) 
        {
          hue = (detectedColor.red - detectedColor.green)/delta + 4;
        }


        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);

        /* Hue Chart:
         * blue   = 3.00
         * green  = 2.20
         * red    = 0.55
         * yellow = 1.55 
        */

        // Write the color to the SmartDashboard
        SmartDashboard.putNumber("Hue", hue);
        if (Utils.aboutEqual(hue, 3.0))
        {
          SmartDashboard.putString("Color", "blue");
        } else if (Utils.aboutEqual(hue, 2.2))
        {
          SmartDashboard.putString("Color", "green");
        } else if (Utils.aboutEqual(hue, 0.55))
        {
          SmartDashboard.putString("Color", "red");
        } else if (Utils.aboutEqual(hue, 1.55))
        {
          SmartDashboard.putString("Color", "yellow");
        }

        SmartDashboard.putNumber("IR", IR);
    
        /**
         * In addition to RGB IR values, the color sensor can also return an 
         * infrared proximity value. The chip contains an IR led which will emit
         * IR pulses and measure the intensity of the return. When an object is 
         * close the value of the proximity will be large (max 2047 with default
         * settings) and will approach zero when the object is far away.
         * 
         * Proximity can be used to roughly approximate the distance of an object
         * or provide a threshold for when an object is close enough to provide
         * accurate color values.
         */
        int proximity = m_colorSensor.getProximity();
    
        SmartDashboard.putNumber("Proximity", proximity);



        // Fill the buffer with a rainbow
        //rainbow();
        // Set the LEDs
        //m_led.setData(m_ledBuffer);

        if(traceFlag)
        {
          trace();
        // Set the LEDs
        m_led.setData(m_ledBuffer);
        } 
    }


    public double getAngle()
    {
        return pod.getCurrentAngle();
    }



    private void rainbow() {
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        // Set the value
        m_ledBuffer.setHSV(i, hue, 255, 128);

      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
    }


    public void launchTrace()
    {
      traceFlag = true;
      m_tracePosition = 0;
    }
    

    private void trace() {
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        if( i > m_tracePosition - 5 && i <= m_tracePosition)
        {
          m_ledBuffer.setRGB(i, 0, 0, 255);
        }
        else
        {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
      }
      ++m_tracePosition;
      if(m_tracePosition > m_ledBuffer.getLength() + 6)
      {
        traceFlag = false;  //stop the trace
      }
    }

}
