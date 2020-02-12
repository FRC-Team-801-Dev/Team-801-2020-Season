/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.components.SwervePod;
import frc.robot.utilities.RollingAverage;
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


    public Stand()
    {
    pod = new SwervePod(Constants.DRIVE_POD_ID, Constants.TURN_POD_ID, 0);
    //averageHeading = new RollingAverage(3);

    desiredHeading = currentHeading = pod.getCurrentAngle();
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

    }
    

    public double getAngle()
    {
        return pod.getCurrentAngle();
    }
}
