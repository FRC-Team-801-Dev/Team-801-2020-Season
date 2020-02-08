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
import frc.robot.utilities.Utils;

/**
 * Add your docs here.
 */
public class Stand extends SubsystemBase
{

    private SwervePod pod = new SwervePod(Constants.DRIVE_POD_ID, Constants.TURN_POD_ID, 0);

    public void teleopPeriodic()
    {
        // Always call to process PID for turn motor
        pod.processPod();


        //double x = Robot.m_oi.getDriverX();
        double speed = Utils.magnitude(RobotContainer.io.getDriverLeftX(), RobotContainer.io.getDriverLeftY());


        // double theta = Utils.angle(Robot.io.getDriverLeftX(), Robot.io.getDriverLeftY());

        //double mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) );

        //pod.setDesiredRPM(Utils.map(y, -1, 1, -8, 8));
        //System.out.println(theta);
        pod.setDesiredRPM(speed);

        if(RobotContainer.io.getButtonAPressed())
        {
            pod.setDesiredAngle(0.0);
        }

        if(RobotContainer.io.getButtonBPressed())
        {
            pod.setDesiredAngle(2.0);
        }

        if(RobotContainer.io.getButtonXPressed())
        {
            pod.setDesiredAngle(4.0);
        }

        if(RobotContainer.io.getButtonYPressed())
        {
            pod.setDesiredAngle(6.0);
        }
    }


    public double getAngle()
    {
        return pod.getCurrentAngle();
    }
}
