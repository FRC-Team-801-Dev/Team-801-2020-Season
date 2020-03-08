/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.components.RPMMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// NOTE: Complain to Luke Newcomb for problems with this subsystem

public class Gatherzine extends SubsystemBase
{
    
    RPMMotor gatherMotor, magazineMotor;

    boolean gathering = false; // A simple flag to toggle the gatherer

    /**
     * Creates a new Gatherer and feeder combination.
     */
    public Gatherzine()
    {
        // Initialize the gather motor (mini-NEO)
        gatherMotor = new RPMMotor(Constants.GATHER_MOTOR_ID);
        
        // Set motor current limits so we don't blow anything up
        gatherMotor.setCurrentLimits(Constants.GATHER_STALL_CURRENT, Constants.GATHER_FREE_CURRENT);
        
        // Set motor PID values for RPM control
        gatherMotor.setPIDConstants(Constants.GATHER_P,
                                    Constants.GATHER_I,
                                    Constants.GATHER_D,
                                    Constants.GATHER_IZ,
                                    Constants.GATHER_FF,
                                    Constants.GATHER_OUTPUT_MIN,
                                    Constants.GATHER_OUTPUT_MAX);

        // Initialize the magazine motor (mini-NEO)
        magazineMotor = new RPMMotor(Constants.MAGAZINE_MOTOR_ID);
        
        // Set motor current limits so we don't blow anything up
        magazineMotor.setCurrentLimits(Constants.MAGAZINE_STALL_CURRENT, Constants.MAGAZINE_FREE_CURRENT);
        
        // Set motor PID values for RPM control
        magazineMotor.setPIDConstants(Constants.MAGAZINE_P,
                                    Constants.MAGAZINE_I,
                                    Constants.MAGAZINE_D,
                                    Constants.MAGAZINE_IZ,
                                    Constants.MAGAZINE_FF,
                                    Constants.MAGAZINE_OUTPUT_MIN,
                                    Constants.MAGAZINE_OUTPUT_MAX);


        // Just in case, set everything to 0 on initialization
        gatherMotor.setDesiredRPM(0);
        magazineMotor.setDesiredRPM(0);

    }

    public void gather()
    {
        gatherMotor.setDesiredRPM(Constants.GATHER_RPM);
    }

    public void stopGathering()
    {
        gatherMotor.setDesiredRPM(0);
    }

    public void feedOne()
    {
        magazineMotor.setDesiredRPM(-Constants.MAGAZINE_RPM);
    }
}
