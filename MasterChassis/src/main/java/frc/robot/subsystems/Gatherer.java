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

public class Gatherer extends SubsystemBase
{
    
    RPMMotor gatherMotor;

    /**
     * Creates a new Gatherer
     */
    public Gatherer()
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

        // Just in case, set everything to 0 on initialization
        gatherMotor.setDesiredRPM(0);
    }

    public void forwardGather()
    {
        gatherMotor.setDesiredRPM(Constants.GATHER_RPM);
    }

    public void stopGathering()
    {
        gatherMotor.setDesiredRPM(0);
    }

    public void reverseGather()
    {
        gatherMotor.setDesiredRPM(-Constants.GATHER_RPM);
    }
}
