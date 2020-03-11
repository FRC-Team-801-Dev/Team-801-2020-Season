/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fasterxml.jackson.databind.util.RootNameLookup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Magazine;

public class Shoot extends CommandBase
{

    Timer timer;
    boolean hasShot, firstShot;

    /**
     * Creates a new Shoot.
     */
    public Shoot()
    {
        addRequirements(RobotContainer.shooter);
        addRequirements(RobotContainer.magazine);
        
        timer = new Timer();
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        timer.start();
        RobotContainer.shooter.enableShooter();
        RobotContainer.gatherer.reverse();
        RobotContainer.magazine.reverse();

        firstShot = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if(timer.hasElapsed(0.1))
        {
            RobotContainer.magazine.stop();
            if (firstShot)
            {
                RobotContainer.gatherer.stop();
                firstShot = false;
            }
            hasShot = false;
        }

        if (RobotContainer.shooter.isReady() && !hasShot)
        {
            RobotContainer.shooter.popUp();
            hasShot = true;

            Timer.delay(0.25);
            RobotContainer.shooter.holdDown();

            RobotContainer.magazine.forward();
            Timer.delay(1);

            RobotContainer.magazine.reverse();
            timer.reset();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.shooter.stop();
        RobotContainer.magazine.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
