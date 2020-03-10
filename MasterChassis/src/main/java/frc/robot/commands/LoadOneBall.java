package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class LoadOneBall extends SequentialCommandGroup
{
    public LoadOneBall()
    {
        addCommands(new ReverseMagazine(),
                    new WaitCommand(Constants.MAGAZINE_REVERSE_TIME),
                    new ForwardMagazine(),
                    new WaitCommand(Constants.MAGAZINE_LOAD_TIME),
                    new StopMagazine());
    }
}