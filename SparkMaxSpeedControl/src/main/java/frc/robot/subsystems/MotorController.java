/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;



import com.revrobotics.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.MotorController;



public class MotorController extends SubsystemBase
{
    
    private MotorController sparkMotorController;
    private NetworkTableEntry motorSpeed;
    
    //private SpeedController sparkMotorController
    
    
    
    /**
     * Creates a new MotorController
     */
    public MotorController()
    {
        
        
        Shuffleboard.enableActuatorWidgets();
        
        
        NetworkTableEntry motorControllerTab = Shuffleboard.getTab("MotorControlTab")
        .getLayout("MotorControlLayout", BuiltInLayouts.kGrid)
        .add("MotorControlPanel", false)
        .getEntry();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }

    public void init()
    {

    }
}
