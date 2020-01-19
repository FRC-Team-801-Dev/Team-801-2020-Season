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

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.MotorController;

import frc.robot.components.DriveMotor;


public class MotorController extends SubsystemBase
{
    
    NetworkTableEntry motorTab;
    
    NetworkTableEntry sliderValue;
    NetworkTableEntry motorRPMs;
    NetworkTableEntry directInput;

    double lastDirect;


    DriveMotor sparkMotor = new DriveMotor(13);
    
    //private SpeedController sparkMotorController
       
    
    /**
     * Creates a new MotorController
     */
    public MotorController()
    {
    
        Shuffleboard.enableActuatorWidgets();
        
        ShuffleboardTab motorTab = Shuffleboard.getTab("Motor Control");
        motorTab.getLayout("Motor Readout", BuiltInLayouts.kList);
        

        directInput = motorTab.add("Direct Speed Control", 0.0)
        .getEntry();
        lastDirect = directInput.getDouble(0.0);

        sliderValue = motorTab.add("Motor Speed Control", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();

        motorRPMs = motorTab.add("Motor Speed Readout", 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", 6000))
        .getEntry();
    
    }

    @Override
    public void periodic()
    {

        if (directInput.getDouble(0.0) != lastDirect) 
        {
            sliderValue.setDouble(directInput.getDouble(0.0));
            lastDirect = directInput.getDouble(0.0);
        }
        
        sparkMotor.setDesiredRPM(sliderValue.getDouble(0));
        
        motorRPMs.setDouble(sparkMotor.getCurrentRPM());
        // This method will be called once per scheduler run

    }

    public void init()
    {

    }
}