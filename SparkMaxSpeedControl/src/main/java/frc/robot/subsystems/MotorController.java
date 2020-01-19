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

    DriveMotor sparkMotor = new DriveMotor(13);
    
    //private SpeedController sparkMotorController
       
    
    /**
     * Creates a new MotorController
     */
    public MotorController()
    {
       /*
        Shuffleboard.enableActuatorWidgets();
        
        ShuffleboardTab motorTab = Shuffleboard.getTab("MotorControl");
        sliderValue = motorTab.add("Motor Speed Slider", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();
    */
    }

    @Override
    public void periodic()
    {
        sparkMotor.setDesiredRPM(0.5); 
        // This method will be called once per scheduler run
    }

    public void init()
    {

    }
}