package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.JoystickDriveChassis;
import frc.robot.components.SwervePod;
import frc.robot.utilities.Utils;

/**
 * Subsystem to control the entire drive base
 */
public class Chassis extends Subsystem 
{

    //                      ^ Front
    //                      |
    //           ________________________
    //          /						 \
    //          |	1				2	 |
    //          |						 |
    //          |						 |
    //          |						 |
    //          |						 |
    //          |  						 |
    //          |						 |
    //          |						 |
    //          |	3				4	 |
    //          |						 |
    //          \________________________/
    
    private SwervePod pod1 = new SwervePod(Constants.POD_1_DRIVE, Constants.POD_1_TURN, Constants.POD_FRONT_LEFT);
    private SwervePod pod2 = new SwervePod(Constants.POD_2_DRIVE,Constants.POD_2_TURN, Constants.POD_FRONT_RIGHT);
    private SwervePod pod3 = new SwervePod(Constants.POD_3_DRIVE, Constants.POD_3_TURN, Constants.POD_BACK_LEFT);
    private SwervePod pod4 = new SwervePod(Constants.POD_4_DRIVE, Constants.POD_4_TURN, Constants.POD_BACK_RIGHT);

    private SwervePod[] pods = new SwervePod[] {pod1, pod2, pod3, pod4};

    public Chassis()
    {
        for(SwervePod pod : pods)
        {
            pod.zeroEncoder();
        }
    }


    @Override
    public void initDefaultCommand() 
    {
        setDefaultCommand(new JoystickDriveChassis());
    }



    /**
     * Function called by the JoystickDriveChassis command to drive the robot
     */
    public void joystickDrive()
    {

        // Always call to process PID for turn motors
        for (SwervePod pod : pods)
        {
            pod.processPod();
        }

        double x = Robot.io.getDriverLeftX(); // Translation x
        double y = -Robot.io.getDriverLeftY(); // Translation y
        double r = Robot.io.getDriverRightX(); // Rotation (x)

        // Dimensions will change! What are the dimensions of the test chassis!
        // Change in Constants.java
        //Robot dimensions (example)
        //Length = 24 in
        //Width  = 20 in
        //
        //      20
        //________________
        //|              |
        //|              |
        //|              |
        //|              |
        //|              |  24
        //|              |
        //|              |
        //|              |
        //----------------
        // SEE Constants.java

        double length = Constants.ROBOT_LENGTH;
        double width = Constants.ROBOT_WIDTH;

        double thetaChassis = Math.atan(length / width); // Gets the angle created from the center of the robot to the top right corner

        /**
         * rh1 is the angle for pod1, and so on.
         * These represent the angles from each pod to the center of the robot.
         * (The angle each one must turn to, to point perpendicular to the line from the center to the pod)
         */

        double rh1 = thetaChassis + Math.PI / 2;
        double rh2 = thetaChassis;
        double rh3 = 2*Math.PI - thetaChassis;
        double rh4 = thetaChassis + Math.PI;

        // double rh1 = (3/4)*Math.PI;
        // double rh2 = (1/4)*Math.PI;
        // double rh3 = (7/4)*Math.PI;
        // double rh4 = (5/4)*Math.PI;

        double rotationPower = -r; // The magnitude of the rotation that we want to perform

        // SwervePower ranges from [0,1], but the xbox control ranges from [0,sqrt(2)], so divide by sqrt(2) 
        double speed = Utils.magnitude(x, y) / Math.sqrt(2);
        double heading = Utils.normalizeAngle(Utils.angle(x, y) - Math.PI/2);

        double[] translationVector = new double[] {heading, speed};

        // Create an array we can loop over of the vectors
        double[][] finalVectors = new double[4][2];

        // We are storing arrays as double arrays in the form [angle, mag]
        finalVectors[0] = Utils.addVectors(new double[] {rh1, rotationPower}, translationVector);
        finalVectors[1] = Utils.addVectors(new double[] {rh2, rotationPower}, translationVector);
        finalVectors[2] = Utils.addVectors(new double[] {rh3, rotationPower}, translationVector);
        finalVectors[3] = Utils.addVectors(new double[] {rh4, rotationPower}, translationVector);

        // Find the largest vector
        double maxVectorMagnitude = Utils.max(new Double[] {finalVectors[0][1], finalVectors[1][1], finalVectors[2][1], finalVectors[3][1]});

        // Normalize all of the vectors to less than 1.0
        // if at least one is larger than 1.0
        if (maxVectorMagnitude > 1.0)
        {
            // Loop through and divide each by the longest
            for (double[] vector : finalVectors)
            {
                vector[1] /= maxVectorMagnitude;
            }
        }

        // Set all of the pod angles and speeds based on these vectors
        if (maxVectorMagnitude != 0)
        {
            for (int i = 0; i < pods.length; i++) 
            {
                pods[i].setDesiredAngle(finalVectors[i][0]);
                pods[i].setDesiredRPM(finalVectors[i][1]);
            }
        }
        else
        {
            for (SwervePod pod : pods)
            {
                pod.setDesiredRPM(0);
            }
        }
    }

    /**
     * A function that gives every pod angle
     * @return An array of every angle of every pod in the order specified above
     */
    public double[] getAngles()
    {
        return new double[] {pod1.getCurrentAngle(), pod2.getCurrentAngle(), pod3.getCurrentAngle(), pod4.getCurrentAngle()};
    }
}
