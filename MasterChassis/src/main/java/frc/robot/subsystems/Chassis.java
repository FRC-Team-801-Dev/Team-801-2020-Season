package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.components.SwervePod;
import frc.robot.utilities.Utils;
import frc.robot.utilities.PID;
import frc.robot.utilities.RollingAverage;

// import java.util.Map;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/**
 * Subsystem to control the entire drive base
 */
public class Chassis extends SubsystemBase
{

    //              ^ Front
    //              |
    //  ________________________
    // /                        \
    // | 1                    2 |
    // |                        |
    // |                        |
    // |                        |
    // |                        |
    // |                        |
    // |                        |
    // |                        |
    // | 3                    4 |
    // |                        |
    // \________________________/

    private SwervePod pod1 =
            new SwervePod(Constants.POD_1_DRIVE, Constants.POD_1_TURN, Constants.POD_FRONT_LEFT);
    private SwervePod pod2 =
            new SwervePod(Constants.POD_2_DRIVE, Constants.POD_2_TURN, Constants.POD_FRONT_RIGHT);
    private SwervePod pod3 =
            new SwervePod(Constants.POD_3_DRIVE, Constants.POD_3_TURN, Constants.POD_BACK_LEFT);
    private SwervePod pod4 =
            new SwervePod(Constants.POD_4_DRIVE, Constants.POD_4_TURN, Constants.POD_BACK_RIGHT);

    private SwervePod[] pods = new SwervePod[] {pod1, pod2, pod3, pod4};

    private AHRS gyro;

    // Speed component for rotation about the Z axis. [-x, x]
    private static double vTheta;

    // heading about a unit circle in radians.
    private final double joystickTurnMultiplier = 50.0;
    private static double desiredHeading; // rotates about the Z axis [0,360) deg.
    private static double currentHeading; // rotates about the Z axis [0,360) deg.

    // PID for the heading
    private final double propCoeff = 0.1;
    private final double integCoeff = 0.0;
    private final double diffCoeff = 0.0;
    private final double OutputLowLimit = -1;
    private final double OutputHighLimit = 1;
    private final double MaxIOutput = 1;
    private final double OutputRampRate = 0.1;
    private final double OutputFilter = 0;
    private final double SetpointRange = 360;

    private final double headingThreshold = 0.05;
    private final int headdingAverageNumberOfSamples = 5;

    private PID headingPID;
    private RollingAverage averageHeading;

    NetworkTableEntry error_x;
    NetworkTableEntry error_y;

    public Chassis()
    {

        for (SwervePod pod : pods)
        {
            pod.zeroEncoder();
        }

        vTheta = 0;

        try
        {
            /* Communicate w/navX-MXP via the MXP SPI Bus. */
            /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
            /*
             * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.
             */
            gyro = new AHRS(SPI.Port.kMXP);
        }
        catch (RuntimeException ex)
        {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }

        headingPID = new PID(propCoeff, integCoeff, diffCoeff);
        averageHeading = new RollingAverage(headdingAverageNumberOfSamples);

        // set initial desired heading to the current actual heading.
        desiredHeading = currentHeading = gyro.getYaw();

        // initially setup the PID parameters
        headingPID.setOutputLimits(OutputLowLimit, OutputHighLimit);
        headingPID.setMaxIOutput(MaxIOutput);
        headingPID.setOutputRampRate(OutputRampRate);
        headingPID.setOutputFilter(OutputFilter);
        headingPID.setAngleUnits(PID.AngleUnit.degrees);
        headingPID.setSetpointRange(SetpointRange);
        headingPID.setContinousInputRange(360);
        headingPID.setContinous(true); // lets PID know we are working with a continuous range
                                       // [0-360)

        NetworkTableInstance networkTableInst = NetworkTableInstance.getDefault();

        // Get the table within instance that contains the data. There can
        NetworkTable networkTable = networkTableInst.getTable("datatable");
        error_x = networkTable.getEntry("error_x");
        error_y = networkTable.getEntry("error_y");
    }

    public void drive(double x, double y, double r)
    {
        // double x_l = RobotContainer.io.getDriverExpoLeftX(2.5); // Translation X
        // double y_l = -RobotContainer.io.getDriverExpoLeftY(2.5); // Translation Y
        // double x_r = RobotContainer.io.getDriverExpoRightX(2.5); // Rotation (x)

        // if there is new joystick input update the heading otherwise hold the current
        // heading as
        // the setpoint.
        // headding is in radians so just using the +/- 1 from the joystick to add as a
        // bias to the
        // current angle will put the desired head +/- 57 degrees from current. This
        // should be more
        // than enough to move the bot at max rotation speed.
        // The chasing of this setpoint is controled by the PID loop on the vTheta
        // value.

        averageHeading.add(r * joystickTurnMultiplier); // average in the current stick value

        // if the averaged stick input is greater then the headingThreshold go ahead and
        // adjust the heading.
        // This keeps from updating the desiredHeading value if no joystick input is
        // being made.
        // Otherwise, it will always drive the desiredHeading to 0 (neutral joystick
        // position)
        if (Math.abs(averageHeading.getAverage()) > headingThreshold)
        {
            desiredHeading = currentHeading + averageHeading.getAverage();
            // keep heading a positive angle
            if (desiredHeading < 0)
            {
                desiredHeading += (360);
            }
        }
        // get magnitude and direction for the roatation.
        vTheta = IMUAngleProcessing();
    
        //System.out.printf("vTheta: %.4f \n" , vTheta);

        // PID controls the vTheta input to the wheel power equation.
        // vTheta = headingPID.getOutput(currentHeading, desiredHeading );

        // Dimensions will change! What are the dimensions of the test chassis!
        // Change in Constants.java
        // Robot dimensions (example)
        // Length = 24 in
        // Width = 20 in
        //
        // 20
        // ________________
        // | |
        // | |
        // | |
        // | |
        // | | 24
        // | |
        // | |
        // | |
        // ----------------
        // SEE Constants.java

        // Angle from the center of the robot to the top right wheel
        double thetaChassis = Utils.angle(Constants.ROBOT_LENGTH, Constants.ROBOT_WIDTH); // Gets
                                                                                          // the
                                                                                          // angle
                                                                                          // created
                                                                                          // from
                                                                                          // the
                                                                                          // center
                                                                                          // of the
                                                                                          // robot
                                                                                          // to
                                                                                          // the top
                                                                                          // right
                                                                                          // corner

        double magnitude = Utils.limitRange(Utils.magnitude(x, y), 0, 1); // Magnitude of left
                                                                          // joystick movement

        double angle = Utils.normalizeAngle(Utils.angle(x, y) - Math.PI / 2); // Angle of left
                                                                              // joystick

        double rotationMagnitude = vTheta; // Magnitude of right joystick sideways movement

        // Angles of rotation of each wheel
        // Each wheel needs to be perpendicular to the angle from the center to it
        double[] rotationAngles = {thetaChassis - Math.PI / 2, // Angle for first wheel to turn the
                                                               // robot clockwise
                -thetaChassis - Math.PI / 2, // Angle for second wheel " " " " "
                -thetaChassis + Math.PI / 2, // Angle for third wheel " " " " "
                thetaChassis + Math.PI / 2 // Angle for fourth wheel " " " " "
        };

        double[] translationVector = {angle, magnitude}; // Vector that represents the translation
                                                         // of the robot

        // An array of vectors for each wheel for the wheel rotation
        double[][] rotationVectors = new double[4][2];

        // Create a rotation vector for wheel rotation for each one
        for (int i = 0; i < 4; i++)
        {
            rotationVectors[i] = new double[] {rotationAngles[i], rotationMagnitude};
        }

        // An array of vectors for the final movement of each wheel
        double[][] podVectors = new double[4][2];

        // Add the translation and rotation vectors to get the final movement vector
        for (int i = 0; i < 4; i++)
        {
            podVectors[i] = Utils.addVectors(translationVector, rotationVectors[i]);
        }

        double maxVectorMagnitude = 0;

        for (int i = 0; i < 4; i++)
        {
            if (podVectors[i][1] > maxVectorMagnitude)
            {
                maxVectorMagnitude = podVectors[i][1];
            }
        }

        if (maxVectorMagnitude > 1.0)
        {
            for (int i = 0; i < 4; i++)
            {
                podVectors[i][1] /= maxVectorMagnitude;
            }
        }

        // Loop through each swerve pod
        for (int i = 0; i < 4; i++)
        {
            // If we are moving the sticks
            if (magnitude != 0 || rotationMagnitude != 0)
            {
                // Set the angle and speed of each wheel according to the final vectors
                pods[i].setDesiredAngle(podVectors[i][0]);
                pods[i].setDesiredRPM(podVectors[i][1]);
            }
            else // If we are not moving the sticks, set the wheel speed to 0
            {
                pods[i].setDesiredRPM(0);
            }
        }

        // Always call to process PID for turn motors
        for (SwervePod pod : pods)
        {
            pod.processPod();
        }
    }

    /**
     * This method will be called once per scheduler run in Autonomous
     */

    public void autonomousPeriodic()
    {
        double x_l = 0;
        double y_l = 0;
        double x_r = error_x.getDouble(0);

        // if there is new joystick input update the heading otherwise hold the current
        // heading as
        // the setpoint.
        // headding is in radians so just using the +/- 1 from the joystick to add as a
        // bias to the
        // current angle will put the desired head +/- 57 degrees from current. This
        // should be more
        // than enough to move the bot at max rotation speed.
        // The chasing of this setpoint is controled by the PID loop on the vTheta
        // value.

        averageHeading.add(x_r); // average in the current error

        // if the averaged stick input is greater then the headingThreshold go ahead and
        // adjust the heading.
        // This keeps from updating the desiredHeading value if no joystick input is
        // being made.
        // Otherwise, it will always drive the desiredHeading to 0 (neutral joystick
        // position)
        if (Math.abs(averageHeading.getAverage()) > headingThreshold)
        {
            desiredHeading = currentHeading + averageHeading.getAverage();
            // keep heading a positive angle
            if (desiredHeading < 0)
            {
                desiredHeading += (360);
            }
        }
        // get magnitude and direction for the roatation.
        vTheta = IMUAngleProcessing();
    
        // System.out.printf("vTheta: %.4f \n" , vTheta);

        // PID controls the vTheta input to the wheel power equation.
        // vTheta = headingPID.getOutput(currentHeading, desiredHeading );

        // Dimensions will change! What are the dimensions of the test chassis!
        // Change in Constants.java
        // Robot dimensions (example)
        // Length = 24 in
        // Width = 20 in
        //
        // 20
        // ________________
        // | |
        // | |
        // | |
        // | |
        // | | 24
        // | |
        // | |
        // | |
        // ----------------
        // SEE Constants.java

        // Angle from the center of the robot to the top right wheel
        double thetaChassis = Utils.angle(Constants.ROBOT_LENGTH, Constants.ROBOT_WIDTH); // Gets
                                                                                          // the
                                                                                          // angle
                                                                                          // created
                                                                                          // from
                                                                                          // the
                                                                                          // center
                                                                                          // of the
                                                                                          // robot
                                                                                          // to
                                                                                          // the top
                                                                                          // right
                                                                                          // corner

        double magnitude = Utils.limitRange(Utils.magnitude(x_l, y_l), 0, 1); // Magnitude of left
                                                                              // joystick movement

        double angle = Utils.normalizeAngle(Utils.angle(x_l, y_l) - Math.PI / 2); // Angle of left
                                                                                  // joystick

        double rotationMagnitude = vTheta; // Magnitude of right joystick sideways movement

        // Angles of rotation of each wheel
        // Each wheel needs to be perpendicular to the angle from the center to it
        double[] rotationAngles = {thetaChassis - Math.PI / 2, // Angle for first wheel to turn the
                                                               // robot clockwise
                -thetaChassis - Math.PI / 2, // Angle for second wheel " " " " "
                -thetaChassis + Math.PI / 2, // Angle for third wheel " " " " "
                thetaChassis + Math.PI / 2 // Angle for fourth wheel " " " " "
        };

        double[] translationVector = {angle, magnitude}; // Vector that represents the translation
                                                         // of the robot

        // An array of vectors for each wheel for the wheel rotation
        double[][] rotationVectors = new double[4][2];

        // Create a rotation vector for wheel rotation for each one
        for (int i = 0; i < 4; i++)
        {
            rotationVectors[i] = new double[] {rotationAngles[i], rotationMagnitude};
        }

        // An array of vectors for the final movement of each wheel
        double[][] podVectors = new double[4][2];

        // Add the translation and rotation vectors to get the final movement vector
        for (int i = 0; i < 4; i++)
        {
            podVectors[i] = Utils.addVectors(translationVector, rotationVectors[i]);
        }

        double maxVectorMagnitude = 0;

        for (int i = 0; i < 4; i++)
        {
            if (podVectors[i][1] > maxVectorMagnitude)
            {
                maxVectorMagnitude = podVectors[i][1];
            }
        }

        if (maxVectorMagnitude > 1.0)
        {
            for (int i = 0; i < 4; i++)
            {
                podVectors[i][1] /= maxVectorMagnitude;
            }
        }

        // Loop through each swerve pod
        for (int i = 0; i < 4; i++)
        {
            // If we are moving the sticks
            if (magnitude != 0 || rotationMagnitude != 0)
            {
                // Set the angle and speed of each wheel according to the final vectors
                pods[i].setDesiredAngle(podVectors[i][0]);
                pods[i].setDesiredRPM(podVectors[i][1]);
            }
            else // If we are not moving the sticks, set the wheel speed to 0
            {
                pods[i].setDesiredRPM(0);
            }
        }

        // Always call to process PID for turn motors
        for (SwervePod pod : pods)
        {
            pod.processPod();
        }
    }

    /**
     * A function that gives every pod angle
     * 
     * @return An array of every angle of every pod in the order specified above
     */
    public double[] getAngles()
    {
        return new double[] {pod1.getCurrentAngle(), pod2.getCurrentAngle(), pod3.getCurrentAngle(),
                pod4.getCurrentAngle()};
    }

    // grab the imu heading and crunch out the values used for navigation and
    // telemetry.
    // This method produces the heading input component to the motors from the PID
    // that holds the
    // desired angle. The error from the PID is sent to the motors in the vTheta
    // variable.
    private double IMUAngleProcessing()
    {
        // in degrees +/- 0 to 180 where CCW is - and CW is + //TODO Verify CW is
        // negative angle
        double yawAngle = gyro.getYaw();

        // System.out.printf("yawAngle: %.4f  desired: %.4f  curr: %.4f\n", yawAngle, desiredHeading,
                // currentHeading);

        // convert imu angle range to our [0, 360) range
        if (yawAngle < 0)
        {
            currentHeading = yawAngle + 360;
        }
        else
        {
            currentHeading = yawAngle;
        }

        return headingPID.getOutput(currentHeading, desiredHeading);
    }

}
