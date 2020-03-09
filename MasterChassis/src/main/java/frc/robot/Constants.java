package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

public final class Constants
{
    public static double DRIVE_P = 0.0005;
    public static double DRIVE_I = 0.0;
    public static double DRIVE_D = 0.0;
    public static double DRIVE_IZ = 0.0;
    public static double DRIVE_FF = 0.000;
    public static double DRIVE_MAX_OUTPUT = 1.0;
    public static double DRIVE_MIN_OUTPUT = -1.0;
    public static boolean DRIVE_INVERT[] = {true, true, true, true}; 
    public static IdleMode DRIVE_IDLEMODE[] = {IdleMode.kCoast, IdleMode.kCoast, IdleMode.kCoast, IdleMode.kCoast}; 

    public static int DRIVE_MAX_RPM = 18730;
    public static int DRIVE_MAX_CURRENT_STALL = 40;
    public static int DRIVE_MAX_CURRENT_RUN = 30;


    public static double TURN_P = 0.5;  // 0.5 gives a little overshoot on the test stand.
    public static double TURN_I = 0.004;  // 0.004
    public static double TURN_D = 0.7; // 0.7
    public static double OutputLowLimit = -1;
    public static double OutputHighLimit = 1;
    public static double MaxIOutput = 0.5;
    public static double OutputRampRate = 1;
    public static double OutputFilter = 0;
    public static double SetpointRange = 2 * Math.PI;

    public static boolean TURN_INVERT[] = {false, false, false, false};
    public static IdleMode TURN_IDLEMODE[] = {IdleMode.kBrake, IdleMode.kBrake, IdleMode.kBrake, IdleMode.kBrake};

    public static int TURN_MAX_CURRENT_STALL = 30;
    public static int TURN_MAX_CURRENT_RUN = 20;

    public static double ARM_P = 0.01;
    public static double ARM_I = 0.0;
    public static double ARM_D = 0.0;
    public static double ARM_IZ = 0.0;
    public static double ARM_FF = 0.000;
    public static double ARM_MAX_OUTPUT = 1.0;
    public static double ARM_MIN_OUTPUT = -1.0;
    public static boolean ARM_INVERT = true; 
    public static IdleMode ARM_IDLEMODE = IdleMode.kBrake;

    public static int ARM_MAX_CURRENT_STALL = 30;
    public static int ARM_MAX_CURRENT_RUN = 20;

    public static double ARM_POSITION_LOW = 20;
    public static double ARM_POSITION_MID = 25;
    public static double ARM_POSITION_HI  = 30;
    public static double ARM_POSITION_RESET = -50;
       


    public static double WINCH_P = 0.0005;
    public static double WINCH_I = 0.0;
    public static double WINCH_D = 0.0;
    public static double WINCH_IZ = 0.0;
    public static double WINCH_FF = 0.000;
    public static double WINCH_MAX_OUTPUT = 1.0;
    public static double WINCH_MIN_OUTPUT = -1.0;
    public static boolean WINCH_INVERT = false; 
    public static IdleMode WINCH_IDLEMODE = IdleMode.kBrake;

    public static int WINCH_MAX_CURRENT_STALL = 40;
    public static int WINCH_MAX_CURRENT_RUN = 30;




    // Swerve Pod Motor CAN IDs
    public static int POD_1_DRIVE = 0;  //13;       // NEO
    public static int POD_1_TURN = 0; //9;         // 550 mini-NEO

    public static int POD_2_DRIVE = 4;        // NEO
    public static int POD_2_TURN = 8;         // 550 mini-NEO

    public static int POD_3_DRIVE = 16;       // NEO
    public static int POD_3_TURN = 12;        // 550 mini-NEO

    public static int POD_4_DRIVE = 1;        // NEO
    public static int POD_4_TURN = 5;         // 550 mini-NEO

    // Swerve Pod position numbers
    public static int POD_FRONT_LEFT  = 0;
    public static int POD_FRONT_RIGHT = 1;
    public static int POD_BACK_LEFT   = 2;
    public static int POD_BACK_RIGHT  = 3;

    public static double ROBOT_LENGTH = 20.5; // inches
    public static double ROBOT_WIDTH = 20.75; // inches

    //Manipulator Neo Motor IDs
    public static int gatherMotorID = 11;        // 550 mini-NEO
    public static int magazineMotorID = 7;      // 550 mini-NEO
    
    public static int turretMotorID = 3;        // 550 mini-NEO

    public static int shooterMotorID = 2;       // NEO
    public static int breachMotorID = 6;        // 550 mini-NEO

    public static int armMotorID = 9; //10;           // 550 mini-NEO
    public static int winchMotorID = 15;         // NEO 

    public static int colorWheelMotorID = 14;     // 550 mini-NEO 
    
}