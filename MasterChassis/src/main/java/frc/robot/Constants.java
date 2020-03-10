package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

public final class Constants
{
    // PID for the gyro heading.
    public static double HEADING_P = 0.005;  
    public static double HEADING_I = 0.0;  
    public static double HEADING_D = 0.0; 
    public static double HEADING_OUTPUT_LIMIT_LOW = -1;
    public static double HEADING_OUTPUT_LIMIT_HIGH = 1;
    public static double HEADING_MAX_I_OUT = 1;
    public static double HEADING_OUTPUT_RAMPRATE = 0.1;
    public static double HEADING_OUTPUT_FILTER = 0;
    public static double HEADING_SETPOINT_RANGE = 360;


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

    public static double GATHER_RPM = 500;
    public static double MAGAZINE_RPM = 300;

    public static double MAGAZINE_REVERSE_TIME = 2; // TODO: (In seconds) change to real value
    public static double MAGAZINE_LOAD_TIME = 1; // TODO: (In seconds) change to real value

    public static double TURN_P = 0.5;  // 0.5 gives a little overshoot on the test stand.
    public static double TURN_I = 0.004;  // 0.004
    public static double TURN_D = 0.7; // 0.7
    public static double TURN_OUTPUT_LIMIT_LOW = -1;
    public static double TURN_OUTPUT_LIMIT_HIGH = 1;
    public static double TURN_MAX_I_OUT = 0.5;
    public static double TURN_OUTPUT_RAMPRATE = 1;
    public static double TURN_OUTPUT_FILTER = 0;
    public static double TURN_SETPOINT_RANGE = 2 * Math.PI;

    public static boolean TURN_INVERT[] = {false, false, false, false};
    public static IdleMode TURN_IDLEMODE[] = {IdleMode.kBrake, IdleMode.kBrake, IdleMode.kBrake, IdleMode.kBrake};

    public static int TURN_MAX_CURRENT_STALL = 30;
    public static int TURN_MAX_CURRENT_RUN = 20;


    public static double ARM_P = 0.02;
    public static double ARM_I = 0.0;
    public static double ARM_D = 0.0;
    public static double ARM_IZ = 0.0;
    public static double ARM_FF = 0.000;
    public static double ARM_MAX_OUTPUT = 1.0;
    public static double ARM_MIN_OUTPUT = -0.3;
    public static boolean ARM_INVERT = true; 
    public static IdleMode ARM_IDLEMODE = IdleMode.kBrake;

    public static int ARM_MAX_CURRENT_STALL = 30;
    public static int ARM_MAX_CURRENT_RUN = 20;

    public static double ARM_POSITION_LOW = 150;
    public static double ARM_POSITION_MID = 250;
    public static double ARM_POSITION_HIGH = 360;
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

    
    public static double COLORWHEEL_P = 0.01;
    public static double COLORWHEEL_I = 0.0;
    public static double COLORWHEEL_D = 0.0;
    public static double COLORWHEEL_IZ = 0.0;
    public static double COLORWHEEL_FF = 0.000;
    public static double COLORWHEEL_MAX_OUTPUT = 1.0;
    public static double COLORWHEEL_MIN_OUTPUT = -0.4;
    public static boolean COLORWHEEL_INVERT = true; 
    public static IdleMode COLORWHEEL_IDLEMODE = IdleMode.kBrake;

    public static int COLORWHEEL_MAX_CURRENT_STALL = 30;
    public static int COLORWHEEL_MAX_CURRENT_RUN = 20;



    public static double GATHER_P = 0.01;
    public static double GATHER_I = 0;
    public static double GATHER_D = 0;
    public static double GATHER_IZ = 0.0;
    public static double GATHER_FF = 0.0;
    public static double GATHER_OUTPUT_MIN = -1;
    public static double GATHER_OUTPUT_MAX = 1;
    public static int GATHER_STALL_CURRENT = 19; // 20 amp fuse
    public static int GATHER_FREE_CURRENT = 19;

    public static double MAGAZINE_P = 0.0002;
    public static double MAGAZINE_I = 0;
    public static double MAGAZINE_D = 0;
    public static double MAGAZINE_IZ = 0.0;
    public static double MAGAZINE_FF = 0.0;
    public static double MAGAZINE_OUTPUT_MIN = -1;
    public static double MAGAZINE_OUTPUT_MAX = 1;
    public static int MAGAZINE_STALL_CURRENT = 19; // 20 amp fuse
    public static int MAGAZINE_FREE_CURRENT = 19;

    // Swerve Pod Motor CAN IDs
    public static int POD_1_DRIVE = 13;       // NEO
    public static int POD_1_TURN = 9;         // 550 mini-NEO 

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
    public static int GATHER_MOTOR_ID = 11;      // 550 mini-NEO
    public static int MAGAZINE_MOTOR_ID = 7;    // 550 mini-NEO
    
    public static int turretMotorID = 3;        // 550 mini-NEO

    public static int shooterMotorID = 2;       // NEO
    public static int breachMotorID = 6;        // 550 mini-NEO

    public static int armMotorID = 10;           // 550 mini-NEO
    public static int winchMotorID = 15;         // NEO 

    public static int colorWheelMotorID = 14;     // 550 mini-NEO 
    
}