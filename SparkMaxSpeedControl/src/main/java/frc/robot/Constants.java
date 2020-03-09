package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

public final class Constants
{
    public static double DRIVE_P = 0.001;
    public static double DRIVE_I = 0.0;
    public static double DRIVE_D = 0.0;
    public static double DRIVE_IZ = 0.0;
    public static double DRIVE_FF = 0.000;
    public static double DRIVE_MAX_OUTPUT = 1.0;
    public static double DRIVE_MIN_OUTPUT = -1.0;
    public static boolean DRIVE_INVERT[] = {true, false, false, true}; //TODO: fix for later motor values
    public static IdleMode DRIVE_IDLEMODE[] = {IdleMode.kCoast, IdleMode.kCoast, IdleMode.kCoast, IdleMode.kCoast}; 

    public static int DRIVE_MAX_RPM = 18730;
    public static int DRIVE_MAX_CURRENT_STALL = 40;
    public static int DRIVE_MAX_CURRENT_RUN = 30;
}