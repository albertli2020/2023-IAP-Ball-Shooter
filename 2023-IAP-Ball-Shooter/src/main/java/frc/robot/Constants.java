// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static final int leftFlyWheelID = 16;
    public static final int rightFlyWheelID = 17;
    
    public static final int joystickPort = 2;
    public static final int portThree = 3;
    public static final int portFour = 4;

    public static final int USBPort = 0;

    public static final int encoderTicks = 4096;
    public static final class leftFlyWheelPIDConstants{
        public static double kp = 0.007;
        public static double ki = 0;
        public static double kd = 0;
    }
    public static final class rightFlyWheelPIDConstants{
        public static double kp = 0.007;
        public static double ki = 0;
        public static double kd = 0;
    }
    public static final class leftFlyWheelFF{
        public static final double kS = 0.41733;
        public static final double kV = 0.4025;
        public static final double kA = 0.046839;
    }
    public static final class rightFlyWheelFF{
        public static final double kS = 0.41733;
        public static final double kV = 0.4025;
        public static final double kA = 0.046839;
    }

}
