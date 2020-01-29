/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 * </p>
 * 
 * <b>ALL UNITS ARE IN INCHES AND SECONDS.</b>
 */
public final class Constants {

    // SPARK MAX motors; placeholder port values
    public static final int LEFT_CANSPARKMAX = 1;
    public static final int LEFT_CANSPARKMAX_FOLLOWER = 2;
    public static final int RIGHT_CANSPARKMAX = 3;
    public static final int RIGHT_CANSPARKMAX_FOLLOWER = 4;
    
    // SPARK MAX encoder constants
	public static final int WHEEL_DIAMETER = 6;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double GEARBOX_RATIO = 1 / 10.5; // First stage 84:12 second stage 36:24
    public static final double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static final int COUNTS_PER_REVOLUTION = 42;

    public static double MOTOR_WARNING_TEMP = 70;
    public static double MOTOR_SHUTOFF_TEMP = 90;

    // Xbox Controller constants
	public static final int XBOX_CONTROLLER = 0; 
	public static final int DRIVE_FWD_REV = XboxController.Axis.kLeftY.value;
    public static final int DRIVE_LEFT_RIGHT = XboxController.Axis.kRightX.value;
    public static final int REVERSE_DRIVE_DIRECTION = XboxController.Button.kStickLeft.value;
}
