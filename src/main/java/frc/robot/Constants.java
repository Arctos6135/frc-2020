/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.arctos6135.robotlib.oi.XboxControllerButtons;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // SPARK MAX motors; placeholder port values
    public static final int RIGHT_CANSPARKMAX = 1;
    public static final int LEFT_CANSPARKMAX = 2;
    public static final int RIGHT_CANSPARKMAX_FOLLOWER = 3;
    public static final int LEFT_CANSPARKMAX_FOLLOWER = 4;
    
    // SPARK MAX encoder constants
	public static final int WHEEL_DIAMETER = 6; // in inches
	public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE;
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE;
    public static final int COUNT_PER_REVOLUTION = 42;


	// Xbox Controller constants
	public static final int XBOX_CONTROLLER = 0; // placeholder controller port
	public static final int DRIVE_FWD_REV = XboxControllerButtons.LSTICK_Y_AXIS;
	public static final int DRIVE_LEFT_RIGHT = XboxControllerButtons.RSTICK_X_AXIS;
}
