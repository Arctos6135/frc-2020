/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

	public static final int LEFT_TALONSRX = 1;
	public static final int RIGHT_TALONSRX = 2;

	//Xbox Controls
	public static final double DEADZONE = 0.15;
	public static final int LSTICK_X_AXIS = 0;
	public static final int LSTICK_Y_AXIS = 1;
	public static final int RSTICK_X_AXIS = 4;
	public static final int RSTICK_Y_AXIS = 5;
	public static final int DRIVE_FWD_REV = LSTICK_Y_AXIS;
	public static final int DRIVE_LEFT_RIGHT = RSTICK_X_AXIS;

}
