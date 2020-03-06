/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.arctos6135.robotlib.oi.XboxControllerButtons;
import com.arctos6135.robotpathfinder.core.RobotSpecs;

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

    // SPARK MAX motors
    public static final int LEFT_CANSPARKMAX = 1;
    public static final int LEFT_CANSPARKMAX_FOLLOWER = 2;
    public static final int RIGHT_CANSPARKMAX = 3;
    public static final int RIGHT_CANSPARKMAX_FOLLOWER = 4;

    // Shooter motors
    public static final int SHOOTER_MOTOR_1 = 5;
    public static final int SHOOTER_MOTOR_2 = 6;
    
    // SPARK MAX encoder constants
    public static final int WHEEL_DIAMETER = 6;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double GEARBOX_RATIO = 1 / 10.5; // First stage 84:12 second stage 36:24
    public static final double POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO;
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE * GEARBOX_RATIO / 60;
    public static final int COUNTS_PER_REVOLUTION = 42;

    public static double MOTOR_WARNING_TEMP = 70;
    public static double MOTOR_SHUTOFF_TEMP = 90;

    // Driver Controller constants
    public static final int XBOX_DRIVER = 0;
    public static final int DRIVE_FWD_REV = XboxController.Axis.kLeftY.value;
    public static final int DRIVE_LEFT_RIGHT = XboxController.Axis.kRightX.value;
    public static final int REVERSE_DRIVE_DIRECTION = XboxController.Button.kStickLeft.value;
    public static final int AUTO_ALIGN = XboxController.Button.kA.value;
    public static final int OVERRIDE_MOTOR_PROTECTION = XboxController.Button.kB.value;
    public static final int PRECISION_DRIVE_TOGGLE = XboxController.Button.kX.value;
    public static final int PRECISION_DRIVE_HOLD = XboxController.Axis.kLeftTrigger.value;
    public static final int INTAKE_TOGGLE = XboxController.Button.kY.value;
    public static final int INTAKE_FORWARD_BUTTON = XboxController.Button.kBumperLeft.value;
    public static final int INTAKE_REVERSE_BUTTON = XboxController.Button.kBumperRight.value;
    
    // Operator Controls Related 
    public static final int XBOX_OPERATOR = 1;
    public static final int INDEXER_ONLY_SAME_POV = XboxControllerButtons.POV_LEFT;
    public static final int INDEXER_ONLY_OPPOSITE_POV = XboxControllerButtons.POV_RIGHT;
    public static final int INDEXER_FRONT_ROLLER_SAME_BUTTON = XboxController.Button.kBumperLeft.value;
    public static final int INDEXER_FRONT_ROLLER_OPPOSITE_BUTTON = XboxController.Button.kBumperRight.value;
    public static final int INDEXER_FRONT_ROLLER_SAME_TRIGGER = XboxController.Axis.kLeftTrigger.value;
    public static final int INDEXER_FRONT_ROLLER_OPPOSITE_TRIGGER = XboxController.Axis.kRightTrigger.value;
    public static final int OVERRIDE_RUN_TIGGER = XboxController.Button.kY.value;
    public static final int TOGGLE_OVERRIDE_MODE = XboxController.Button.kStart.value;
    public static final int SHOOT = XboxController.Button.kX.value;
    public static final int BRYCE_FOUR_OVERRIDDE_TOGGLE = XboxController.Button.kStickLeft.value;
    public static final int BRYCE_FOUR_ELEVATOR_CONTROL = XboxController.Axis.kLeftY.value;

    // | Pneumatics Constants
    public static final int INTAKE_SOLENOID_FWD = 2;
    public static final int INTAKE_SOLENOID_REV = 3;
    public static final int PRESSURE_SENSOR_CHANNEL = 0;

    // | Roller Motor Constants
    public static final int TIGGER_BACK_ROLLER = 1;
    public static final int TIGGER_FRONT_ROLLER = 2;
    public static final int INDEXER_LEFT_ROLLER = 3;
    public static final int INDEXER_RIGHT_ROLLER = 4;
    // Photoelectric Sensor place holder values
    public static final int TIGGER_TOP_SENSOR = 0;
    public static final int TIGGER_BOTTOM_SENSOR = 1;

    // | Intake Motor Constants
    public static final int INTAKE_ROLLER_VICTOR = 0;

    public static final double LIMELIGHT_HEIGHT = 34;
    public static final double TARGET_HEIGHT = 115.25;
    public static final double LIMELIGHT_ANGLE = 0;
    // Elevator relvated place holder values
    public static final int BRYCE_FOUR_RIGHT_MOTOR = 5;
    public static final int BRYCE_FOUR_LEFT_MOTOR = 6;

    public static final int COLOR_MOTOR_OK = 0x00FF00FF;
    public static final int COLOR_MOTOR_WARNING = 0xFFFF00FF;
    public static final int COLOR_MOTOR_SHUTOFF = 0xFF0000FF;
    public static final int COLOR_MOTOR_OVERRIDDEN = 0xA72DFFFF;

    // TODO: Change these
    public static final double ROBOT_MAX_VELOCITY = 0;
    public static final double ROBOT_MAX_ACCELERATION = 0;
    public static final double ROBOT_BASE_WIDTH = 24.5;
    public static final RobotSpecs ROBOT_SPECS = new RobotSpecs(ROBOT_MAX_VELOCITY, ROBOT_MAX_ACCELERATION,
            ROBOT_BASE_WIDTH);
    
    // Robot Dimensions
    public static final double BUMPER_WIDTH = 3.5;
    public static final double ROBOT_WIDTH = BUMPER_WIDTH * 2 + 28;
    public static final double ROBOT_LENGTH = BUMPER_WIDTH * 2 + 31.5;
    public static final double ROBOT_LENGTH_INTAKE_DOWN = ROBOT_LENGTH + 12;

    // Field dimensions
    // Most of the following numbers are somewhat approximate
    // They were measured from the CAD and rounded
    public static final double INIT_LINE_DISTANCE = 120;
    public static final double INIT_LINE_TRENCH_DISTANCE = 86.6;
    public static final double TRENCH_DISTANCE_SHORT = 42.2;
    public static final double TRENCH_PC_DISTANCE = 36;
}
