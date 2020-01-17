/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final Drivetrain drivetrain;

    private static XboxController driverController;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        drivetrain = new Drivetrain(Constants.LEFT_CANSPARKMAX, Constants.LEFT_CANSPARKMAX_FOLLOWER,
                Constants.RIGHT_CANSPARKMAX, Constants.RIGHT_CANSPARKMAX_FOLLOWER);
        drivetrain.setDefaultCommand(
                new TeleopDrive(drivetrain, driverController, Constants.DRIVE_FWD_REV, Constants.DRIVE_LEFT_RIGHT));

        // Configure the button bindings
        configureButtonBindings();

        // Put the precision factor on the dashboard and make it configurable
        ShuffleboardTab configTab = Shuffleboard.getTab("Config");
        configTab.add("Precision Drive Factor", TeleopDrive.getPrecisionFactor())
                // Use a number slider from 0-1
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.0, "max", 1.0)).getEntry()
                // Add a listener to update the value in code once the entry updates
                .addListener(notif -> {
                    TeleopDrive.setPrecisionFactor(notif.value.getDouble());
                }, EntryListenerFlags.kUpdate);
        // Do the same with the ramping rate
        configTab.add("Ramping Rate", TeleopDrive.getRampingRate()).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 3.0)).getEntry().addListener(notif -> {
                    TeleopDrive.setRampingRate(notif.value.getDouble());
                }, EntryListenerFlags.kUpdate);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        driverController = new XboxController(Constants.XBOX_CONTROLLER);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TODO
        return new InstantCommand();
    }
}
