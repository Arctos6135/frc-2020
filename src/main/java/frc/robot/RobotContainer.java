/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.util.Map;
import java.util.logging.Level;

import com.arctos6135.robotlib.logging.RobotLogger;
import com.arctos6135.robotlib.oi.Rumble;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final IntakeSubsystem intakeSubsystem;

    private final XboxController driverController = new XboxController(Constants.XBOX_DRIVER);
    private final XboxController operatorController = new XboxController(Constants.XBOX_INTAKE);

    private final Rumble errorRumble = new Rumble(driverController, Rumble.SIDE_BOTH, 1, 400, 3);
    private final Rumble warningRumble = new Rumble(driverController, Rumble.SIDE_BOTH, 0.75, 300);

    private final ShuffleboardTab configTab;
    private final ShuffleboardTab driveTab;

    private NetworkTableEntry driveReversedEntry;
    private NetworkTableEntry precisionDriveEntry;
    private SimpleWidget drivetrainMotorStatus;

    private NetworkTableEntry lastError;
    private NetworkTableEntry lastWarning;

    private static RobotLogger logger = new RobotLogger();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain = new Drivetrain(Constants.LEFT_CANSPARKMAX, Constants.LEFT_CANSPARKMAX_FOLLOWER,
                Constants.RIGHT_CANSPARKMAX, Constants.RIGHT_CANSPARKMAX_FOLLOWER);
        limelight = new Limelight();

        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, limelight, driverController, Constants.DRIVE_FWD_REV,
                Constants.DRIVE_LEFT_RIGHT, Constants.AUTO_ALIGN));

        intakeSubsystem = new IntakeSubsystem(Constants.MAIN_ROLLER_TALONSRX, Constants.SOLENOID_CHANNEL_1,
                Constants.SOLENOID_CHANNEL_2);
        intakeSubsystem.setDefaultCommand(new ManualIntake(intakeSubsystem, operatorController,
                Constants.INTAKE_FORWARD_BUTTON, Constants.INTAKE_REVERSE_BUTTON));

        // Configure the button bindings
        configureButtonBindings();

        configTab = Shuffleboard.getTab("Config");
        driveTab = Shuffleboard.getTab("Drive");
        addConfigurableValues();

        // Wait for DS to attach before initializing the logger
        // The roboRIO's system time only gets updated after connecting
        while (!DriverStation.getInstance().isDSAttached()) {
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                // Should never happen
                e.printStackTrace();
                lastError.setString("InterruptedException while waiting for DS");
            }
        }
        initLogger();
    }

    private void addConfigurableValues() {
        // Put the precision factor on the dashboard and make it configurable
        configTab.add("Precision Drive Factor", TeleopDrive.getPrecisionFactor()).withPosition(0, 0).withSize(9, 4)
                // Use a number slider from 0-1
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.0, "max", 1.0)).getEntry()
                // Add a listener to update the value in code once the entry updates
                .addListener(notif -> {
                    TeleopDrive.setPrecisionFactor(notif.value.getDouble());
                }, EntryListenerFlags.kUpdate);
        configTab.add("Ramping Rate", TeleopDrive.getRampingRate()).withWidget(BuiltInWidgets.kNumberSlider)
                .withPosition(9, 0).withSize(9, 4).withProperties(Map.of("min", 0.0, "max", 1.0)).getEntry()
                .addListener(notif -> {
                    TeleopDrive.setRampingRate(notif.value.getDouble());
                }, EntryListenerFlags.kUpdate);
        configTab.add("Align PID", AlignToTarget.getSendable()).withWidget(BuiltInWidgets.kPIDController)
                .withPosition(0, 1).withSize(6, 11);
        configTab.add("Motor Warning Temp.", Constants.MOTOR_WARNING_TEMP).withWidget(BuiltInWidgets.kNumberSlider)
                .withPosition(18, 0).withSize(9, 4).withProperties(Map.of("min", 0.0, "max", 150.0)).getEntry()
                .addListener(notif -> {
                    Constants.MOTOR_WARNING_TEMP = notif.value.getDouble();
                }, EntryListenerFlags.kUpdate);
        configTab.add("Motor Shutoff Temp.", Constants.MOTOR_SHUTOFF_TEMP).withWidget(BuiltInWidgets.kNumberSlider)
                .withPosition(27, 0).withSize(9, 4).withProperties(Map.of("min", 0.0, "max", 150.0)).getEntry()
                .addListener(notif -> {
                    Constants.MOTOR_SHUTOFF_TEMP = notif.value.getDouble();
                }, EntryListenerFlags.kUpdate);

        driveReversedEntry = driveTab.add("Reversed", TeleopDrive.isReversed()).withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 0).withSize(4, 4).getEntry();
        precisionDriveEntry = driveTab.add("Precision", TeleopDrive.isPrecisionDrive()).withPosition(4, 0)
                .withSize(4, 4).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        drivetrainMotorStatus = driveTab.add("DT Motor Status", true).withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(8, 0).withSize(6, 4).withProperties(Map.of("color when true", Constants.COLOR_MOTOR_OK,
                        "color when false", Constants.COLOR_MOTOR_WARNING));
        drivetrain.setOverheatShutoffCallback((motor, temp) -> {
            if (!drivetrain.getOverheatShutoffOverride()) {
                // Make it red
                drivetrainMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_SHUTOFF))
                        .getEntry().setBoolean(false);
                errorRumble.execute();
            }
            getLogger().logError(
                    "Drivetrain motor " + motor.getDeviceId() + " reached overheat shutoff limit at " + temp + "C!");
        });
        drivetrain.setOverheatWarningCallback((motor, temp) -> {
            if (!drivetrain.getOverheatShutoffOverride()) {
                // Make it yellow
                drivetrainMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_WARNING))
                        .getEntry().setBoolean(false);
                warningRumble.execute();
            }
            getLogger().logWarning(
                    "Drivetrain motor " + motor.getDeviceId() + " reached overheat warning at " + temp + "C!");
        });
        drivetrain.setNormalTempCallback(() -> {
            drivetrainMotorStatus.getEntry().setBoolean(true);
        });

        lastError = driveTab.add("Last Error", "").withPosition(37, 0).withSize(20, 4).getEntry();
        lastWarning = driveTab.add("Last Warning", "").withPosition(37, 4).withSize(20, 4).getEntry();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Button reverseDriveButton = new JoystickButton(driverController, Constants.REVERSE_DRIVE_DIRECTION);
        // Button autoAlignButton = new JoystickButton(driverController,
        // Constants.AUTO_ALIGN);
        Button overrideMotorProtectionButton = new JoystickButton(driverController,
                Constants.OVERRIDE_MOTOR_PROTECTION);
        Button toggleIntakeButton = new JoystickButton(operatorController, Constants.INTAKE_TOGGLE);
        Button precisionDriveButton = new JoystickButton(driverController, Constants.PRECISION_DRIVE_TOGGLE);
        // Piston Toggle Code
        toggleIntakeButton.whenPressed(new InstantCommand(() -> {
            // Piston Code
            boolean stateExtension = intakeSubsystem.getPistons();
            if (stateExtension) {
                intakeSubsystem.setPistons(false);
            } else {
                intakeSubsystem.setPistons(true);
            }
        }, intakeSubsystem));
        reverseDriveButton.whenPressed(() -> {
            TeleopDrive.toggleReverseDrive();
            driveReversedEntry.setBoolean(TeleopDrive.isReversed());
            getLogger().logInfo("Drive reverse set to " + TeleopDrive.isReversed());
        });
        precisionDriveButton.whenPressed(() -> {
            TeleopDrive.togglePrecisionDrive();
            precisionDriveEntry.setBoolean(TeleopDrive.isPrecisionDrive());
            getLogger().logInfo(TeleopDrive.isPrecisionDrive() ? "Precision drive is ON" : "Precision drive is OFF");
        });
        overrideMotorProtectionButton.whenPressed(() -> {
            boolean override = !drivetrain.getOverheatShutoffOverride();
            drivetrain.setOverheatShutoffOverride(override);
            if (override) {
                // Set the colour to a new one
                drivetrainMotorStatus.withProperties(Map.of("color when true", Constants.COLOR_MOTOR_OVERRIDDEN))
                        .getEntry().setBoolean(true);
                getLogger().logWarning("Motor temperature protection overridden");
            } else {
                // Set the colour back
                drivetrainMotorStatus.withProperties(Map.of("color when true", Constants.COLOR_MOTOR_OK)).getEntry()
                        .setBoolean(!(drivetrain.isOverheating() || drivetrain.isOverheatWarning()));
                getLogger().logInfo("Motor temperature protection re-enabled");
            }
        });
        // toggleIntakeButton.whenPressed()
    }

    private void initLogger() {
        try {
            logger.init(Robot.class);

            // Set logger level
            // Change this to include or exclude information
            logger.setLevel(Level.FINE);
            // Attach log handler to set the last error and warning
            logger.setLogHandler((level, message) -> {
                if (level == Level.SEVERE) {
                    lastError.setString(message);
                } else if (level == Level.WARNING) {
                    lastWarning.setString(message);
                }
            });
            // Clean old logs
            logger.cleanLogs(72);
            logger.logInfo("Logger initialized");
        } catch (IOException e) {
            e.printStackTrace();
            lastError.setString("Failed to init logger!");
        }
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

    /**
     * Get the robot logger.
     * 
     * @return The robot logger
     */
    public static RobotLogger getLogger() {
        return logger;
    }
}
