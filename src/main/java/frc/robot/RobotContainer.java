/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.logging.Level;

import com.arctos6135.robotlib.logging.RobotLogger;
import com.arctos6135.robotlib.newcommands.triggers.AnalogTrigger;
import com.arctos6135.robotlib.oi.Rumble;
import com.arctos6135.stdplug.api.StdPlugWidgets;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.IndexerTiggerCommand;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IndexerTiggerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
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
    private final IntakeSubsystem intakeSubsystem;
    private final Shooter shooter;
    private final IndexerTiggerSubsystem indexerTiggerSubsystem;

    private static final XboxController driverController = new XboxController(Constants.XBOX_DRIVER);
    private static final XboxController operatorController = new XboxController(Constants.XBOX_OPERATOR);

    public static final Rumble infoRumbleDriver = new Rumble(driverController, Rumble.SIDE_BOTH, 1, 200);
    public static final Rumble errorRumbleDriver = new Rumble(driverController, Rumble.SIDE_BOTH, 1, 400, 3);
    public static final Rumble warningRumbleDriver = new Rumble(driverController, Rumble.SIDE_BOTH, 0.75, 300);
    public static final Rumble infoRumbleOperator = new Rumble(operatorController, Rumble.SIDE_BOTH, 1, 200);
    public static final Rumble errorRumbleOperator = new Rumble(operatorController, Rumble.SIDE_BOTH, 1, 400, 3);
    public static final Rumble warningRumbleOperator = new Rumble(operatorController, Rumble.SIDE_BOTH, 0.75, 300);

    private final ShuffleboardTab configTab;
    private final ShuffleboardTab driveTab;
    private final ShuffleboardTab prematchTab;
    private final ShuffleboardTab debugTab;

    private NetworkTableEntry driveReversedEntry;
    private NetworkTableEntry precisionDriveEntry;
    private NetworkTableEntry overrideModeEntry;
    private NetworkTableEntry shooterRPMEntry;
    private NetworkTableEntry tiggerPowerCellCountEntry;
    private SimpleWidget drivetrainMotorStatus;
    private SimpleWidget shooterMotorStatus;

    private SendableChooser<Integer> tiggerPowerCellChooser = new SendableChooser<>();

    private NetworkTableEntry lastError;
    private NetworkTableEntry lastWarning;

    private static RobotLogger logger = new RobotLogger();

    private Autos autos;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain = new Drivetrain(Constants.LEFT_CANSPARKMAX, Constants.LEFT_CANSPARKMAX_FOLLOWER,
                Constants.RIGHT_CANSPARKMAX, Constants.RIGHT_CANSPARKMAX_FOLLOWER);

        shooter = new Shooter(Constants.SHOOTER_MOTOR_1, Constants.SHOOTER_MOTOR_2);

        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, shooter.getLimelight(), driverController,
                Constants.DRIVE_FWD_REV, Constants.DRIVE_LEFT_RIGHT, Constants.AUTO_ALIGN));

        intakeSubsystem = new IntakeSubsystem(Constants.INTAKE_ROLLER_VICTOR, Constants.SOLENOID_CHANNEL_1,
                Constants.SOLENOID_CHANNEL_2);
        intakeSubsystem.setDefaultCommand(new ManualIntake(intakeSubsystem, driverController,
                Constants.INTAKE_FORWARD_BUTTON, Constants.INTAKE_REVERSE_BUTTON));

        // Construct here
        // There's a chance this may take a lot of time
        autos = new Autos();

        indexerTiggerSubsystem = new IndexerTiggerSubsystem(Constants.TIGGER_BACK_ROLLER, Constants.TIGGER_FRONT_ROLLER,
                Constants.TIGGER_BOTTOM_SENSOR, Constants.TIGGER_TOP_SENSOR, Constants.INDEXER_LEFT_ROLLER,
                Constants.INDEXER_RIGHT_ROLLER);
        indexerTiggerSubsystem.setDefaultCommand(new IndexerTiggerCommand(indexerTiggerSubsystem, operatorController));

        configTab = Shuffleboard.getTab("Config");
        driveTab = Shuffleboard.getTab("Drive");
        prematchTab = Shuffleboard.getTab("Pre-match");
        debugTab = Shuffleboard.getTab("Debug");
        configureDashboard();

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

        // Configure the button bindings
        configureButtonBindings();

        // Try to load the shooter range table
        try {
            shooter.loadRangeTable("rangetable.csv");
        } catch (Exception e) {
            getLogger().logError("Error loading range table: " + e.getMessage());
        }
    }

    /**
     * Add stuff to the dashboard.
     */
    private void configureDashboard() {
        // Add stuff to the dashboard to make them configurable
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
                .withPosition(0, 4).withSize(6, 11);
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
        configTab.add("Follower Gains", FollowTrajectory.getGains()).withWidget(StdPlugWidgets.PIDVA_GAINS)
                .withPosition(12, 4).withSize(6, 11).withProperties(Map.of("Show kDP", true));
        configTab.add("Update Delay", FollowTrajectory.getUpdateDelay()).withWidget(BuiltInWidgets.kTextView)
                .withPosition(18, 4).withSize(5, 4).getEntry().addListener(notif -> {
                    FollowTrajectory.setUpdateDelay(notif.value.getDouble());
                }, EntryListenerFlags.kUpdate);
        configTab.add("Shooter PID", new Shooter.SendableCANPIDController(shooter.getPIDController()))
                .withWidget(BuiltInWidgets.kPIDController).withPosition(6, 4).withSize(6, 11);

        driveTab.add("Gyro", drivetrain.getAHRS()).withWidget(BuiltInWidgets.kGyro).withPosition(0, 4).withSize(9, 10);
        driveReversedEntry = driveTab.add("Reversed", TeleopDrive.isReversed()).withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 0).withSize(4, 4).getEntry();
        precisionDriveEntry = driveTab.add("Precision", TeleopDrive.isPrecisionDrive()).withPosition(4, 0)
                .withSize(4, 4).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        overrideModeEntry = driveTab.add("Override", IndexerTiggerCommand.getOverrideMode())
                .withWidget(BuiltInWidgets.kBooleanBox).withPosition(9, 4).withSize(4, 4).getEntry();
        shooterRPMEntry = driveTab.add("Shooter RPM", 0).withWidget(BuiltInWidgets.kDial).withPosition(37, 8)
                .withSize(6, 6).withProperties(Map.of("min", 0, "max", 5000)).getEntry();
        tiggerPowerCellCountEntry = driveTab.add("Tigger PCs", 0).withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(43, 8).withSize(5, 6)
                .withProperties(Map.of("min", 0, "max", 3, "center", 0, "num tick marks", 4, "show text", false))
                .getEntry();
        // Add the motor status boolean boxes
        drivetrainMotorStatus = driveTab.add("Drivetrain", true).withWidget(BuiltInWidgets.kBooleanBox)
                // Set the size and custom colours
                .withPosition(8, 0).withSize(4, 4).withProperties(Map.of("color when true", Constants.COLOR_MOTOR_OK,
                        "color when false", Constants.COLOR_MOTOR_WARNING));
        shooterMotorStatus = driveTab.add("Shooter", true).withWidget(BuiltInWidgets.kBooleanBox).withPosition(12, 0)
                .withSize(4, 4).withProperties(Map.of("color when true", Constants.COLOR_MOTOR_OK, "color when false",
                        Constants.COLOR_MOTOR_WARNING));
        shooter.getLimelight().setStreamingMode(Limelight.StreamingMode.STANDARD);
        driveTab.add("Camera Stream", Limelight.STREAM_URL).withWidget(StdPlugWidgets.MJPEG_STREAM_VIEWER)
                .withPosition(16, 0).withSize(21, 14);
        // Overheat shutoff
        // Should log error, rumble and make the status box red
        drivetrain.getMonitorGroup().setOverheatShutoffCallback((motor, temp) -> {
            if (!drivetrain.getOverheatShutoffOverride()) {
                // Make it red
                drivetrainMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_SHUTOFF))
                        .getEntry().setBoolean(false);
                errorRumbleDriver.execute();
            }
            getLogger().logError(
                    "Drivetrain motor " + motor.getDeviceId() + " reached overheat shutoff limit at " + temp + "C!");
        });
        shooter.getMonitorGroup().setOverheatShutoffCallback((motor, temp) -> {
            if (!shooter.getOverheatShutoffOverride()) {
                shooterMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_SHUTOFF)).getEntry()
                        .setBoolean(false);
                errorRumbleOperator.execute();
            }
            getLogger().logError(
                    "Shooter motor " + motor.getDeviceId() + " reached overheat shutoff limit at " + temp + "C!");
        });
        // Overheat warning
        // Should log warning, rumble and make the box yellow
        drivetrain.getMonitorGroup().setOverheatWarningCallback((motor, temp) -> {
            if (!drivetrain.getOverheatShutoffOverride()) {
                // Make it yellow
                drivetrainMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_WARNING))
                        .getEntry().setBoolean(false);
                warningRumbleDriver.execute();
            }
            getLogger().logWarning(
                    "Drivetrain motor " + motor.getDeviceId() + " reached overheat warning at " + temp + "C!");
        });
        shooter.getMonitorGroup().setOverheatWarningCallback((motor, temp) -> {
            if (!shooter.getOverheatShutoffOverride()) {
                shooterMotorStatus.withProperties(Map.of("color when false", Constants.COLOR_MOTOR_WARNING)).getEntry()
                        .setBoolean(false);
                warningRumbleOperator.execute();
            }
            getLogger().logWarning(
                    "Shooter motor " + motor.getDeviceId() + " reached overheat warning at " + temp + " C!");
        });
        // Return to normal callback
        // Should just clear the box and set it to green
        drivetrain.getMonitorGroup().setNormalTempCallback(() -> {
            drivetrainMotorStatus.getEntry().setBoolean(true);
        });
        shooter.getMonitorGroup().setNormalTempCallback(() -> {
            shooterMotorStatus.getEntry().setBoolean(true);
        });

        // Add auto chooser
        prematchTab.add("Auto Mode", autos.getChooser()).withPosition(0, 0).withSize(9, 5);
        tiggerPowerCellChooser.setDefaultOption("0", 0);
        tiggerPowerCellChooser.addOption("1", 1);
        tiggerPowerCellChooser.addOption("2", 2);
        tiggerPowerCellChooser.addOption("3", 3);
        prematchTab.add("Preload", tiggerPowerCellChooser).withPosition(9, 0).withSize(5, 5);

        lastError = driveTab.add("Last Error", "").withPosition(37, 0).withSize(20, 4).getEntry();
        lastWarning = driveTab.add("Last Warning", "").withPosition(37, 4).withSize(20, 4).getEntry();

        debugTab.add(drivetrain).withPosition(0, 0).withSize(19, 15);
    }

    /**
     * Update the dashboard.
     */
    public void updateDashboard() {
        shooterRPMEntry.setNumber(shooter.getVelocity());
        tiggerPowerCellCountEntry.setNumber(indexerTiggerSubsystem.getPowercellCount());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Button reverseDriveButton = new JoystickButton(driverController, Constants.REVERSE_DRIVE_DIRECTION);
        Button dtOverheatOverrideButton = new JoystickButton(driverController, Constants.OVERRIDE_MOTOR_PROTECTION);
        Button shooterOverheatOverrideButton = new JoystickButton(operatorController,
                Constants.OVERRIDE_MOTOR_PROTECTION);
        Button toggleIntakeButton = new JoystickButton(driverController, Constants.INTAKE_TOGGLE);
        Button precisionDriveButton = new JoystickButton(driverController, Constants.PRECISION_DRIVE_TOGGLE);
        AnalogTrigger precisionDriveTrigger = new AnalogTrigger(driverController, Constants.PRECISION_DRIVE_HOLD, 0.5);
        Button operatorOverrideModeButton = new JoystickButton(operatorController, Constants.TOGGLE_OVERRIDE_MODE);
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
        operatorOverrideModeButton.whenPressed(() -> {
            boolean override = !IndexerTiggerCommand.getOverrideMode();
            overrideModeEntry.setBoolean(override);
            IndexerTiggerCommand.setOverrideMode(override);
            if (override) {
                indexerTiggerSubsystem.setPowercellCount(0);
            }
        });
        reverseDriveButton.whenPressed(() -> {
            TeleopDrive.toggleReverseDrive();
            driveReversedEntry.setBoolean(TeleopDrive.isReversed());
            getLogger().logInfo("Drive reverse set to " + TeleopDrive.isReversed());
        });
        precisionDriveButton.whenPressed(() -> {
            TeleopDrive.togglePrecisionDrive();
            precisionDriveEntry.setBoolean(TeleopDrive.isPrecisionDrive());
        });
        precisionDriveTrigger.setMinTimeRequired(0.05);
        precisionDriveTrigger.whileActiveOnce(new FunctionalCommand(() -> {
            TeleopDrive.togglePrecisionDrive();
            precisionDriveEntry.setBoolean(TeleopDrive.isPrecisionDrive());
        }, () -> {}, (interrupted) -> {
            TeleopDrive.togglePrecisionDrive();
            precisionDriveEntry.setBoolean(TeleopDrive.isPrecisionDrive());
        }, () -> false));
        // Override overheat protection
        // This should set the colour to purple if overridden, otherwise it should
        // restore the correct colour
        dtOverheatOverrideButton.whenPressed(() -> {
            boolean override = !drivetrain.getOverheatShutoffOverride();
            drivetrain.setOverheatShutoffOverride(override);
            if (override) {
                // Set the colour to a new one
                drivetrainMotorStatus.withProperties(Map.of("color when true", Constants.COLOR_MOTOR_OVERRIDDEN))
                        .getEntry().setBoolean(true);
                getLogger().logWarning("Drivetrain motor temperature protection overridden");
            } else {
                // Set the colour back
                drivetrainMotorStatus.withProperties(Map.of("color when true", Constants.COLOR_MOTOR_OK)).getEntry()
                        .setBoolean(!(drivetrain.getMonitorGroup().getOverheatShutoff()
                                || drivetrain.getMonitorGroup().getOverheatWarning()));
                getLogger().logInfo("Drivetrain motor temperature protection re-enabled");
            }
        });
        shooterOverheatOverrideButton.whenPressed(() -> {
            boolean override = !shooter.getOverheatShutoffOverride();
            shooter.setOverheatShutoffOverride(override);
            if (override) {
                shooterMotorStatus.withProperties(Map.of("color when true", Constants.COLOR_MOTOR_OVERRIDDEN))
                        .getEntry().setBoolean(true);
                getLogger().logWarning("Shooter motor temperature protection overridden");
            } else {
                shooterMotorStatus.withProperties(Map.of("color when true", Constants.COLOR_MOTOR_OK)).getEntry()
                        .setBoolean(!(shooter.getMonitorGroup().getOverheatShutoff()
                                || shooter.getMonitorGroup().getOverheatWarning()));
                getLogger().logInfo("Shooter motor temperature protection re-enabled");
            }
        });
    }

    private void initLogger() {
        try {
            logger.init(Robot.class,
                    new File(Filesystem.getOperatingDirectory().getCanonicalPath() + "/frc-robot-logs"));

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
        indexerTiggerSubsystem.setPowercellCount(tiggerPowerCellChooser.getSelected());
        return autos.getAuto(autos.getChooser().getSelected(), drivetrain, indexerTiggerSubsystem, shooter);
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
