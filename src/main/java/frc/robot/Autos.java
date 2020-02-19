package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.Drivetrain;

/**
 * A class that sets up all the autos.
 */
public class Autos {

    private SendableChooser<AutoMode> chooser;

    /**
     * Valid auto modes.
     */
    public enum AutoMode {
        NONE("None"), INIT_FORWARDS("Initiation Line (Forwards"), INIT_REVERSE("Initiation Line (Reverse)"),
        DEBUG("Debug");

        String name;

        AutoMode(String name) {
            this.name = name;
        }
    }

    /**
     * Get an auto chooser with all the available auto modes.
     * 
     * @return The auto chooser
     */
    public SendableChooser<AutoMode> getChooser() {
        return chooser;
    }

    /**
     * Get the auto command corresponding to an auto mode.
     * 
     * @param mode       The auto mode
     * @param drivetrain The drivetrain
     * @return The command for the auto mode
     */
    public Command getAuto(AutoMode mode, Drivetrain drivetrain) {
        switch (mode) {
            case NONE:
                return null;
            case DEBUG:
                return null;
            case INIT_FORWARDS:
                return new DriveDistance(drivetrain, 36);
            case INIT_REVERSE:
                return new DriveDistance(drivetrain, -36);
            default:
                return null;
        }
    }

    /**
     * Constructor.
     */
    public Autos() {
        // TODO: Is it better to construct trajectories here, or in getAutos()?
        // getAutos() will be called in autonomousInit(), so if it takes too long, the
        // entire robot will stop. On the other hand, a lot of memory can be saved by
        // only generating the trajectories we need.

        chooser = new SendableChooser<>();
        // Add all possible modes
        for (AutoMode mode : AutoMode.class.getEnumConstants()) {
            chooser.addOption(mode.name, mode);
        }
        // Set default mode as none
        chooser.setDefaultOption(AutoMode.NONE.name, AutoMode.NONE);
    }
}
