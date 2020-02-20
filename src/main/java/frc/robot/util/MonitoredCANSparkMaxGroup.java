package frc.robot.util;

import java.util.function.BiConsumer;

import com.revrobotics.CANSparkMax;

import frc.robot.RobotContainer;

/**
 * A group of SPARK MAX motor controllers that allows for temperature and faults
 * monitoring.
 */
public class MonitoredCANSparkMaxGroup {

    private final double SHUTOFF_TEMPERATURE;
    private final double WARNING_TEMPERATURE;

    private CANSparkMax[] motors;
    private final short[] motorFaults;

    private String name;

    private boolean overheatShutoff = false;
    private boolean overheatWarning = false;
    private BiConsumer<CANSparkMax, Double> overheatShutoffCallback;
    private BiConsumer<CANSparkMax, Double> overheatWarningCallback;
    private Runnable normalTempCallback;

    /**
     * Construct a new monitored SPARK MAX group.
     * 
     * @param name        The name of the group
     * @param warningTemp The overheat warning temperature
     * @param shutoffTemp The overheat shutoff temperature
     * @param motors      The motors
     */
    public MonitoredCANSparkMaxGroup(String name, double warningTemp, double shutoffTemp, CANSparkMax... motors) {
        this.motors = motors;
        this.motorFaults = new short[motors.length];
        this.name = name;
        this.WARNING_TEMPERATURE = warningTemp;
        this.SHUTOFF_TEMPERATURE = shutoffTemp;
    }

    /**
     * Get whether any motor is overheating.
     * 
     * <p>
     * If any motor is overheating, the setMotor methods will do nothing.
     * </p>
     * 
     * @return Whether the motors are overheating
     */
    public boolean getOverheatShutoff() {
        return overheatShutoff;
    }

    /**
     * Get whether any motor has triggered the overheat warning.
     * 
     * @return Whether the motors have triggered the warning
     */
    public boolean getOverheatWarning() {
        return overheatWarning;
    }

    /**
     * Set the overheat shutoff callback.
     * 
     * <p>
     * The first argument of the callback is the motor that's overheating, and the
     * second is the temperature.
     * </p>
     * 
     * @param callback The callback
     */
    public void setOverheatShutoffCallback(BiConsumer<CANSparkMax, Double> callback) {
        overheatShutoffCallback = callback;
    }

    /**
     * Set the overheat warning callback.
     * 
     * <p>
     * The first argument of the callback is the motor that's overheating, and the
     * second is the temperature.
     * </p>
     * 
     * @param callback The callback
     */
    public void setOverheatWarningCallback(BiConsumer<CANSparkMax, Double> callback) {
        overheatWarningCallback = callback;
    }

    /**
     * Set the callback to be called when the motor's temperature has returned to
     * normal.
     * 
     * @param callback The callback
     */
    public void setNormalTempCallback(Runnable callback) {
        normalTempCallback = callback;
    }

    /**
     * Runs the monitoring loop once.
     */
    public void monitorOnce() {
        boolean shutoff = false;
        boolean warning = false;
        for (int i = 0; i < motors.length; i++) {
            var motor = motors[i];

            // Check each motor for faults
            short faults = motor.getFaults();
            // If faults have been updated and there is at least one fault
            // Log it as an error
            if (faults != motorFaults[i] && faults != 0) {
                RobotContainer.getLogger().logError(name + " motor " + motor.getDeviceId() + " had fault(s) " + faults);
            }
            motorFaults[i] = faults;

            // Check every motor for temperature
            double temp = motor.getMotorTemperature();

            if (temp >= SHUTOFF_TEMPERATURE) {
                // Only call the callback if not already overheating
                if (!overheatShutoff && overheatShutoffCallback != null) {
                    overheatShutoffCallback.accept(motor, temp);
                }
                shutoff = true;
            } else if (temp >= WARNING_TEMPERATURE) {
                if (!overheatWarning && overheatWarningCallback != null) {
                    overheatWarningCallback.accept(motor, temp);
                }
                warning = true;
            }
        }
        // Run the return to normal callback
        if ((overheatShutoff || overheatWarning) && (!shutoff && !warning) && normalTempCallback != null) {
            normalTempCallback.run();
        }

        overheatShutoff = shutoff;
        overheatWarning = warning;
    }
}
