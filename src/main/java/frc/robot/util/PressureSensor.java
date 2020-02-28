/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * A REV Robotics analog pressure sensor.
 */
public class PressureSensor {

    private static final double SUPPLY_VOLTAGE = 5.0;

    private AnalogInput analogInput;

    /**
     * Create a new pressure sensor.
     * 
     * @param port The analog input port the sensor is on
     */
    public PressureSensor(int port) {
        analogInput = new AnalogInput(port);
    }

    /**
     * Get the pressure reported by the sensor in PSI.
     * 
     * @return The pressure
     */
    public double getPressure() {
        // apply a formula for turning voltage to psi value
        // 250 (voltageOut/supplyVoltage) - 25
        return 250 * (analogInput.getAverageVoltage() / SUPPLY_VOLTAGE) - 25;
    }
}