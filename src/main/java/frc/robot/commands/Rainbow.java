/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDStrip;

/**
 * A command that makes the LEDs show a moving rainbow effect.
 */
public class Rainbow extends CommandBase {

    private final LEDStrip led;
    private float startHue = 0;

    /**
     * Creates a new Rainbow command.
     * 
     * @param led The LED strip
     */
    public Rainbow(LEDStrip led) {
        this.led = led;
        addRequirements(led);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        float hue = startHue;
        for (int i = 0; i < led.getLength(); i++) {
            led.setColor(i, LEDStrip.applyGamma(LEDStrip.hsvColor8Bit(hue, 1.0f, 1.0f)));
            // Full colour cycle every 10 leds (1/3 of a meter)
            hue += 0.1;
        }
        // 1.0 / 0.005 per iteration / 50 iterations per second = 1 full cycle per 4
        // seconds
        startHue += 0.005;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
