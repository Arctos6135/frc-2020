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
 * A command that makes the LEDs show a moving sine wave effect with a set hue
 * and varying brightness.
 */
public class SineColorWave extends CommandBase {

    private final LEDStrip led;
    private final float hue;
    private float startTime = 0;

    /**
     * Creates a new SineColorWave command.
     * 
     * @param led The LED strip
     */
    public SineColorWave(LEDStrip led, float hue) {
        this.led = led;
        this.hue = hue;
        addRequirements(led);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        float time = startTime;
        for (int i = 0; i < led.getLength(); i++) {
            led.setColor(i, LEDStrip.applyGamma(LEDStrip.hsvColor8Bit(
                    hue, 1.0f, (float) ((Math.sin(time) + 1) / 2))));
            // 2pi/20
            // Full cycle every 10 leds (1/3 of a meter)
            time += 0.3141592653589793;
        }
        // 2pi / (2pi/50) per iteration / 50 iterations per second = 1 full cycle per second
        startTime += 0.12566370614359174;
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
