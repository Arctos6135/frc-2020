/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.awt.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A WS2812B individually addressable RGB LED strip.
 */
public class LEDStrip extends SubsystemBase {

    private final AddressableLED leds;
    private final AddressableLEDBuffer ledData;

    private int brightness = 255;

    /**
     * The gamma correction table.
     * 
     * <p>
     * This was generated with a gamma of 2.2. Run the following Python script to 
     * generate:
     * </p>
     * <pre>
     * gamma = 2.2
     * for i in range(0x100):
     *     print("{:3}".format(round(((i / 255) ** gamma) * 255)), end=(',\n' if (i + 1) % 16 == 0 else ', '))
     * </pre>
     */
    public static final int[] gammaTable = {
          0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,
          1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,
          3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6,
          6,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  11,  11,  11,  12,
         12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,
         20,  20,  21,  22,  22,  23,  23,  24,  25,  25,  26,  26,  27,  28,  28,  29,
         30,  30,  31,  32,  33,  33,  34,  35,  35,  36,  37,  38,  39,  39,  40,  41,
         42,  43,  43,  44,  45,  46,  47,  48,  49,  49,  50,  51,  52,  53,  54,  55,
         56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,
         73,  74,  75,  76,  77,  78,  79,  81,  82,  83,  84,  85,  87,  88,  89,  90,
         91,  93,  94,  95,  97,  98,  99, 100, 102, 103, 105, 106, 107, 109, 110, 111,
        113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 127, 129, 130, 132, 133, 135,
        137, 138, 140, 141, 143, 145, 146, 148, 149, 151, 153, 154, 156, 158, 159, 161,
        163, 165, 166, 168, 170, 172, 173, 175, 177, 179, 181, 182, 184, 186, 188, 190,
        192, 194, 196, 197, 199, 201, 203, 205, 207, 209, 211, 213, 215, 217, 219, 221,
        223, 225, 227, 229, 231, 234, 236, 238, 240, 242, 244, 246, 248, 251, 253, 255,
    };

    /**
     * Apply gamma correction to a {@link Color8Bit} by correcting each individual 
     * channel.
     * 
     * @param color The input color
     * @return The color after applying gamma correction
     */
    public static Color8Bit applyGamma(Color8Bit color) {
        return new Color8Bit(gammaTable[color.red], gammaTable[color.green], gammaTable[color.blue]);
    }

    /**
     * Construct a {@link Color8Bit} from an int.
     * 
     * <p>
     * Bits 16-23 are red, 8-15 are green and 0-7 are blue. The alpha channel is
     * ignored.
     * </p>
     * 
     * @param color The input color
     * @return A {@link Color8Bit} representing the same color
     */
    public static Color8Bit intColor8Bit(int color) {
        return new Color8Bit(color & 0x00_FF_00_00 >> 16, color & 0x00_00_FF_00 >> 8, color & 0x00_00_00_FF);
    }

    /**
     * Construct a {@link Color8Bit} from HSV values.
     * 
     * <p>
     * All values should be in the range 0-1.
     * </p>
     * 
     * @param h The hue (0-1)
     * @param s The saturation (0-1)
     * @param v The value (0-1)
     * @return A {@link Color8Bit} with the RGB representation
     */
    public static Color8Bit hsvColor8Bit(float h, float s, float v) {
        return intColor8Bit(Color.HSBtoRGB(h, s, v));
    }

    /**
     * Get the brightness.
     * 
     * <p>
     * The brightness is an integer between 0 and 255 inclusive, with 255 
     * representing full brightness. All RGB values passed to 
     * {@link #setColor(int, Color8Bit)} will first be scaled by this fraction.
     * </p>
     * 
     * @return The brightness
     */
    public int getBrightness() {
        return brightness;
    }

    /**
     * Set the brightness.
     * 
     * <p>
     * The brightness is an integer between 0 and 255 inclusive, with 255 
     * representing full brightness. All RGB values passed to 
     * {@link #setColor(int, Color8Bit)} will first be scaled by this fraction.
     * </p>
     * 
     * <p>
     * This function has no effect on existing LED brightnesses.
     * </p>
     * 
     * @param brightness The brightness
     */
    public void setBrightness(int brightness) {
        this.brightness = brightness;
    }

    /**
     * Get the length of this LED strip.
     * 
     * @return The length of the strip
     */
    public int getLength() {
        return ledData.getLength();
    }

    /**
     * Set the colour of an LED in the strip.
     * 
     * <p>
     * All RGB values are first scaled by the brightness, and then applied.
     * </p>
     * 
     * @param index The index of the LED
     * @param color The colour to set to
     * @see #setBrightness(int)
     */
    public void setColor(int index, Color8Bit color) {
        ledData.setLED(index, new Color8Bit(
                color.red * brightness / 255, color.green * brightness / 255, color.blue * brightness / 255));
    }

    /**
     * Get the colour of the LED at the specified index.
     * 
     * @param index The index of the LED
     * @return The colour of the LED
     */
    public Color8Bit getColor(int index) {
        return ledData.getLED8Bit(index);
    }

    /**
     * Create a new LED strip.
     * 
     * @param port   The PWM port the strip is connected to; <b>must be on one of
     *               the PWM headers, not DIO or MXP</b>
     * @param length The number of LEDs in the strip
     */
    public LEDStrip(int port, int length) {
        leds = new AddressableLED(port);
        leds.setLength(length);
        ledData = new AddressableLEDBuffer(length);
        
        leds.setData(ledData);
        // Start writing the output
        leds.start();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
