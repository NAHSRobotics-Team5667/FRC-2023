package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.Constants.LightConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private Robot robot;
    public Lights(Robot robot) {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        this.m_led = new AddressableLED(LightConstants.kLEDPort);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        this.m_ledBuffer = new AddressableLEDBuffer(150);
        this.m_led.setLength(m_ledBuffer.getLength());
        this.setSolidRGB(0, 255, 0);
        // Set the data
        this.m_led.setData(m_ledBuffer);
        this.m_led.start();

    }

    public void setSolidRGB(int R, int G, int B) {
        for (var i = 0; i < this.m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            this.m_ledBuffer.setRGB(i, R, G, B);
        }
    }

    private int rainbowFirstPixelHue = 0;

    /**
     * Creates a rainbow using the LEDs (must be called periodically for cool
     * effect)
     * This is also the example code from the documentation:
     * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#creating-a-rainbow-effect
     */
    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (this.rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            this.m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        this.rainbowFirstPixelHue += 3;
        this.rainbowFirstPixelHue %= 180;
    }

    private int cylon_center = 0;
    private double cylon_velocity = 1;

    /**
     * Creates a cylon effect using the LEDs (must be called periodically for the
     * effect).
     * 
     * @param hue              The hue of the cylon from [0-180]
     * @param saturation       The saturation of the cylon from [0-255]
     * @param speed_multiplier The speed multiplier of the cylon
     */
    public void cylon(int hue, int saturation, double speed_multiplier) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            int value = 250 - (Math.abs(this.cylon_center - i) * 50);
            if (value < 0) {
                value = 0;
            }
            this.m_ledBuffer.setHSV(i, hue, saturation, value);
        }

        this.cylon_center += cylon_velocity;
        if (this.cylon_center >= this.m_ledBuffer.getLength() || this.cylon_center < 0) {
            this.cylon_velocity *= -1;
        } // makes the cylon go back and forth

    }

    /**
     * Calls the cylon function with default values for your convenience.
     * Use {@link #cylon(int, int, double)} for more control
     */
    public void cylon() {
        this.cylon(0, 255, 1);
    }

    @Override
    public void periodic() {
        System.out.println(this.robot.getPeriod()); // TODO: test and remove this. I'm done for the night

        // rainbow();
        cylon();
        this.m_led.setData(m_ledBuffer);
    }
}
