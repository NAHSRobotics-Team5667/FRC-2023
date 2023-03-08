package frc.robot.subsystems;

import frc.robot.Constants.LightConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public Lights() {
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
    
    /**
    * Sets the color of all LEDs to the specified RGB color
    * @param R Red value (0-255)
    * @param G Green value (0-255)
    * @param B Blue value (0-255)
    */
    public void setSolidRGB(int R, int G, int B) {
        for (var i = 0; i < this.m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values
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
    public void rainbow(double speed_multiplier) {
        final var length = m_ledBuffer.getLength();
        for (var i = 0; i < length; i++) {
            final var hue = (this.rainbowFirstPixelHue + (i * 180 / length)) % 180;
            this.m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        this.rainbowFirstPixelHue += 1 * speed_multiplier;
        this.rainbowFirstPixelHue %= 180;
    }

    /**
     * Calls the rainbow function with the default speed of 3. Use
     * {@link #rainbow(double)} for more control.
     */
    public void rainbow() {
        this.rainbow(3);
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
     * Calls the cylon function with default values for your convenience. Use
     * {@link #cylon(int, int, double)} for more control.
     */
    public void cylon() {
        this.cylon(0, 255, 1);
    }

    public enum period {
        AUTO, TELEOP, DISABLED
    }
    private period currentPeriod = Lights.period.DISABLED;
    public Lights.period getPeriod() {
        return currentPeriod;
    }
        public void setPeriod(Lights.period p) {
        currentPeriod = p;
    }

    @Override
    public void periodic() {

        if (this.getPeriod() == period.DISABLED) {
            rainbow();
        } else {
            cylon();
        }

        
        this.m_led.setData(m_ledBuffer);
    }
    
}
