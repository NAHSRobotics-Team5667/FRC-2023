package frc.robot.subsystems;
import frc.robot.Constants.LightConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue = 0;
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
    public void setSolidRGB(int R, int G, int B){
        for (var i = 0; i < this.m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            this.m_ledBuffer.setRGB(i, R, G, B);
        }
    }

    /*
     * Creates a rainbow using the LEDs (must be called periodically for cool effect)
     * This is also the example code from the documentation: https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#creating-a-rainbow-effect
     */
    private void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (this.m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            this.m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        this.m_rainbowFirstPixelHue += 3;
        // Check bounds
        this.m_rainbowFirstPixelHue %= 180;
    }
    
    
    @Override
    public void periodic() {
        rainbow();
        this.m_led.setData(m_ledBuffer);
    }
}
