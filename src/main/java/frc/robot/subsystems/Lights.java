package frc.robot.subsystems;
import frc.robot.Constants.LightConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int rainbowFirstPixelHue = 0;
    private int cylon_center = 0;
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
            final var hue = (this.rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            this.m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        this.rainbowFirstPixelHue += 3;
        // Check bounds
        this.rainbowFirstPixelHue %= 180;
    }

    private void cylon() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            int v = 250 - (Math.abs(this.cylon_center-i) * 50);
            if (v<0) {
                v = 0;
            }
            this.m_ledBuffer.setHSV(i, 0, 255, v);
            
        }
        this.cylon_center++;
        cylon_center = cylon_center % this.m_ledBuffer.getLength();
    }
    
    @Override
    public void periodic() {
        //rainbow();
        cylon();
        this.m_led.setData(m_ledBuffer);
    }
}
