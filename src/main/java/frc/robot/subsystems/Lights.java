package frc.robot.subsystems;

import frc.robot.Constants.LightConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    public Light_Scheduler scheduler;

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
        scheduler = new Light_Scheduler();
    }

    /**
     * Sets the color of all LEDs to the specified RGB color
     * 
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

    private int flashTimer = 0;

    /**
     * Flashes the LEDs between on and off. Should be called periodically
     * @param R Red value (0-255)
     * @param G Green value (0-255)
     * @param B Blue value (0-255)
     * @param speedMultiplier The speed of the flashing. A multiplier of 1 makes it 2 seconds a cycle.
     */
    public void flashingRGB(int R, int G, int B, double speedMultiplier) {
        double interval = 100 * speedMultiplier; // if multiplier is 1: 2 seconds a cycle aka 1 second on 1 second off.
        if (flashTimer < interval / 2) {
            setSolidRGB(R, G, B);
        } else {
            setSolidRGB(0, 0, 0);
        }
        flashTimer++;
        flashTimer %= interval;
    }

    private int rainbowHueValue = 0;

    /**
     * Creates a rainbow using the LEDs (must be called periodically for cool
     * effect)
     * This is also the example code from the documentation:
     * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#creating-a-rainbow-effect
     */
    public void rainbow(double speed_multiplier) {
        final var length = m_ledBuffer.getLength();
        for (var i = 0; i < length; i++) {
            final var hue = (this.rainbowHueValue + (i * 180 / length)) % 180;
            this.m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        this.rainbowHueValue += 1 * speed_multiplier;
        this.rainbowHueValue %= 180;
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

    @FunctionalInterface
    public static interface LightEffect {
        void apply();
    }

    /**
     * This is the Light Scheduler. It handles the scheduling of light effects.
     * Each stage of the game has a defualt light effect, and this class allows for
     * reactive light effects with the setLightEffect method.
     */

    public class Light_Scheduler {
        LightEffect current_effect;
        double time_left = 0; // in seconds
        boolean periodic;
        LightEffect default_disabled,
                defualt_teleop,
                defualt_auto;

        public Light_Scheduler() {
            this.default_disabled = () -> {
                Lights.this.cylon(0, 255, 1);
            };
            this.defualt_teleop = () -> {
                Lights.this.rainbow(3);
            };
            this.defualt_auto = () -> {
                Lights.this.rainbow(1);
            };
            this.setDefaultLightEffect();
        }

        /**
         * Sets the light effect to the specified effect for the specified duration.
         * 
         * @param effect   The light effect to be applied. see the default effects in
         *                 {@link #Light_Scheduler} for an example of how to do this.
         * @param duration The duration of the light effect in seconds.
         * @param periodic Whether or not the light effect should be applied
         *                 periodically.
         */
        public void setLightEffect(LightEffect effect, double duration, boolean periodic) {
            this.current_effect = effect;
            this.time_left = duration;
            this.periodic = periodic;
            this.applyLightEffect();
        }

        private void applyLightEffect() {
            current_effect.apply();
        }

        /** Sets the default light effect for the current period */
        public void setDefaultLightEffect() {
            period period = Lights.this.getPeriod();
            switch (period) {
                case DISABLED:
                    this.setLightEffect(default_disabled, 0, true);
                    break;
                case AUTO:
                    this.setLightEffect(defualt_auto, 0, true);
                    break;
                case TELEOP:
                    this.setLightEffect(defualt_teleop, 0, true);
                    break;
            }
        }

        /** Method to be executed from the parent classes periodic function */
        public void periodic() {
            if (this.periodic) {
                this.applyLightEffect();
            }
            if (this.time_left > 0) {
                this.time_left -= 0.02; // 0.02 seconds is usually the period of the periodic function
            } else {
                this.setDefaultLightEffect(); // if no time is left for whatever effect, set the default light effect
                                              // for the current period.
            }
        }
    }

    @Override
    public void periodic() {
        this.scheduler.periodic();

        this.m_led.setData(m_ledBuffer);
    }
}
