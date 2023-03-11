package frc.robot.subsystems;

import frc.robot.Constants.LightConstants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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

    private boolean flashing = false;
    /**
     * Flashes the LEDs between on and off. Should be called periodically. Speed is meant to be controlled by the scheduler
     * 
     * @param R               Red value (0-255)
     * @param G               Green value (0-255)
     * @param B               Blue value (0-255)
     */
    public void flashingRGB(int R, int G, int B) {
        if (!this.flashing) {
            setSolidRGB(R, G, B);
        } else {
            setSolidRGB(0, 0, 0);
        }
        this.flashing = !this.flashing;
    }

    private int rainbowHueValue = 0;

    /**
     * Creates a rainbow using the LEDs (must be called periodically for cool
     * effect)
     * This is also the example code from the documentation:
     * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#creating-a-rainbow-effect
     * @param speed_multiplier The speed multiplier of the rainbow. <strong>This is how you speed up the light effect.</strong>
     */
    public void rainbow(int speed_multiplier) {
        final var length = m_ledBuffer.getLength();
        for (var i = 0; i < length; i++) {
            final var hue = (this.rainbowHueValue + (i * 180 / length)) % 180;
            this.m_ledBuffer.setHSV(i, hue, 255, 255);
        }
        this.rainbowHueValue += 1 * speed_multiplier;
        this.rainbowHueValue %= 180;
    }

    private int cylon_center = 0;
    private double cylon_velocity = 1;

    /**
     * Creates a cylon effect using the LEDs (must be called periodically for the
     * effect).
     * 
     * @param hue              The hue of the cylon from [0-180]
     * @param saturation       The saturation of the cylon from [0-255]
     * @param speed_multiplier The speed multiplier of the cylon. <strong>This is how you speed up the light effect.</strong>
     */
    public void cylon(int hue, int saturation, int speed_multiplier) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            int value = 250 - (Math.abs(this.cylon_center - i) * 50);
            if (value < 0) {
                value = 0;
            }
            this.m_ledBuffer.setHSV(i, hue, saturation, value);
        }

        this.cylon_center += (cylon_velocity * speed_multiplier);
        if (this.cylon_center >= this.m_ledBuffer.getLength() || this.cylon_center < 0) {
            this.cylon_velocity *= -1;
        } // makes the cylon go back and forth

    }

    private int carnival_index = 0;

    public void carnival(int[][] colors, double speed_multiplier, int segment_length) {
        int current_color_index = 0;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB((i + carnival_index) % m_ledBuffer.getLength(), colors[current_color_index][0],
                    colors[current_color_index][1], colors[current_color_index][2]);
            if (i % segment_length == 0) {
                current_color_index++;
                current_color_index %= colors.length;
            }
        }
        carnival_index++;

    }

    public enum period {
        AUTO, TELEOP, DISABLED, TEST
    }

    private period currentPeriod = Lights.period.DISABLED;

    public Lights.period getPeriod() {
        return currentPeriod;
    }

    public void setPeriod(Lights.period p) {
        currentPeriod = p;
        scheduler.setDefaultLightEffect(.5);
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
        private int ticks_per_call = 1, tick_counter = 0, test_index = 0;
        double time_left = 0, fade_time_left = 0; // in seconds
        Color[] fade_from, fade_to;
        LightEffect default_disabled, defualt_teleop, defualt_auto;
        
        LightEffect[] tests = new LightEffect[] { 
            //() -> {Lights.this.setSolidRGB(0, 0, 0);},
            //() -> {Lights.this.setSolidRGB(255, 255, 255);}
            () -> {Lights.this.flashingRGB(255, 0, 0);},
            () -> {Lights.this.carnival(new int[][] { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 } }, 1, 3);},
            () -> {Lights.this.rainbow(1);}, 
            () -> {Lights.this.cylon(60, 255, 1);} 
        };

        public Light_Scheduler() {
            this.default_disabled = () -> {
                Lights.this.cylon(0, 255, 1);};
            this.defualt_teleop = () -> {
                Lights.this.rainbow(3);};
            this.defualt_auto = () -> {
                Lights.this.rainbow(1);};
                
            this.setDefaultLightEffect(0);
        }

        /**
         * Sets the light effect to the specified effect for the specified duration.
         * 
         * @param effect   The light effect to be applied. see the default effects in
         *                 {@link #Light_Scheduler} for an example of how to do this.
         * @param duration The duration of the light effect in seconds.
         * @param ticks_per_call How ofted the light effect should be applied. A value of 1 is every .02 seconds. <strong>This is how you slow down the light effect.</strong>
         */
        public void setLightEffect(LightEffect effect, double duration, int ticks_per_call, double fade_duration) {
            this.current_effect = effect;
            this.time_left = duration;
            this.ticks_per_call = ticks_per_call;
            this.fade_time_left = fade_duration;

            if (fade_duration > 0) {
                this.fadeLightEffectInit();
            }else {
                this.current_effect.apply();
            }
        }

        private void fadeLightEffectInit() {
            this.fade_from = new Color[m_ledBuffer.getLength()];
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                this.fade_from[i] = m_ledBuffer.getLED(i);
            }
            this.current_effect.apply();
            this.fade_to = new Color[m_ledBuffer.getLength()];
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                this.fade_to[i] = m_ledBuffer.getLED(i);
            }
            fade_time_elapsed = 0;
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, (int)fade_from[i].red, (int)fade_from[i].green, (int)fade_from[i].blue);
            }
        }

        private double fade_time_elapsed = 0;
        private void fadeLightEffect(){
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                int red = (int) (255*(fade_from[i].red + (fade_time_elapsed*((fade_to[i].red - fade_from[i].red) / (fade_time_left + fade_time_elapsed)))));
                int green = (int) (255*(fade_from[i].green + (fade_time_elapsed*((fade_to[i].green - fade_from[i].green) / (fade_time_left + fade_time_elapsed)))));
                int blue = (int) (255*(fade_from[i].blue + (fade_time_elapsed*((fade_to[i].blue - fade_from[i].blue) / (fade_time_left + fade_time_elapsed)))));
                m_ledBuffer.setRGB(i, red, green, blue);
            }
            fade_time_left -= .02;
            fade_time_elapsed += .02;
        }

        /** Sets the default light effect for the current period */
        public void setDefaultLightEffect(double fade_duration) {
            period period = Lights.this.getPeriod();
            switch (period) {
                case DISABLED:
                    this.setLightEffect(default_disabled, 0, 1,fade_duration);
                    break;
                case AUTO:
                    this.setLightEffect(defualt_auto, 0, 1,fade_duration);
                    break;
                case TELEOP:
                    this.setLightEffect(defualt_teleop, 0, 1, fade_duration);
                    break;
                case TEST:
                    SmartDashboard.putBoolean("TEST", true);
                    this.setLightEffect(this.tests[test_index], 7,1, 1);
                    test_index++; test_index %= tests.length;
                    break;
            }
        }

        /** Method to be executed from the parent classes periodic function */
        public void periodic() {
            if (this.tick_counter++ % this.ticks_per_call == 0 && this.fade_time_left <= 0) {
                this.current_effect.apply();
            }
            if (this.fade_time_left > 0){
                this.fadeLightEffect();
            }
            if (this.time_left <= 0) { 
                if (this.fade_time_left <= 0) {
                    this.tick_counter = 0; // not really necessary but might help debugging
                    this.setDefaultLightEffect(0); // if no time is left for whatever effect, set the default light effect for the current period.
                }
            } else {
                this.time_left-=.02; // 0.02 seconds is usually the period of the periodic function
            }
        }
    }

    @Override
    public void periodic() {
        this.scheduler.periodic();

        this.m_led.setData(m_ledBuffer);

    }
    

}