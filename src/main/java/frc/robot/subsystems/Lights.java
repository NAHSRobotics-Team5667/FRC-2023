package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    public Light_Scheduler scheduler;
    private Color teamColor;

    public Lights(int ledPort, int ledLength) {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        this.led = new AddressableLED(ledPort);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        this.ledBuffer = new AddressableLEDBuffer(ledLength);
        this.led.setLength(ledBuffer.getLength());
        this.teamColor = DriverStation.getAlliance().equals(Alliance.Red) ? new Color(255, 0, 0) : new Color(0, 0, 255);
        this.setSolidRGB(0, 255, 0);

        // Set the data
        this.led.setData(ledBuffer);

        this.led.start();
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
        for (var i = 0; i < this.ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values
            this.ledBuffer.setRGB(i, R, G, B);
        }
    }

    private boolean flashing = false;

    /**
     * Flashes the LEDs between on and off. Should be called periodically. Speed is
     * meant to be controlled by the scheduler
     * 
     * @param R Red value (0-255)
     * @param G Green value (0-255)
     * @param B Blue value (0-255)
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
     * 
     * @param speed_multiplier The speed multiplier of the rainbow. <strong>This is
     *                         how you speed up the light effect.</strong>
     */
    public void rainbow(int speed_multiplier) {
        final var length = ledBuffer.getLength();
        for (var i = 0; i < length; i++) {
            final var hue = (this.rainbowHueValue + (i * 180 / length)) % 180;
            this.ledBuffer.setHSV(i, hue, 255, 255);
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
     * @param speed_multiplier The speed multiplier of the cylon. <em>This is
     *                         how you speed up the light effect.</em>
     */
    public void cylon(int hue, int saturation, int speed_multiplier) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            int value = 255 - (Math.abs(this.cylon_center - i) * 50);
            if (value < 0) {
                value = 0;
            }
            this.ledBuffer.setHSV(i, hue, saturation, value);
        }
        this.cylon_center += (cylon_velocity * speed_multiplier);
        if (this.cylon_center >= this.ledBuffer.getLength() || this.cylon_center < 0) {
            this.cylon_velocity *= -1;
        } // makes the cylon go back and forth
    }

    /**
     * Creates a cylon effect but with two opposite things...
     * 
     * @param hue              The hue of the cylon from [0-180]
     * @param saturation       The saturation of the cylon from [0-255]
     * @param speed_multiplier The speed multiplier of the cylon. <em>This is how
     *                         you speed up the light effect.</em>
     * @param hue2             The hue of the second cylon from [0-180]
     * @param sat2             The saturation of the second cylon from [0-255]
     */
    public void cylon_but_two(int hue, int saturation, int speed_multiplier, int hue2, int sat2) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            int value = 255 - (Math.abs(this.cylon_center - i) * 40);
            this.ledBuffer.setHSV(i, hue, saturation, value);
            if (value < 0) {
                value = 0;
                int value2 = 255
                        - (Math.abs((ledBuffer.getLength() - this.cylon_center) % ledBuffer.getLength() - i) * 40);
                if (value2 < 0) {
                    value2 = 0;
                }
                this.ledBuffer.setHSV(i, hue2, sat2, value2);
            } // this is a bit wierd but it should be faster to compute than before (at least
              // double speed)
        }
        this.cylon_center += (cylon_velocity * speed_multiplier);
        if (this.cylon_center >= this.ledBuffer.getLength() || this.cylon_center < 0) {
            this.cylon_velocity *= -1;
        } // makes the cylon go back and forth
    }

    private int carnival_index = 0;

    /**
     * Creates a carnival effect using the LEDs (must be called periodically).
     * Looks best if the number of colors times the segment length is divisible by
     * the number of LEDs (150).
     * 
     * @param colors         An array of {@link edu.wpi.first.wpilibj.util.Color}s
     *                       to cycle through
     * @param segment_length The length of each segment of the color
     */
    public void carnival(Color[] colors, int segment_length) {
        int current_color_index = 0;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED((i + carnival_index) % ledBuffer.getLength(), colors[current_color_index]);
            if ((i + 1) % segment_length == 0) {
                current_color_index++;
                current_color_index %= colors.length;
            }
        }
        carnival_index++;
    }

    public enum period {
        AUTO, TELEOP, DISABLED, TEST, ENDGAME
    }

    private period currentPeriod = Lights.period.DISABLED;

    public Lights.period getPeriod() {
        return currentPeriod;
    }

    long teleop_start_time = Long.MAX_VALUE;
    long auto_start_time = Long.MAX_VALUE;

    /**
     * Sets the current period of the game. This is used to set the default light
     * effect. Should generally be called from {@link frc.robot.Robot}
     * 
     * @param p The period of the game
     */
    public void setPeriod(Lights.period p) {
        currentPeriod = p;
        scheduler.setDefaultLightEffect();
        if (p == period.TELEOP) {
            teleop_start_time = System.currentTimeMillis();
        }
        if (p == period.AUTO) {
            auto_start_time = System.currentTimeMillis();
        }
        this.teamColor = DriverStation.getAlliance().equals(Alliance.Red) ? new Color(255, 0, 0) : new Color(0, 0, 255);
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
        private LightEffect current_effect;
        /* metadata that is useful for certain events (see the default effects) */
        private String current_effect_name = "None";
        private long tick_counter = 0;
        private int ticks_per_call = 1, test_index = 0;
        private double time_left = 0, fade_time_left = 0; // in seconds
        private Color[] fade_from, fade_to;
        LightEffect default_disabled, defualt_teleop, defualt_auto, default_endgame;

        LightEffect[] tests = new LightEffect[] {
                () -> {
                    Lights.this.flashingRGB(255, 0, 0);
                },
                () -> {
                    Lights.this.carnival(new Color[] { teamColor, Color.kBlack, Color.kWhite, Color.kBlack }, 3);
                },
                () -> {
                    Lights.this.rainbow(1);
                },
                () -> {
                    Lights.this.cylon(60, 255, 1);
                }
        };

        public Light_Scheduler() {
            this.default_disabled = () -> {
                Lights.this.cylon_but_two(2, 255, 1, 24, 255);
            };
            this.defualt_teleop = () -> {
                Lights.this.rainbow(2);
            };
            this.defualt_auto = () -> {
                Lights.this.carnival(new Color[] { teamColor, Color.kBlack, Color.kWhite, Color.kBlack }, 3);
            };
            this.default_endgame = () -> {
                Lights.this.flashingRGB(255, 0, 0);
            };

            this.setDefaultLightEffect();
        }

        /**
         * Sets the light effect to the specified effect for the specified duration.
         * 
         * @param effect         The light effect to be applied. see the default effects
         *                       in {@link #Light_Scheduler} for an example of how to do
         *                       this.
         * @param duration       The duration of the light effect in seconds.
         * @param ticks_per_call How ofted the light effect should be applied. A value
         *                       of 1 is every .02 seconds. <em>This is how you slow
         *                       down the light effect.</em>
         * @param fade_duration  The duration of the fade to this effect in seconds.
         */
        public void setLightEffect(LightEffect effect, double duration, int ticks_per_call, double fade_duration) {
            this.current_effect = effect;
            this.time_left = duration;
            this.ticks_per_call = ticks_per_call;
            this.fade_time_left = fade_duration;

            if (fade_duration > 0) {
                this.fadeLightEffectInit();
            }
            this.current_effect_name = "Undefined";
        }

        /**
         * Sets the light effect to the specified effect for the specified duration.
         * 
         * @param effect         The light effect to be applied. see the default effects
         *                       in {@link #Light_Scheduler} for an example of how to do
         *                       this.
         * @param duration       The duration of the light effect in seconds.
         * @param ticks_per_call How ofted the light effect should be applied. A value
         *                       of 1 is every .02 seconds. <em>This is how you slow
         *                       down the light effect.</em>
         * @param fade_duration  The duration of the fade to this effect in seconds.
         * @param name           The name of the effect. This is used for conditional
         *                       things
         */
        public void setLightEffect(LightEffect effect, double duration, int ticks_per_call, double fade_duration,
                String name) {
            this.setLightEffect(effect, duration, ticks_per_call, fade_duration);
            this.current_effect_name = name;
        }

        private void fadeLightEffectInit() {
            this.fade_from = new Color[ledBuffer.getLength()];
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                this.fade_from[i] = ledBuffer.getLED(i);
            }
            this.current_effect.apply();
            this.fade_to = new Color[ledBuffer.getLength()];
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                this.fade_to[i] = ledBuffer.getLED(i);
            }
            fade_time_elapsed = 0;
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, (int) fade_from[i].red, (int) fade_from[i].green, (int) fade_from[i].blue);
            }
        }

        private double fade_time_elapsed = 0;

        private void fadeLightEffect() {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int red = (int) (255 * (fade_from[i].red + (fade_time_elapsed
                        * ((fade_to[i].red - fade_from[i].red) / (fade_time_left + fade_time_elapsed)))));
                int green = (int) (255 * (fade_from[i].green + (fade_time_elapsed
                        * ((fade_to[i].green - fade_from[i].green) / (fade_time_left + fade_time_elapsed)))));
                int blue = (int) (255 * (fade_from[i].blue + (fade_time_elapsed
                        * ((fade_to[i].blue - fade_from[i].blue) / (fade_time_left + fade_time_elapsed)))));
                ledBuffer.setRGB(i, red, green, blue);
            }
            fade_time_left -= .02;
            fade_time_elapsed += .02;
        }

        /** Sets the default light effect for the current period */
        public void setDefaultLightEffect() {
            period period = Lights.this.getPeriod();
            switch (period) {
                case DISABLED:
                    if (!this.current_effect_name.equals("Default Disabled")) {
                        this.setLightEffect(default_disabled, 0, 1, .25, "Default Disabled");
                    }
                    break;
                case AUTO:
                    if (!this.current_effect_name.equals("Default Auto")) {
                        this.setLightEffect(defualt_auto, 0, 1, .25, "Default Auto");
                    }
                    break;
                case TELEOP:
                    if (!this.current_effect_name.equals("Default Teleop")) {
                        this.setLightEffect(defualt_teleop, 0, 1, .25, "Default Teleop");
                    }
                    break;
                case TEST:
                    if (!this.current_effect_name.equals("Default Test")) {
                        this.setLightEffect(tests[test_index], 7, 1, 1, "Default Test");
                    }
                    test_index++;
                    test_index %= tests.length;
                    break;
                case ENDGAME:
                    if (!this.current_effect_name.equals("Default Endgame")) {
                        this.setLightEffect(default_endgame, 0, 25, .25, "Default Endgame");
                    }
                    break;
            }
        }

        /** Method to be executed from the parent classes periodic function */
        public void periodic() {
            if (this.tick_counter++ % this.ticks_per_call == 0 && this.fade_time_left <= 0) {
                this.current_effect.apply();
            }
            if (this.fade_time_left > 0) {
                this.fadeLightEffect();
            } else if (this.time_left <= 0) {
                // if no time is left for whatever effect, set the default light effect for the
                // current period.
                this.setDefaultLightEffect();
                // this can be optimized by only calling it once after the time runs out but for
                // readability it works.
            } else {
                this.time_left -= .02; // 0.02 seconds is usually the period of the periodic function
            }

            if (this.tick_counter == Long.MAX_VALUE) { // reset the tick counter to prevent overflow
                this.tick_counter = 0;
            }

        }

        /**
         * @return The time left in the current effect in seconds.
         */
        public double getTimer() {
            return this.time_left;
        }

        /**
         * Sets the time left in an effect to a specific value. This is useful when you
         * want the effect to stop after a certain event
         * 
         * @param time The time left in the effect in seconds.
         */
        public void setTimer(double time) {
            this.time_left = time;
        }

        /**
         * @return The name of the current effect.
         */
        public String getCurrentEffectName() {
            return this.current_effect_name;
        }
    }

    @Override
    public void periodic() {
        this.scheduler.periodic();
        this.led.setData(ledBuffer);

        // After 1:45 (105000ms) of teleop, set the period to endgame
        if (this.currentPeriod == period.TELEOP && System.currentTimeMillis() - teleop_start_time >= 105000) {
            setPeriod(period.ENDGAME);
            // After 15 seconds of auto, set the period to endgame (the transition for 3
            // seconds)
        } else if (this.currentPeriod == period.AUTO && System.currentTimeMillis() - auto_start_time >= 15000) {
            setPeriod(period.ENDGAME);
        }
    }
}
