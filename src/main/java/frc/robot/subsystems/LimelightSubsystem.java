package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Lime Light Subsystem Singleton
 */
public class LimelightSubsystem extends SubsystemBase {
    private static LimelightSubsystem instance;
    private NetworkTable table; // Network table to access Lime Light Values

    // Frequently used entries to store
    private NetworkTableEntry tx, ty, ta, tid, botpose;
    private double[] camerapose;

    public enum LightMode {
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        private final int ledMode;

        private LightMode(int ledMode) {
            this.ledMode = ledMode;
        }

        /**
         * Purpose - Get the current LED Mode
         * 
         * @return the LED mode as int
         */
        public int getLedMode() {
            return ledMode;
        }

        public enum StreamMode {
            // Side by side, Secondary camera is placed in lower right, Primary camera is
            // placed in lower right
            STANDARD(0), MAIN(1), SECONDARY(2);

            private final int mode;

            private StreamMode(int mode) {
                this.mode = mode;
            }

            /**
             * Get the Stream mode
             * 
             * @return The Stream Mode as an int
             */
            public int getMode() {
                return mode;
            }
        }

        public enum CamMode {
            // Camera feed uses vision processing, camera feed increases exposure and
            // disables vision processing
            VISION(0), DRIVER(1);

            private int mode;

            private CamMode(int mode) {
                this.mode = mode;
            }

            /**
             * Get the Camera mode
             * 
             * @return The Cam Mode as an int
             */
            public int getMode() {
                return mode;
            }
        }

        public enum SnapMode {
            // Enables snapshots, Disables snapshots
            DISABLED(0), ENABLED(1);

            private int mode;

            private SnapMode(int mode) {
                this.mode = mode;
            }

            /**
             * Get the Camera mode
             * 
             * @return The Cam Mode as an int
             */
            public int getMode() {
                return mode;
            }
        }
    }

    // Light mode tracking values
    private LightMode currentState = LightMode.OFF;
    private final LightMode LIGHT_ON = LightMode.ON,
            LIGHT_OFF = LightMode.OFF;

    public SendableChooser<Boolean> snapshotChooser = new SendableChooser<>(),
            lightChooser = new SendableChooser<>();

    /**
     * Lime Light Driver Singleton
     */
    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        camerapose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid");
        botpose = table.getEntry("botpose");
        setLightState(LIGHT_OFF);

        snapshotChooser.setDefaultOption("Disabled", false);
        snapshotChooser.addOption("Enabled", true);

        lightChooser.setDefaultOption("On", true);
        lightChooser.addOption("Off", false);
    }

    /**
     * Are we currently tracking any potential targets
     * 
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public boolean hasValidTarget() {
        return (table.getEntry("tv").getDouble(0) == 0) ? false : true;
    }

    /**
     * Horizontal offset from crosshair to target
     * 
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
     *         degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getXAngle() {
        return tx.getDouble(0);
    }

    public double getID() {
        return tid.getDouble(0);
    }

    public Pose2d getVisionPose2d() {
        double[] botposeAHHHH = botpose.getDoubleArray(new double[8]);
        return new Pose2d(new Translation2d(botposeAHHHH[0], botposeAHHHH[1]), new Rotation2d(botposeAHHHH[5]));
    }

    /**
     * Vertical offset from crosshair to target
     * 
     * @return Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5
     *         degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getYAngle() {
        return ty.getDouble(0);
    }

    /**
     * Get the area of the vision tracking box
     * 
     * @return Target Area (0% of image to 100% of image)
     */
    public double getArea() {
        return ta.getDouble(0);
    }

    public Transform3d getCameraPoseTargetSpace() {

        var pose2dCam = new Pose2d(new Translation2d(camerapose[0], camerapose[1]), new Rotation2d(camerapose[2]));
        var pose2dTarg = new Pose2d(new Translation2d(camerapose[3], camerapose[4]), new Rotation2d(camerapose[5]));

        Transform3d cameraPose = new Transform3d(new Pose3d(pose2dCam), new Pose3d(pose2dTarg));
        return cameraPose;
    }

    /**
     * Rotation of the object
     * 
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkew() {
        return table.getEntry("ts").getDouble(0);
    }

    /**
     * Latency in ms of the pipeline
     * 
     * @return The pipeline’s latency contribution (ms) Add at least 11ms for image
     *         capture latency.
     */
    public double getDeltaTime() {
        return table.getEntry("tl").getDouble(0);
    }

    /**
     * The length of the shortest side of the bounding box in pixels
     * 
     * @return Side length of shortest side of the fitted bounding box (pixels)
     */
    public double getShortLength() {
        return table.getEntry("tshort").getDouble(0);
    }

    /**
     * The length of the longest side of the bounding box in pixels
     * 
     * @return Side length of longest side of the fitted bounding box (pixels)
     */
    public double getLongLength() {
        return table.getEntry("tlong").getDouble(0);
    }

    /**
     * The length of the horizontal side of the box (0-320 pixels)
     * 
     * @return Horizontal side length of the rough bounding box (0 - 320 pixels)
     */
    public double getHorizontalLength() {
        return table.getEntry("thor").getDouble(0);
    }

    /**
     * The length of the vertical side of the box (0-320 pixels)
     * 
     * @return Vertical side length of the rough bounding box (0 - 320 pixels)
     */
    public double getVerticalLength() {
        return table.getEntry("tvert").getDouble(0);
    }

    /**
     * Returns the index of the current vision pipeline (0... 9)
     * 
     * @return True active pipeline index of the camera (0 .. 9)
     */
    public int getPipeIndex() {
        return (int) table.getEntry("getpipe").getDouble(0);
    }

    /**
     * The X-Coordinates of the tracked box
     * 
     * @return Number array of corner x-coordinates
     */
    public double[] getXCorners() {
        return table.getEntry("tcornx").getDoubleArray(new double[] { 0, 0, 0, 0 });
    }

    /**
     * The Y-Coordinates of the tracked box
     * 
     * @return Number array of corner y-coordinates
     */
    public double[] getYCorners() {
        return table.getEntry("tcorny").getDoubleArray(new double[] { 0, 0, 0, 0 });
    }

    /**
     * Sets the Lime Light LED's
     * 
     * @param mode - LightMode (On, Off, Blinking, or determined by the pipeline)
     */
    public void setLightState(LightMode mode) {
        currentState = mode;
        table.getEntry("ledMode").setNumber(currentState.getLedMode());
    }

    /**
     * Set the Lime Light Camera Mode
     * 
     * @param mode - VISION enables vision processing and decreases exposure, DRIVER
     *             disables vision processing and increases exposure
     */
    public void setCamMode(LightMode.CamMode mode) {
        table.getEntry("camMode").setNumber(mode.getMode());

    }

    /**
     * Sets the limelights current pipeline
     * 
     * @param pipeline The pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Sets the layout of the cameras viewed at 10.56.67.11:5800
     * 
     * @param streamMode - STANDARD is side by side, MAIN is Limelight big with
     *                   secondary camera in bottom right, SECONDARY is vice versa
     */
    public void setStreamMode(LightMode.StreamMode streamMode) {
        table.getEntry("stream").setNumber(streamMode.getMode());
    }

    /**
     * Allow the Lime Light to take snapshots so that it can be tuned off the field
     * 
     * @param mode DISABLED turns Snap Mode off, Enabled turns Snap Mode on
     */
    public void takeSnapshots(LightMode.SnapMode mode) {
        table.getEntry("snapshot").setNumber(mode.getMode());
    }

    /**
     * Toggle the current Lime Light LED's between on and off states
     */
    public void toggleLight() {
        currentState = (currentState == LIGHT_ON) ? LIGHT_OFF : LIGHT_ON;
        setLightState(currentState);
    }

    /**
     * Turn the Lime Light LED's off
     */
    public void turnLightOff() {
        setLightState(LIGHT_OFF);
    }

    /**
     * Turn the Lime Light LED's on
     */
    public void turnLightOn() {
        setLightState(LIGHT_ON);
    }

    /**
     * Get the Lime Light Subsystem instance
     * 
     * @return The Lime Light instance
     */
    public static LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    /**
     * Purpose: Get the distance of the limelight target
     * 
     * @param h1 - The height of the limelight with respect to the floor
     * @param h2 - The height of the target
     * @param a1 - The mounting angle for the limelight
     * @param a2 - The angle between the limelight angle and the target
     * 
     * @return The distance between the robot and the limelight
     */
    public double getDistance(double h1, double h2, double a1, double a2) {
        return (h2 - h1) / Math.abs(Math.tan(Math.toRadians(a1) + Math.toRadians(a2)));
    }

    /**
     * Output diagnostics
     */
    public void outputTelemetry() {
        SmartDashboard.putBoolean("LimelightHasTarget", hasValidTarget());
        SmartDashboard.putNumber("LimelightX", getXAngle());
        SmartDashboard.putNumber("LimelightY", getYAngle());
        SmartDashboard.putNumber("LimelightArea", getArea());
        SmartDashboard.putNumber("LimelightSkew", getSkew());
        SmartDashboard.putString("LimelightXCorners", Arrays.toString(getXCorners()));
        SmartDashboard.putString("LimelightYCorners", Arrays.toString(getYCorners()));
    }

    /**
     * Send the custom choosers to Shuffleboard
     */
    public void outputChoosers() {
        Shuffleboard.getTab("Teleop").add("Snapshot", snapshotChooser);
        Shuffleboard.getTab("Teleop").add("Light Mode", lightChooser);
    }

    public void updateChoosers() {
        if (lightChooser.getSelected()) {
            turnLightOn();
        } else {
            turnLightOff();
        }
    }

    @Override
    public void periodic() {
        outputTelemetry(); // updates values for x angle, y angle, skew, area, x corners, y corners
    }
}