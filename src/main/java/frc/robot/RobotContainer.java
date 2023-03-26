// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ClawConeIntake;
import frc.robot.commands.ClawConeOuttake;
import frc.robot.commands.ClawCubeIntake;
import frc.robot.commands.ClawCubeOuttake;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.TestAuto;
import frc.robot.commands.SlideCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.FlatSurfaceFinder;
import frc.robot.util.PoleFinder;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static final XboxController slideController = new XboxController(0), // creates intake/outtake controller
            driveController = new XboxController(1); // creates drive controller

    public SendableChooser<String> autoChooser = new SendableChooser<String>(); // decides which auto we are using

    // declares all subsystems
    private SlideSubsystem slide;
    private DrivetrainSubsystem drive;
    public WristSubsystem wrist;
    public IntakeSubsystem intake;

    public boolean outtakeFinish = false;
    public double intakeToggle = 0;

    public Lights lightstrip;

    public LimelightSubsystem limelight;
    public static PoseEstimator poseEstimate;

    public SwerveAutoBuilder autoBuilder; // builds on-the-fly and autonomous paths
    public boolean intakeFinish = false;

    private double speedMultiplier; // determines how fast robot goes
    private double turnMultiplier;

    private GamePiece currentElement, targetElement; // keeps track of piece elements
    PathPlannerTrajectory HSC, CSC, BSC; // autonomous trajectories, needs inital pose set to path initial pose

    private int positionLevel;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drive = new DrivetrainSubsystem();
        drive.setDefaultCommand(new DrivetrainCommand(drive, this));

        speedMultiplier = 1;
        turnMultiplier = 0.7;

        limelight = new LimelightSubsystem(); // instantiate commands

        poseEstimate = new PoseEstimator(drive, limelight);

        wrist = new WristSubsystem(this);
        intake = new IntakeSubsystem();
        slide = new SlideSubsystem(this);
        lightstrip = new Lights(Constants.LightConstants.lightstrip1Port, Constants.LightConstants.lightstrip1Length);

        intake.setDefaultCommand(new IntakeCommand(intake)); // assign commands to subsystems
        wrist.setDefaultCommand(new WristCommand(wrist, this));
        slide.setDefaultCommand(new SlideCommand(slide, this));

        currentElement = GamePiece.NONE;
        targetElement = GamePiece.NONE;

        positionLevel = 0;

        // ===================================================================
        // AUTONOMOUS
        // ===================================================================

        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        CSC = PathPlanner.loadPath("CSC", new PathConstraints(5, 5));
        BSC = PathPlanner.loadPath("BSC", new PathConstraints(5, 5));
        HSC = PathPlanner.loadPath("HSC", new PathConstraints(5, 5));

        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        @SuppressWarnings({ "unchecked", "rawtypes" })
        HashMap<String, Command> eventMap = new HashMap();

        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        Supplier<Pose2d> poseSupplier = new Supplier<Pose2d>() {
            @Override
            public Pose2d get() {
                return drive.getPositionPose2d();
            }
        };

        Consumer<Pose2d> resetPoseConsumer = new Consumer<Pose2d>() {
            @Override
            public void accept(Pose2d pose) {
                drive.resetPose(DrivetrainSubsystem.positions, pose);
            }
        };

        Consumer<SwerveModuleState[]> outputModuleConsumer = new Consumer<SwerveModuleState[]>() {
            @Override
            public void accept(SwerveModuleState[] t) {
                drive.pleaseGodLetThisWork(t[0], t[1], t[2], t[3]);
            }
        };

        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.
        autoBuilder = new SwerveAutoBuilder(
                poseSupplier, // Pose2d supplier
                resetPoseConsumer, // Pose2d consumer, used to reset odometry at the beginning of auto
                drive.kinematics, // SwerveDriveKinematics
                new PIDConstants(2.7109, 0, 0), // PID constants to correct for translation error (used to create the X
                                                // and Y PID controllers)
                new PIDConstants(7, 12, 0.1), // PID constants to correct for rotation error (used to create the
                                              // rotation controller)
                outputModuleConsumer, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color.
                      // Optional, defaults to true
                drive // The drive subsystem. Used to properly set the requirements of path following
                      // commands
        );
        configureButtonBindings();
        autoChooser.addOption("CSC", "CSC");
        autoChooser.addOption("BSC", "BSC");
        autoChooser.addOption("HSC", "HSC");
        autoChooser.setDefaultOption("default", "default");

        SmartDashboard.putData(autoChooser);
        // ===================================================================
    }

    // ===================================================================
    // GAME PIECE HANDLING
    // ===================================================================

    public void setCurrentElement(GamePiece element) {
        currentElement = element;
    }

    public GamePiece getCurrentElement() {
        return currentElement;
    }

    public void setTargetElement(GamePiece element) {
        targetElement = element;
    }

    public GamePiece getTargetElement() {
        return targetElement;
    }

    // ===================================================================
    // POSITION LEVEL
    // ===================================================================

    public int getPositionLevel() {
        return positionLevel;
    }

    public void setPositionLevel(int positionLevel) {
        this.positionLevel = positionLevel;
    }

    public void updatePositionLevel(boolean isLeftBumperPressed, boolean isRightBumperPressed) {
        int maxPositionLevel = 0;
        // set max position level
        switch (getTargetElement()) {
            case CONE:
                maxPositionLevel = WristConstants.coneIntakeSetpoints.length;
                break;
            case CUBE:
                maxPositionLevel = WristConstants.cubeIntakeSetpoints.length;
                break;
            default:
                maxPositionLevel = (getCurrentElement().equals(GamePiece.NONE)) ? 0 : 3;
                break;
        }

        if (getPositionLevel() < maxPositionLevel && isRightBumperPressed) {
            positionLevel++;
        } else if (getPositionLevel() > 0 && isLeftBumperPressed) {
            positionLevel--;
        } else { // 0 <= position level <= maxPositionLevel and neither bumper is pressed
            // do nothing
            // what...
        }
    }

    // ===================================================================
    // SPEED MULTIPLIERS
    // ===================================================================

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public double getTurnMultiplier() {
        return turnMultiplier;
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public void setTurnMultiplier(double turnMultiplier) {
        this.turnMultiplier = turnMultiplier;
    }

    // ===================================================================

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public SwerveAutoBuilder getBuild() {
        return autoBuilder;
    }

    public void configureButtonBindings() {
        @SuppressWarnings("unused")
        final Trigger LeftBumper = new JoystickButton(slideController, XboxController.Button.kLeftBumper.value),
                RightBumper = new JoystickButton(slideController, XboxController.Button.kRightBumper.value),
                yButton = new JoystickButton(slideController, XboxController.Button.kY.value),
                bButton = new JoystickButton(slideController, XboxController.Button.kB.value),
                aButton = new JoystickButton(slideController, XboxController.Button.kA.value),
                xButton = new JoystickButton(slideController, XboxController.Button.kX.value),
                ySecondButton = new JoystickButton(driveController, XboxController.Button.kY.value),
                bSecondButton = new JoystickButton(driveController, XboxController.Button.kB.value),
                aSecondButton = new JoystickButton(driveController, XboxController.Button.kA.value),
                xSecondButton = new JoystickButton(driveController, XboxController.Button.kX.value);
        // makes all triggers

        aButton.and(xButton).whileTrue( // intake cone
                new ClawConeIntake(intake, wrist, true, this)
                        .until(checkIntakeFinish(IntakeOrOuttake.OUTTAKE)));
        aButton.and(bSecondButton).whileTrue( // outtake cone
                new ClawConeOuttake(intake, this)
                        .until(checkIntakeFinish(IntakeOrOuttake.OUTTAKE)));
        bButton.and(bSecondButton).whileTrue( // outtake cube
                new ClawCubeOuttake(intake, this)
                        .until(checkIntakeFinish(IntakeOrOuttake.INTAKE)));
        bButton.and(yButton).whileTrue( // intake cube
                new ClawCubeIntake(intake, wrist, true, this)
                        .until(checkIntakeFinish(IntakeOrOuttake.INTAKE)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        switch (autoChooser.getSelected()) {
            case "default":
                return new ClawCubeOuttake(intake, this).finallyDo((boolean interrupt) -> {
                    ++intakeToggle;
                });
            case "BSC":
                drive.pose = new Pose2d(1.66, .43, drive.getInitGyro()); // BSC
                return new ClawCubeOuttake(intake, this)
                        .withTimeout(2)
                        .andThen(autoBuilder.fullAuto(BSC))
                        .finallyDo((boolean interrupt) -> {
                            ++intakeToggle;
                        });
            case "CSC":
                // drive.m_pose = new Pose2d(1.66, 2.98, drive.getInitGyro()); // CSC
                // return new ClawCubeOuttake(claw, wrist,
                // this).withTimeout(2).andThen(autoBuilder.fullAuto(CSC))
                // .finallyDo((boolean interrupt) -> {
                // ++intakeToggle;
                // });
                return new TestAuto(drive, intake);
            case "HSC":
                drive.pose = new Pose2d(1.73, 4.67, drive.getInitGyro());
                return new ClawCubeOuttake(intake, this).withTimeout(2).andThen(autoBuilder.fullAuto(HSC))
                        .finallyDo((boolean interrupt) -> {
                            ++intakeToggle;
                        });
            // return autoBuilder.fullAuto(HSC);
        }

        return new ClawCubeOuttake(intake, this).finallyDo((boolean interrupt) -> {
            ++intakeToggle;
        });
    }

    public static enum GetSticksMode {
        POLE, SURFACE, NONE
    }

    public static enum IntakeOrOuttake {
        INTAKE, OUTTAKE
    }

    public static enum GamePiece {
        NONE, CUBE, CONE
    }

    public BooleanSupplier checkIntakeFinish(IntakeOrOuttake mode) {
        return new BooleanSupplier() {
            public boolean getAsBoolean() {
                switch (mode) {
                    case INTAKE:
                        return intakeFinish;
                    case OUTTAKE:
                        return outtakeFinish;
                    default:
                        return false; // catch all just in case
                }
            }
        };
    };

    public BooleanSupplier getSticks(GetSticksMode mode) {
        return new BooleanSupplier() {
            public boolean getAsBoolean() {
                boolean extra = false;
                switch (mode) {
                    case POLE:
                        extra = Math.pow((Math.pow(PoleFinder.getNearestPole().getX(), 2)
                                + Math.pow(PoleFinder.getNearestPole().getY(), 2)), .5) < .08;
                        break;
                    case SURFACE:
                        extra = Math.pow((Math.pow(FlatSurfaceFinder.getNearestPole().getX(), 2)
                                + Math.pow(FlatSurfaceFinder.getNearestPole().getY(), 2)), .5) < .08;
                        break;
                    case NONE:
                        extra = false;
                        break;
                }

                return (Math.abs((MathUtil.applyDeadband(RobotContainer.driveController.getLeftX(), 0.1))) > 0) ||
                        (Math.abs((MathUtil.applyDeadband(RobotContainer.driveController.getLeftY(), 0.1))) > 0) ||
                        extra; // extra returns true if distance to goal is small enough
                // returns true if sticks are moved as well
            }
        };
    }
}
