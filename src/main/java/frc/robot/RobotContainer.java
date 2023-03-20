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
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ClawConeIntake;
import frc.robot.commands.ClawConeOuttake;
import frc.robot.commands.ClawCubeIntake;
import frc.robot.commands.ClawCubeOuttake;
import frc.robot.commands.DrivetrainCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.WristCubeIntake;
import frc.robot.commands.WristCubeOuttake;
import frc.robot.commands.autoGoBrrrrr;
import frc.robot.commands.SlideDefaultCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ClawSubsystem;
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
    // public static BooleanSupplier BButtonPressedFirst = new BooleanSupplier() {
    // public boolean getAsBoolean() {
    // return m_firstController.getBButtonPressed();
    // }
    // }, XButtonPressedFirst = new BooleanSupplier() {
    // public boolean getAsBoolean() {
    // return m_firstController.getXButtonPressed();
    // }
    // }, AButtonPressedFirst = new BooleanSupplier() {
    // public boolean getAsBoolean() {
    // return m_firstController.getAButtonPressed();
    // }
    // };

    // The robot's subsystems and commands are defined here...
    public static final XboxController m_firstController = new XboxController(0); // creates xboxController object
    public static final CommandXboxController firstCommandController = new CommandXboxController(0);

    public static final XboxController m_secondController = new XboxController(1); // creates xboxController object
    public static final CommandXboxController secondCommandController = new CommandXboxController(1);

    @SuppressWarnings("unused")
    public SendableChooser<String> autoChooser = new SendableChooser<String>();
    private SlideSubsystem m_slide;
    private DrivetrainSubsystem m_drive; // declares dt subsystem
    public WristSubsystem m_wrist;
    public ClawSubsystem m_claw; // declares claw subsystem

    // deal with it liam

    public boolean done;
    public boolean outtakeFinish = false;
    public double inOrOut = 0;
    public String coneOrCube;
    public Lights lightstrip;
    public LimelightSubsystem Limelight;
    public static PoseEstimator poseEstimate;
    public static double pEditor = 0, dEditor = 0;
    public double intakeToggle = 0;
    public SwerveAutoBuilder autoBuilder;
    public Robot robot; // uh i dont think we need this -benjamin
    private Command fullAuto;
    public boolean intakeFinish = false;
    public double speedMultiplier = .9;

    private GamePiece currentElement, targetElement;
    PathPlannerTrajectory HSC;
    PathPlannerTrajectory CSC;
    PathPlannerTrajectory BSC;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Robot robot) {
        this.robot = robot;
        this.coneOrCube = "cube";
        m_drive = new DrivetrainSubsystem();
        m_drive.setDefaultCommand(new DrivetrainCommand(m_drive, this));
        // poseEstimate = new PoseEstimator(null, m_drive);
        Limelight = new LimelightSubsystem();
        m_wrist = new WristSubsystem(this);
        m_claw = new ClawSubsystem();
        m_slide = new SlideSubsystem();

        m_claw.setDefaultCommand(new ClawCommand(m_claw, this));
        m_wrist.setDefaultCommand(new WristCommand(m_wrist, this));
        // m_slide.setDefaultCommand(new SlideDefaultCommand(m_slide, m_wrist, this));

        currentElement = GamePiece.NONE;
        targetElement = GamePiece.NONE;

        lightstrip = new Lights(Constants.LightConstants.lightstrip1Port, Constants.LightConstants.lightstrip1Length);

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
                return m_drive.getPositionPose2d();
            }
        };

        Consumer<Pose2d> resetPoseConsumer = new Consumer<Pose2d>() {
            @Override
            public void accept(Pose2d pose) {
                m_drive.resetPose(DrivetrainSubsystem.positions, pose);

            }
        };

        Consumer<SwerveModuleState[]> outputModuleConsumer = new Consumer<SwerveModuleState[]>() {
            @Override
            public void accept(SwerveModuleState[] t) {
                m_drive.pleaseGodLetThisWork(t[0], t[1], t[2], t[3]);
            }
        };

        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.
        autoBuilder = new SwerveAutoBuilder(
                poseSupplier, // Pose2d supplier
                resetPoseConsumer, // Pose2d consumer, used to reset odometry at the beginning of auto
                m_drive.m_kinematics, // SwerveDriveKinematics
                new PIDConstants(2.7109, 0, 0), // PID constants to correct for translation error (used to create the X
                                                // and Y PID controllers)
                new PIDConstants(7, 12, 0.1), // PID constants to correct for rotation error (used to create the
                                              // rotation controller)
                outputModuleConsumer, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color.
                      // Optional, defaults to true
                m_drive // The drive subsystem. Used to properly set the requirements of path following
                        // commands
        );
        configureButtonBindings();
        // autoChooser.addOption("CSC", new ClawCubeOuttake(m_claw, m_wrist,
        // this).andThen(autoBuilder.fullAuto(CSC)));
        // autoChooser.addOption("HSC", new ClawCubeOuttake(m_claw, m_wrist,
        // this).andThen(autoBuilder.fullAuto(HSC)));
        // autoChooser.addOption("BSC", new ClawCubeOuttake(m_claw, m_wrist,
        // this).andThen(autoBuilder.fullAuto(BSC)));
        // autoChooser.setDefaultOption("default", new ClawCubeOuttake(m_claw, m_wrist,
        // this));
        autoChooser.addOption("CSC", "CSC");
        autoChooser.addOption("BSC", "BSC");
        autoChooser.addOption("HSC", "HSC");
        autoChooser.addOption("default", "default");

        SmartDashboard.putData(autoChooser);

        // this.fullAuto = autoBuilder.fullAuto(pathGroup);
    }

    public void setCurrentElement(GamePiece element) {
        currentElement = element;
    }

    public int getBumperPos() {
        return m_wrist.getBumperPos();
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

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public void setConeOrCube(String coneOrCube) {
        this.coneOrCube = coneOrCube;
    }

    public String getConeOrCube() {
        return this.coneOrCube;
    }

    /** @return true if cube, false if cone */
    public boolean coneOrCubeBoolean() {
        return coneOrCube.equals("cube");
    }

    public SwerveAutoBuilder getBuild() {
        return autoBuilder;
    }

    public Command cubeChooser() {
        if (inOrOut % 2 == 0) {
            return new ClawCubeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0)
                    .until(getSticks(getSticksMode.NONE)).andThen(new WristCubeIntake(m_wrist, this))
                    .until(getSticks(getSticksMode.NONE));
        } else {
            return new ClawCubeOuttake(m_claw, m_wrist, this).until(getSticks((getSticksMode.NONE)))
                    .andThen(new WristCubeOuttake(m_wrist, this)).until(getSticks(getSticksMode.NONE));
        }
    }

    public Command coneChooser() {
        if (inOrOut % 2 == 0) {
            return new ClawConeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0)
                    .until(doneIntakeOuttake(intakeOrOuttake.INTAKE));
            // .andThen(new WristConeIntake(m_wrist, intakeFinish, m_claw,
            // this)).until(doneIntakeOuttake(intakeOrOuttake.INTAKE));
            // return new ClawConeOuttake(m_claw,
            // this).until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE));

        } else {
            return new ClawConeOuttake(m_claw, this).until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE));
            // .andThen(new
            // WristConeOuttake(m_wrist,
            // this)).until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))
            // return new ClawConeIntake(m_claw, m_wrist, true, this, inOrOut % 2 ==
            // 0).until(doneIntakeOuttake(intakeOrOuttake.INTAKE));/* .andThen(new
            // WristConeIntake(m_wrist, intakeFinish, m_claw,
            // this)).until(doneIntakeOuttake(intakeOrOuttake.INTAKE));
        }
    }

    public void configureButtonBindings() {
        @SuppressWarnings("unused")
        final Trigger LeftTrigger = new JoystickButton(m_firstController, XboxController.Button.kLeftBumper.value),
                RightTrigger = new JoystickButton(m_firstController, XboxController.Button.kRightBumper.value),
                yButton = secondCommandController.y(),
                bButton = new JoystickButton(m_firstController, XboxController.Button.kB.value),
                aButton = new JoystickButton(m_firstController, XboxController.Button.kA.value),
                xButton = new JoystickButton(m_firstController, XboxController.Button.kX.value);
        // bButton.onTrue(new ClawCommand(m_claw, this));
        // bButton.onTrue(new AlignFlatSurface(this));

        // xButton.onT`q1rue(autoBuilder.followPath(PathPlanner.generatePath(
        // new PathConstraints(5, 5),
        // new PathPoint(new Translation2d(
        // poseEstimate.getCurrentPose().getX(),
        // poseEstimate.getCurrentPose().getY()),
        // poseEstimate.getCurrentPose().getRotation()),
        // new PathPoint(new Translation2d(
        // PoleFinder.getNearestPole().getX(),
        // PoleFinder.getNearestPole().getY()),
        // PoleFinder.getNearestPole().getRotation())))
        // .until(getSticks(getSticksMode.POLE)));
        // // pole align
        // yButton.onTrue(autoBuilder.followPath(PathPlanner.generatePath(new
        // PathConstraints(5, 5),
        // new PathPoint(
        // new Translation2d(poseEstimate.getCurrentPose().getX(),
        // poseEstimate.getCurrentPose().getY()),
        // poseEstimate.getCurrentPose().getRotation()),
        // new PathPoint(
        // new Translation2d(FlatSurfaceFinder.getNearestPole().getX(),
        // FlatSurfaceFinder.getNearestPole().getY()),
        // FlatSurfaceFinder.getNearestPole().getRotation())))
        // .until(getSticks(getSticksMode.SURFACE)));
        // surface align

        aButton.onTrue(
                new ClawCubeOuttake(m_claw, m_wrist, this)
                        .until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))
        /*
         * .andThen(new WristCubeOuttake(m_wrist, this))
         * .until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))
         */);
        yButton.onTrue(
                new ClawConeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0)
                        .until(doneIntakeOuttake(intakeOrOuttake.INTAKE))/*
                                                                          * .andThen(new WristConeIntake(m_wrist,
                                                                          * intakeFinish, m_claw,
                                                                          * this)).until(doneIntakeOuttake(
                                                                          * intakeOrOuttake.INTAKE))
                                                                          */);
        xButton.onTrue(
                new ClawCubeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0)
                        .until(doneIntakeOuttake(intakeOrOuttake.INTAKE))
        /*
         * .andThen(new WristCubeIntake(m_wrist, this))
         * .until(doneIntakeOuttake(intakeOrOuttake.INTAKE))
         */);
        bButton.onTrue(
                new ClawConeOuttake(m_claw, this)
                        .until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))/*
                                                                           * .andThen(new WristConeOuttake(m_wrist,
                                                                           * this)).until(doneIntakeOuttake(
                                                                           * intakeOrOuttake.OUTTAKE))
                                                                           */);

        // aButton.onTrue(new ClawIntakeAndOuttakeCommand(m_claw, m_wrist, false, this,
        // inOrOut % 2 == 0).until(getSticks(getSticksMode.NONE))/* .andThen(new
        // IntakeOuttakeProcessWrist(m_wrist, false,m_claw, this)*/)/* )*/;
        // bButton.onTrue(new ClawCubeIntake(m_claw, m_wrist, true, this, inOrOut % 2 ==
        // 0).until(getSticks(getSticksMode.INTAKE))/*.andThen(new
        // IntakeOuttakeProcessWrist(m_wrist, true, m_claw, this))*/);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if (autoChooser.getSelected() == "default") {
            return new ClawCubeOuttake(m_claw, m_wrist, this).finallyDo((boolean interrupt) -> {
                ++intakeToggle;
            });
        } else if (autoChooser.getSelected() == "BSC") {
            m_drive.m_pose = new Pose2d(1.66, .43, m_drive.getInitGyro()); // BSC
            return new ClawCubeOuttake(m_claw, m_wrist,
                    this).withTimeout(2).andThen(autoBuilder.fullAuto(BSC))
                    .finallyDo((boolean interrupt) -> {
                        ++intakeToggle;
                    });

        }

        else if (autoChooser.getSelected() == "CSC") {
            // m_drive.m_pose = new Pose2d(1.66, 2.98, m_drive.getInitGyro()); // CSC
            // return new ClawCubeOuttake(m_claw, m_wrist,
            // this).withTimeout(2).andThen(autoBuilder.fullAuto(CSC))
            // .finallyDo((boolean interrupt) -> {
            // ++intakeToggle;
            // });
            return new autoGoBrrrrr(m_drive, m_claw);

        }

        else if (autoChooser.getSelected() == "HSC") {
            m_drive.m_pose = new Pose2d(1.73, 4.67, m_drive.getInitGyro());

            return new ClawCubeOuttake(m_claw, m_wrist,
                    this).withTimeout(2).andThen(autoBuilder.fullAuto(HSC))
                    .finallyDo((boolean interrupt) -> {
                        ++intakeToggle;

                    });
            // return autoBuilder.fullAuto(HSC);
        }
        // autoChooser.addOption("CSC", new ClawCubeOuttake(m_claw, m_wrist,
        // this).andThen(autoBuilder.fullAuto(CSC)));
        // autoChooser.addOption("HSC", new ClawCubeOuttake(m_claw, m_wrist,
        // this).andThen(autoBuilder.fullAuto(HSC)));
        // autoChooser.addOption("BSC", new ClawCubeOuttake(m_claw, m_wrist,
        // this).andThen(autoBuilder.fullAuto(BSC)));
        // autoChooser.setDefaultOption("default", new ClawCubeOuttake(m_claw,
        // m_wrist,
        // this));

        // An ExampleCommand will run in autonomous
        return new ClawCubeOuttake(m_claw, m_wrist, this).finallyDo((boolean interrupt) -> {
            ++intakeToggle;
        });
        // return null;

    }

    public static enum getSticksMode {
        POLE, SURFACE, NONE, INTAKE
    }

    public static enum intakeOrOuttake {
        INTAKE, OUTTAKE
    }

    public static enum GamePiece {
        NONE,
        CUBE,
        CONE
    }

    public BooleanSupplier inOrOutCheck(double inOrOut) {
        return new BooleanSupplier() {
            public boolean getAsBoolean() {
                return inOrOut % 2 == 0;
            }
        };
    }

    public BooleanSupplier doneIntakeOuttake(intakeOrOuttake mode) {
        return new BooleanSupplier() {

            public boolean getAsBoolean() {
                switch (mode) {
                    case INTAKE:
                        done = intakeFinish;
                        break;
                    case OUTTAKE:
                        done = outtakeFinish;
                        break;

                }
                return done;
            }

        };
    };

    public BooleanSupplier getSticks(getSticksMode mode) {
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
                    case INTAKE:
                        extra = intakeFinish;
                        break;
                    case NONE:
                        extra = false;
                        break;
                }
                return (Math.abs((MathUtil.applyDeadband(RobotContainer.m_firstController.getLeftX(), 0.1))) > 0) ||
                        (Math.abs((MathUtil.applyDeadband(RobotContainer.m_firstController.getLeftY(), 0.1))) > 0) ||
                        extra;

            }
        };
    }
}
