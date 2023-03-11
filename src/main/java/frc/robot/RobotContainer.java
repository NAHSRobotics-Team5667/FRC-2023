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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignFlatSurfaceAgain;
import frc.robot.commands.AlignPoleAgain;
import frc.robot.commands.ClawIntakeAndOuttakeCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.PoseEstimator;
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
   
    public static EventLoop BButtonCommand = new EventLoop();
    public static EventLoop XButtonCommand = new EventLoop();;
    public static EventLoop AButtonCommand = new EventLoop();;
    public static BooleanSupplier BButtonPressed = new BooleanSupplier() {
        public boolean getAsBoolean(){
           return m_controller.getBButtonPressed();
        }
    };
    
    public static Trigger BButton = new Trigger(BButtonCommand, BButtonPressed);
    public static BooleanSupplier XButtonPressed = new BooleanSupplier() {
        public boolean getAsBoolean(){
           return m_controller.getXButtonPressed();
        }
    };
    public static Trigger XButton = new Trigger(XButtonCommand, XButtonPressed);
    public static BooleanSupplier AButtonPressed = new BooleanSupplier() {
        public boolean getAsBoolean(){
           return m_controller.getAButtonPressed();
        }
    };
    public static Trigger AButton = new Trigger(AButtonCommand, AButtonPressed);
    
    // The robot's subsystems and commands are defined here...
    public static final XboxController m_controller = new XboxController(0); // creates xboxController object
    private DrivetrainSubsystem m_drive; // declares dt subsystem

    @SuppressWarnings("unused")
    private ClawSubsystem m_claw; // declares claw subsystem

     //deal with it liam
    public Lights lightstrip;
    public static LimelightSubsystem Limelight;
    public static PoseEstimator poseEstimate;
    public static double pEditor = 0;
    public static double dEditor = 0;
    public double intakeToggle = 0;
    public SwerveAutoBuilder autoBuilder;
    public Robot robot; //uh i dont think we need this -benjamin
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private Command fullAuto;

    public RobotContainer(Robot robot) {
        this.robot = robot;
        m_drive = new DrivetrainSubsystem();
        m_drive.setDefaultCommand(new DrivetrainCommand(m_drive));
        poseEstimate = new PoseEstimator(null, m_drive);
        Limelight = new LimelightSubsystem();
        //removing everything that isnt the drive train for now to make troubleshooting easier
        //why are we insantiating the robot in robot container? doesnt the hierarchy go from robot to robot container?
       // m_claw = new ClawSubsystem();
        //m_claw.setDefaultCommand(new ClawCommand(m_claw));
        
        
        


        lightstrip = new Lights();

        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        PathPlannerTrajectory pathGroup = PathPlanner.loadPath("New Path", new PathConstraints(5
, 5));

        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        @SuppressWarnings({"unchecked","rawtypes"})
        HashMap<String, Command> eventMap = new HashMap();

        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        Supplier<Pose2d> poseSupplier = new Supplier<Pose2d>() {
            @Override
            public Pose2d get() {
                return poseEstimate.getCurrentPose();
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
                new PIDConstants(2.7109,0 ,0), // PID constants to correct for translation error (used to create the X
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

        this.fullAuto = autoBuilder.fullAuto(pathGroup);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public SwerveAutoBuilder getBuild(){
        return autoBuilder;
    }
    private void configureButtonBindings() {
        BButton.onTrue(new AlignFlatSurfaceAgain(this));
        XButton.onTrue(new AlignPoleAgain(this));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return this.fullAuto;
        
    }
}
