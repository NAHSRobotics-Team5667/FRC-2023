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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignFlatSurface;
import frc.robot.commands.AlignPoleAgain;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ClawConeIntake;
import frc.robot.commands.ClawConeOuttake;
import frc.robot.commands.ClawCubeIntake;
import frc.robot.commands.ClawCubeOuttake;
import frc.robot.commands.DrivetrainCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeAndOuttakeProcedure;
import frc.robot.commands.WristConeIntake;
import frc.robot.commands.WristConeOuttake;
import frc.robot.commands.WristCubeIntake;
import frc.robot.commands.WristCubeOuttake;
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
    public static BooleanSupplier BButtonPressed = new BooleanSupplier() {
        public boolean getAsBoolean(){
           return m_controller.getBButtonPressed();
        }
    };
    
    public static BooleanSupplier XButtonPressed = new BooleanSupplier() {
        public boolean getAsBoolean(){
           return m_controller.getXButtonPressed();
        }
    };
    public static BooleanSupplier AButtonPressed = new BooleanSupplier() {
        public boolean getAsBoolean(){
           return m_controller.getAButtonPressed();
        }
    };
    
    // The robot's subsystems and commands are defined here...
    public static final XboxController m_controller = new XboxController(0); // creates xboxController object
    public static final CommandXboxController commandController = new CommandXboxController(0);


    @SuppressWarnings("unused")
    
    private SlideSubsystem m_slide;
    private DrivetrainSubsystem m_drive; // declares dt subsystem
    public WristSubsystem m_wrist;
    public ClawSubsystem m_claw; // declares claw subsystem

     //deal with it liam
    public boolean done;
    public boolean outtakeFinish = false;
    public double inOrOut = 0;
    public String coneOrCube; 
    public Lights lightstrip;
    public static LimelightSubsystem Limelight;
    public static PoseEstimator poseEstimate;
    public static double pEditor = 0, dEditor = 0;
    public double intakeToggle = 0;
    public SwerveAutoBuilder autoBuilder;
    public Robot robot; //uh i dont think we need this -benjamin
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private Command fullAuto;

    public boolean intakeFinish = false;

    private GamePiece currentElement;

    private GamePiece targetElement;

    public RobotContainer(Robot robot) {
        this.coneOrCube = "cube";
        m_slide = new SlideSubsystem();
        this.robot = robot;
        m_wrist = new WristSubsystem(this);
        m_drive = new DrivetrainSubsystem();
        // m_drive.setDefaultCommand(new DrivetrainCommand(m_drive));
        poseEstimate = new PoseEstimator(null, m_drive);
        Limelight = new LimelightSubsystem();
        //removing everything that isnt the drive train for now to make troubleshooting easier
        //why are we insantiating the robot in robot container? doesnt the hierarchy go from robot to robot container?
        m_claw = new ClawSubsystem();

        m_claw.setDefaultCommand(new ClawCommand(m_claw, this));
        m_wrist.setDefaultCommand(new WristCommand(m_wrist, this));
        m_slide.setDefaultCommand(new SlideDefaultCommand(m_slide, m_wrist, this));
        
        currentElement = GamePiece.NONE;
        targetElement = GamePiece.NONE;

        lightstrip = new Lights(Constants.LightConstants.lightstrip1Port, Constants.LightConstants.lightstrip1Length);

        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        PathPlannerTrajectory pathGroup = PathPlanner.loadPath("New Path", new PathConstraints(5, 5));

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
    public void coneOrCubeChange(String coneOrCube){
        this.coneOrCube = coneOrCube;

    }

    public String getCubeOrCone(){
        return this.coneOrCube;
    }

    public boolean coneOrCubeBoolean(){
        if (coneOrCube == "cube") {
            return true;
            
        } else {
            return false;
        }
    }
    public SwerveAutoBuilder getBuild(){
        return autoBuilder;
    }

    public Command cubeChooser(){
        
        if (inOrOut % 2 == 0){
            return new ClawCubeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0).until(getSticks(getSticksMode.NONE)).andThen(new WristCubeIntake(m_wrist, this)).until(getSticks(getSticksMode.NONE));
            
        } else{
            return new ClawCubeOuttake(m_claw, this).until(getSticks((getSticksMode.NONE))).andThen(new WristCubeOuttake(m_wrist, this)).until(getSticks(getSticksMode.NONE));
        }
    }

    public Command coneChooser(){
        if (inOrOut % 2 == 0){
            return new ClawConeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0).until(doneIntakeOuttake(intakeOrOuttake.INTAKE))/* .andThen(new WristConeIntake(m_wrist, intakeFinish, m_claw, this)).until(doneIntakeOuttake(intakeOrOuttake.INTAKE))*/;
            //return new ClawConeOuttake(m_claw, this).until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE));
            
        } else{
           return new ClawConeOuttake(m_claw, this).until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))/* .andThen(new WristConeOuttake(m_wrist, this)).until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))*/;
            //return new ClawConeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0).until(doneIntakeOuttake(intakeOrOuttake.INTAKE));/* .andThen(new WristConeIntake(m_wrist, intakeFinish, m_claw, this)).until(doneIntakeOuttake(intakeOrOuttake.INTAKE))*/
        }
    }
   


    public void configureButtonBindings() {
        final Trigger 
        LeftTrigger = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value),
        RightTrigger = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value),
        yButton = commandController.y(),
        bButton = new JoystickButton(m_controller, XboxController.Button.kB.value),
        aButton = new JoystickButton(m_controller, XboxController.Button.kA.value),
        xButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
       // bButton.onTrue(new ClawCommand(m_claw, this));
       // bButton.onTrue(new AlignFlatSurface(this));
        xButton.onTrue(autoBuilder.followPath(PathPlanner.generatePath(
            new PathConstraints(5, 5),
            new PathPoint(new Translation2d(
                    RobotContainer.poseEstimate.getCurrentPose().getX(),
                    RobotContainer.poseEstimate.getCurrentPose().getY()),
                    RobotContainer.poseEstimate.getCurrentPose().getRotation()),
            new PathPoint(new Translation2d(
                    PoleFinder.getNearestPole().getX(),
                    PoleFinder.getNearestPole().getY()),
                    PoleFinder.getNearestPole().getRotation()))).until(getSticks(getSticksMode.POLE)));
                    //pole align
        yButton.onTrue(autoBuilder.followPath(PathPlanner.generatePath(new PathConstraints(5, 5),
        new PathPoint(
                new Translation2d(RobotContainer.poseEstimate.getCurrentPose().getX(),
                        RobotContainer.poseEstimate.getCurrentPose().getY()),
                RobotContainer.poseEstimate.getCurrentPose().getRotation()),
        new PathPoint(
                new Translation2d(FlatSurfaceFinder.getNearestPole().getX(),
                        FlatSurfaceFinder.getNearestPole().getY()),
                FlatSurfaceFinder.getNearestPole().getRotation()))).until(getSticks(getSticksMode.SURFACE)));
                //surface align


        aButton.onTrue( 
            new ClawCubeOuttake(m_claw, this)
            .until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))
            /*.andThen(new WristCubeOuttake(m_wrist, this))
            .until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))*/);
        bButton.onTrue(
            new ClawConeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0)
            .until(doneIntakeOuttake(intakeOrOuttake.INTAKE))/* .andThen(new WristConeIntake(m_wrist, intakeFinish, m_claw, this)).until(doneIntakeOuttake(intakeOrOuttake.INTAKE))*/);
        xButton.onTrue(
            new ClawCubeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0)
            .until(doneIntakeOuttake(intakeOrOuttake.INTAKE))
            /*.andThen(new WristCubeIntake(m_wrist, this))
            .until(doneIntakeOuttake(intakeOrOuttake.INTAKE))*/);
        yButton.onTrue(
            new ClawConeOuttake(m_claw, this)
            .until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))/* .andThen(new WristConeOuttake(m_wrist, this)).until(doneIntakeOuttake(intakeOrOuttake.OUTTAKE))*/);
     

    //    aButton.onTrue(new ClawIntakeAndOuttakeCommand(m_claw, m_wrist, false, this, inOrOut % 2 == 0).until(getSticks(getSticksMode.NONE))/* .andThen(new IntakeOuttakeProcessWrist(m_wrist, false,m_claw, this)*/)/* )*/;
     //  bButton.onTrue(new ClawCubeIntake(m_claw, m_wrist, true, this, inOrOut % 2 == 0).until(getSticks(getSticksMode.INTAKE))/*.andThen(new IntakeOuttakeProcessWrist(m_wrist, true, m_claw, this))*/);
        
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
    public static enum getSticksMode{
        POLE, SURFACE, NONE, INTAKE
    }

    public static enum intakeOrOuttake{
        INTAKE, OUTTAKE
    }

    public static enum GamePiece {
        NONE, 
        CUBE,
        CONE
    }

    public BooleanSupplier inOrOutCheck(double inOrOut){
        return new BooleanSupplier() {
            public boolean getAsBoolean(){
                return inOrOut % 2 == 0;
            }
        };
    }
    public BooleanSupplier doneIntakeOuttake(intakeOrOuttake mode){
    return new BooleanSupplier() {
        
    
     public boolean getAsBoolean(){
        switch (mode){
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
    public BooleanSupplier getSticks(getSticksMode mode){
    return new BooleanSupplier() {
            
            public boolean getAsBoolean() {
                boolean extra = false;
                switch (mode){
                    case POLE:
                        extra = Math.pow((Math.pow(PoleFinder.getNearestPole().getX(), 2) + Math.pow(PoleFinder.getNearestPole().getY(), 2)), .5) < .08;
                        break;
                    case SURFACE:
                        extra = Math.pow((Math.pow(FlatSurfaceFinder.getNearestPole().getX(), 2) + Math.pow(FlatSurfaceFinder.getNearestPole().getY(), 2)), .5) < .08;
                        break;
                    case NONE:
                        extra = false;
                        break;
                    case INTAKE:
                        extra = intakeFinish;
                        break;
                }
                return 
                    (Math.abs((MathUtil.applyDeadband(RobotContainer.m_controller.getLeftX(), 0.1)))> 0) || 
                    (Math.abs((MathUtil.applyDeadband(RobotContainer.m_controller.getLeftY(), 0.1)))> 0) || 
                    extra
                    ;
                    
            }
        };
}
}
