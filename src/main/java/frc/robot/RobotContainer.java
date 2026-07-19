// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.handlers.DriveHandler;
import frc.robot.handlers.Superstructure;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;
import frc.robot.utils.simulation.crystalcaverns.CrystalCavernsArena;
import frc.robot.utils.simulation.crystalcaverns.MineSim;

public class RobotContainer {

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DriveConstants.TranslationDeadband).withRotationalDeadband(Constants.DriveConstants.RotationDeadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController opJoystick = new CommandXboxController(1);

    static {
    SimulatedArena.overrideInstance(new CrystalCavernsArena());
    }

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    //public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final IntakeSubsystem intake =  new IntakeSubsystem(drivetrain.getSimDrivetrain(), SimulatedArena.getInstance());
    public final ShooterSubsystem shooter = new ShooterSubsystem(drivetrain, intake); //only here because theres no shooterhandler
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem(elevator);
    public final Superstructure superstructure = Superstructure.getInstance(); 
 
    /* Path follower */
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {

        Superstructure.getInstance().initialize(drivetrain, joystick, elevator, arm);
        

        drivetrain.setParamsDT(joystick, MaxSpeed, MaxAngularRate, drive, drivetrain);
        

        configurePathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        SimulatedArena.getInstance().resetFieldForAuto();
        drivetrain.resetPose(new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(0)));
    }


    private void configurePathPlanner() {
        NamedCommands.registerCommand("ExampleCommand",
         new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
    }


     private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
       /* 
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward) DriveStraight = robot centric 
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        ); 
        */

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        // Reset the field-centric heading on right stick press.
        joystick.rightStick().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);
        
        //On true for pressing the Y button on joystick, it sets the superstructure state to idle
        joystick.y().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
        joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.SCOREHIGH)));


        joystick.leftTrigger().onTrue(new InstantCommand(() -> MineSim.drop(MineSim.getNearestStation(drivetrain.getSimPose()), false)));
        joystick.leftTrigger().onTrue(new InstantCommand(() -> intake.startIntake()));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> intake.stopIntake()));
        joystick.leftBumper().onTrue(new InstantCommand(() -> SimulatedArena.getInstance().clearGamePieces()));
        joystick.rightBumper().onTrue(new InstantCommand(() -> shooter.shoot()));

    }        

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private final StructArrayPublisher<Pose3d> crystalsPublisher =
        NetworkTableInstance.getDefault()
                .getStructArrayTopic("FieldSimulation/Crystals", Pose3d.struct)
                .publish();

    private final StructArrayPublisher<Pose3d> gemstonesPublisher =
        NetworkTableInstance.getDefault()
                .getStructArrayTopic("FieldSimulation/Gemstones", Pose3d.struct)
                .publish();

    public void updateSimulation() {
        crystalsPublisher.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Crystal"));
        gemstonesPublisher.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Gemstone"));
    }
}
