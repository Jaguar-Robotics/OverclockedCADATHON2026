// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoStuff.BlineStuff;
import frc.robot.generated.TunerConstants;
import frc.robot.handlers.DriveHandler;
import frc.robot.handlers.Superstructure;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
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
    public final IntakeSubsystem intake =  new IntakeSubsystem(drivetrain.getSimDrivetrain(), SimulatedArena.getInstance(), drivetrain);
    public final ShooterSubsystem shooter = new ShooterSubsystem(drivetrain, intake); //only here because theres no shooterhandler
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem(elevator, drivetrain, intake);
    public final Superstructure superstructure = Superstructure.getInstance(); 
 
    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    

    //BLINE
    FollowPath.Builder pathBuilder = new FollowPath.Builder(
    drivetrain,
    drivetrain::getPose,
    drivetrain::getRobotRelativeSpeeds,
    drivetrain::driveRobotRelative,
    new PIDController(5, 0.0, 1.5), //10, 7, 5
    new PIDController(2, 0.0, 0.1),
    new PIDController(7, 0.0, 2)
        ).withTRatioBasedTranslationHandoffs(true);


    private final Map<String, Command> autos_BLine = BlineStuff.buildAutos(pathBuilder, drivetrain);

    final Trigger noButtonsHeld = new Trigger(() ->
        !joystick.a().getAsBoolean() &&
        !joystick.b().getAsBoolean() &&
        !joystick.y().getAsBoolean() &&
        !joystick.x().getAsBoolean() &&
        !joystick.rightTrigger().getAsBoolean() &&
        !joystick.rightBumper().getAsBoolean() &&
        !joystick.leftTrigger().getAsBoolean() &&
        !joystick.leftBumper().getAsBoolean()
        );

    


    public RobotContainer() {

        Superstructure.getInstance().initialize(drivetrain, joystick, elevator, arm, pathBuilder, intake, shooter);
        
        drivetrain.setParamsDT(joystick, MaxSpeed, MaxAngularRate, drive, drivetrain);

        configureBindings();

        SimulatedArena.getInstance().resetFieldForAuto();
        drivetrain.resetPose(new Pose2d(10, 4.0, Rotation2d.fromDegrees(0))); 
        configureAutoChooser();
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
        joystick.y().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.HANDOFFHIGH)));
        joystick.x().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.HANDOFFLOW)));
        joystick.a().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
        joystick.leftBumper().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.HANDOFFLOW)));
        joystick.b().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.HANDOFFSHOOTER)));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> arm.score()));


        joystick.leftTrigger().onTrue(new InstantCommand(() -> MineSim.drop(MineSim.getNearestStation(drivetrain.getSimPose()), false)));
        joystick.leftTrigger().onTrue(new InstantCommand(() -> intake.startIntake()));
        joystick.leftTrigger().onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
        joystick.leftTrigger().onFalse(new InstantCommand(() -> intake.stopIntake()));
        joystick.start().onTrue(new InstantCommand(() -> SimulatedArena.getInstance().clearGamePieces()));

        joystick.back().onTrue(new InstantCommand(() -> drivetrain.resetPose(new Pose2d(10, 4.0, Rotation2d.fromDegrees(0)))));
        joystick.rightBumper().onTrue(new InstantCommand(() -> superstructure.setDriveState(DriveHandler.DriveState.DRIVETOPOINTLOW)));
        joystick.rightBumper().onFalse(new InstantCommand(() -> superstructure.setDriveState(DriveHandler.DriveState.TELEOPDRIVE)));
        

        //noButtonsHeld.onTrue(new InstantCommand(() -> superstructure.setDesiredState(Superstructure.SuperstructureState.IDLE)));
    }        

    public Command getAutonomousCommand() {
        String key = autoChooser.getSelected();
        if (key == null) return Commands.none();

        Command auto = autos_BLine.get(key);
        return auto != null ? auto : Commands.none();
    }

    private void configureAutoChooser() {
        boolean first = true;

        for (String key : autos_BLine.keySet()) {
            if (first) {
                autoChooser.setDefaultOption(key, key);
                first = false;
            } else {
                autoChooser.addOption(key, key);
            }
        }

        SmartDashboard.putData("Autonomous Mode", autoChooser);
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
        DogLog.log("Zeroed Components", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()}); //last stage of EE, mid stage EE, arm
        DogLog.log("Final component poses", new Pose3d[] {elevator.getElevatorPose3d(), elevator.getElevatorLilPose3d(), arm.getArmPose3d(elevator)}); //last stage of EE, midstage Ee, arm
    }
}
