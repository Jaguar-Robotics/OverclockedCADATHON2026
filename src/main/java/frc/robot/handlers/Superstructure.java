package frc.robot.handlers;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Superstructure extends SubsystemBase {

  // Define your states
  public enum SuperstructureState {
    IDLE,
    HANDOFFLOW,
    HANDOFFHIGH,
    HANDOFFSHOOTER,
    SHOOTING,
    SCORELOW,
    SCOREHIGH,
    CLIMBENGADGE,
    CLIMBED
  }

  private static Superstructure instance;
  private CommandSwerveDrivetrain drivetrain;
  private ElevatorSubsystem elevator;
  private ArmSubsystem arm;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private CommandXboxController joystick;
  
  private final ElevatorHandler elevatorHandler = ElevatorHandler.getInstance();
  private final ArmHandler armhandler = ArmHandler.getInstance();
  private final DriveHandler driveHandler = DriveHandler.getInstance();


  private SuperstructureState desiredState = SuperstructureState.IDLE;
  private SuperstructureState currentState = SuperstructureState.IDLE;

  private final Timer timer =  new Timer();
  

  public static Superstructure getInstance(){
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

    /**
   * Initialize superstructure with required subsystems
   */
  public void initialize( CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, ElevatorSubsystem elevator, ArmSubsystem arm, FollowPath.Builder pathBuilder, IntakeSubsystem intake, ShooterSubsystem shooter) {
      this.drivetrain = drivetrain;
      this.joystick = joystick;
      this.elevator = elevator;
      this.arm = arm;
      this.intake = intake;
      this.shooter = shooter;
      driveHandler.initialize(drivetrain, pathBuilder);
      elevatorHandler.initialize(elevator);
      armhandler.initialize(elevator, arm);
  }

  private Superstructure() {}

  public void setDriveState(DriveHandler.DriveState state){
    driveHandler.setDesiredState(state);
  }

  /** Call this from commands or joystick logic to set the next goal. */
  public void setDesiredState(SuperstructureState newState) {
    DogLog.log("States/Superstructure state", currentState);
    if (desiredState != newState || currentState.equals(SuperstructureState.HANDOFFSHOOTER)) {
      desiredState = newState;
      handleStateTransition();
    }
  }

  /** Handles transitions between states (2910-style logic). */
  private void handleStateTransition() {
    switch (desiredState) {
      case HANDOFFLOW:
          if (currentState.equals(SuperstructureState.SCOREHIGH)) {
            setDesiredState(SuperstructureState.SCORELOW); 
            break;}
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.IN);
        armhandler.setDesiredState(ArmHandler.ArmState.HANDOFF);
        break;
      case HANDOFFHIGH:
          if (currentState.equals(SuperstructureState.SCORELOW)) {
            setDesiredState(SuperstructureState.SCOREHIGH); 
            break;}
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.IN);
        armhandler.setDesiredState(ArmHandler.ArmState.HANDOFF);
        break;
      case SHOOTING:
      case HANDOFFSHOOTER:
          if(intake.NoteInEE){
            elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.IN);
            armhandler.setDesiredState(ArmHandler.ArmState.HANDOFF);
            timer.reset();
            timer.start();
            break;}
          elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.IN);
          armhandler.setDesiredState(ArmHandler.ArmState.CLIMB);
          break;
      case SCORELOW:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.IN);
        armhandler.setDesiredState(ArmHandler.ArmState.SCORE);
        intake.NoteInEE = true;
        intake.NoteinIntake = false;
        break;
      case SCOREHIGH:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.SCOREOUT);
        armhandler.setDesiredState(ArmHandler.ArmState.SCORE);
        intake.NoteInEE = true;
        intake.NoteinIntake = false;
        break;
      case CLIMBENGADGE:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.CLIMBOUT);
        armhandler.setDesiredState(ArmHandler.ArmState.CLIMB);
        break;
      case CLIMBED:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.CLIMBIN);
        armhandler.setDesiredState(ArmHandler.ArmState.CLIMB);
        drivetrain.setClimbVisualizationEnabled(true, elevator);
        break;
      case IDLE:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.IN);
        armhandler.setDesiredState(ArmHandler.ArmState.HANDOFF);
        break;
    }
    currentState = desiredState;
  }



  @Override
  public void periodic() {
    DogLog.log("SuperStructre/Superstructure State", currentState);
    DogLog.log("Arm/AtSetpoint", arm.isAtSetpoint());
    DogLog.log("Elevator/AtSetpoint", elevator.isAtSetpoint());
    if (currentState.equals(SuperstructureState.HANDOFFLOW) && arm.readyToHandoff() && elevator.readyToHandoff() && intake.NoteinIntake){
      setDesiredState(SuperstructureState.SCORELOW);
    }
    if (currentState.equals(SuperstructureState.HANDOFFHIGH) && arm.readyToHandoff() && elevator.readyToHandoff() && intake.NoteinIntake){
      setDesiredState(SuperstructureState.SCOREHIGH);
    }
    if (currentState.equals(SuperstructureState.HANDOFFSHOOTER) && arm.readyToHandoff() && elevator.readyToHandoff() && intake.NoteInEE){
      intake.NoteInEE = false;
      //setDesiredState(SuperstructureState.HANDOFFSHOOTER);
    }
    if (currentState.equals(SuperstructureState.IDLE) && arm.readyToHandoff() && elevator.readyToHandoff() && intake.NoteInEE){
      intake.NoteInEE = false;
    }
    if (currentState.equals(SuperstructureState.HANDOFFSHOOTER) && arm.awayFromShooter() && elevator.isAtSetpoint() && intake.NoteinIntake && joystick.b().getAsBoolean()){
      shooter.shoot();
    }
  }
  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public SuperstructureState getDesiredState() {
    return desiredState;
  }
}