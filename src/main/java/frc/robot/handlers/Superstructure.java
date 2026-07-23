package frc.robot.handlers;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Superstructure extends SubsystemBase {

  // Define your states
  public enum SuperstructureState {
    IDLE,
    HANDOFF,
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
  private CommandXboxController joystick;
  
  private final ElevatorHandler elevatorHandler = ElevatorHandler.getInstance();
  private final ArmHandler armhandler = ArmHandler.getInstance();
  private final DriveHandler driveHandler = DriveHandler.getInstance();


  private SuperstructureState desiredState = SuperstructureState.IDLE;
  private SuperstructureState currentState = SuperstructureState.IDLE;
  

  public static Superstructure getInstance(){
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

    /**
   * Initialize superstructure with required subsystems
   */
  public void initialize( CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, ElevatorSubsystem elevator, ArmSubsystem arm, FollowPath.Builder pathBuilder, IntakeSubsystem intake) {
      this.drivetrain = drivetrain;
      this.joystick = joystick;
      this.elevator = elevator;
      this.arm = arm;
      this.intake = intake;
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
    if (desiredState != newState) {
      desiredState = newState;
      handleStateTransition();
    }
  }

  /** Handles transitions between states (2910-style logic). */
  private void handleStateTransition() {
    switch (desiredState) {
      case HANDOFF:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.IN);
        armhandler.setDesiredState(ArmHandler.ArmState.HANDOFF);
        if (intake.hasNote()) {intake.NoteInEE = true;}
        break;
      case SCORELOW:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.IN);
        armhandler.setDesiredState(ArmHandler.ArmState.SCORE);
        break;
      case SCOREHIGH:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.OUT);
        armhandler.setDesiredState(ArmHandler.ArmState.SCORE);
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
         if(currentState.equals(SuperstructureState.IDLE) && intake.hasNote()){
          setDesiredState(SuperstructureState.HANDOFF);
         }
  }
  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public SuperstructureState getDesiredState() {
    return desiredState;
  }
}