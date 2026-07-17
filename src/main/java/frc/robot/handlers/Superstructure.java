package frc.robot.handlers;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Superstructure extends SubsystemBase {

  // Define your states
  public enum SuperstructureState {
    IDLE,
    EXTEND,
    OFF
  }

  private static Superstructure instance;
  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController joystick;
  
  private final ElevatorHandler elevatorHandler = ElevatorHandler.getInstance();

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
  public void initialize( CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
      this.drivetrain = drivetrain;
      this.joystick = joystick;
  }

  private Superstructure() {}

  /** Call this from commands or joystick logic to set the next goal. */
  public void setDesiredState(SuperstructureState newState) {
    System.out.print("SetDesiredState:" + newState);
    if (desiredState != newState) {
      desiredState = newState;
      handleStateTransition();
    }
  }

  /** Handles transitions between states (2910-style logic). */
  private void handleStateTransition() {
    switch (desiredState) {
      case IDLE:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.IN);
        break;
      case EXTEND:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.OUT);
        break;
      case OFF:
        elevatorHandler.setDesiredState(ElevatorHandler.ElevatorState.BRAKE);
        break;
    }
    currentState = desiredState;
  }

  @Override
  public void periodic() {
         
  }
  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public SuperstructureState getDesiredState() {
    return desiredState;
  }
}