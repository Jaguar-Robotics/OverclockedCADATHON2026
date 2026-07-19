package frc.robot.handlers;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmHandler extends SubsystemBase implements StateSubsystem {

  ElevatorSubsystem elevator;
  ArmSubsystem arm;

    public enum ArmState implements State {
        HANDOFF,
        SCORE,
        CLIMB,
        BRAKE
  }

  private static ArmHandler instance;

  private ArmState desiredState = ArmState.HANDOFF;
  private ArmState currentState = ArmState.HANDOFF;
  /** Creates a new IntakeHandler. */
  private ArmHandler() {}

  public static ArmHandler getInstance(){
    if (instance == null){
      instance = new ArmHandler();
    }
    return instance;
  }

  public void initialize(ElevatorSubsystem elevator, ArmSubsystem arm){
    this.elevator = elevator;
    this.arm = arm;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof ArmState armState && desiredState != armState) {
        desiredState = armState;
    }
  }

  @Override
  public void handleStateTransition() {
    update();
  }

      @Override
    public void update() {
      if((currentState != desiredState)){
        switch (desiredState) {
            case HANDOFF:
            arm.goToSetpoint(Constants.ArmConstants.Handoff);
                break;
            case SCORE:
            arm.goToSetpoint(Constants.ArmConstants.Scoring);
                break;
            case CLIMB:
            arm.goToSetpoint(Constants.ArmConstants.climb);
                break;
            case BRAKE:
            arm.holdPosition();
                break;
        }
        currentState = desiredState;
    }
  }

      public ArmState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    DogLog.log("ArmHandler/CurrentState", currentState);
    update();
    // This method will be called once per scheduler run
  }
}
