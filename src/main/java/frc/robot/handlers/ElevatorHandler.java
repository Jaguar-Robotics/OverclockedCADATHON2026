// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorHandler extends SubsystemBase implements StateSubsystem {

    public enum ElevatorState implements State {
      OUT,
      MID,
      IN,
      BRAKE
  }
  private ElevatorSubsystem elevator;
  private static ElevatorHandler instance;

  private ElevatorState desiredState = ElevatorState.IN;
  private ElevatorState currentState = ElevatorState.IN;
  /** Creates a new IntakeHandler. */
  private ElevatorHandler() {}

  public static ElevatorHandler getInstance(){
    if (instance == null){
      instance = new ElevatorHandler();
    }
    return instance;
  }

  public void initialize(ElevatorSubsystem elevator){
    this.elevator = elevator;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof ElevatorState elevatorState && desiredState != elevatorState) {
        desiredState = elevatorState;
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
            case OUT:
                elevator.goToSetpoint(Constants.ElevatorConstants.Out);
                break;
            case MID:
                elevator.goToSetpoint(Constants.ElevatorConstants.Mid);
                break;
            case IN:
                elevator.goToSetpoint(Constants.ElevatorConstants.In);
                break;
            case BRAKE:
                elevator.holdPosition();
                break;
        }
        currentState = desiredState;
    }
  }

      public ElevatorState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    DogLog.log("ElevatorHandler/CurrentState", currentState);
    update();
    // This method will be called once per scheduler run
  }
}
