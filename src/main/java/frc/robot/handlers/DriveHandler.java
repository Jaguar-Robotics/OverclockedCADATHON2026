// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveHandler extends SubsystemBase {

    public enum DriveState implements State {
        TELEOPDRIVE
  }


  private CommandSwerveDrivetrain drivetrain;
  private static DriveHandler instance;



  private DriveState desiredState = DriveState.TELEOPDRIVE; 
  private DriveState currentState = DriveState.TELEOPDRIVE;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /** Creates a new IntakeHandler. */
  private DriveHandler() {
  }

  public void initialize(CommandSwerveDrivetrain drivetrain){
    this.drivetrain = drivetrain;
    update(); 
  }

  public static DriveHandler getInstance(){
    if (instance == null){
      instance = new DriveHandler();
    }
    return instance;
  }

  
  public void setDesiredState(DriveState state){
        if (desiredState != state) {
        desiredState = state;
        updateStates();
    }
  }

  public void updateStates(){
    if (currentState != desiredState) {
        currentState = desiredState;
        update();
    }
  }

  public void handleStateTransition() {
    update();
  }

    public void update() {
    if (drivetrain.getCurrentCommand() != null) {
        CommandScheduler.getInstance().cancel(drivetrain.getCurrentCommand());
    }
        switch (desiredState) {
            case TELEOPDRIVE:
                drivetrain.setDefaultCommand(drivetrain.TeleopDrive());
                break;
            default:
                drivetrain.setDefaultCommand(drivetrain.TeleopDrive());
                break;
        }
        currentState = desiredState;
    }

      public DriveState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    DogLog.log("DriveHandler/DriveState", currentState.toString());
    // This method will be called once per scheduler run
  }
}