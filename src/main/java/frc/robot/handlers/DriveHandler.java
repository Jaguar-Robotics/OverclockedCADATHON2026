// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.DTPshi;

public class DriveHandler extends SubsystemBase {

    public enum DriveState implements State {
        TELEOPDRIVE,
        DRIVETOPOINTLOW,
        DRIVETOPOINTHIGH
  }


  private CommandSwerveDrivetrain drivetrain;
  private static DriveHandler instance;
  FollowPath.Builder pathBuilder;
  Command driveToPointCommand;



  private DriveState desiredState = DriveState.TELEOPDRIVE; 
  private DriveState currentState = DriveState.TELEOPDRIVE;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  PathConstraints DTPconstraints = new PathConstraints()
     .setMaxVelocityMetersPerSec(1.5)
     .setMaxAccelerationMetersPerSec2(2.0)
     .setEndTranslationToleranceMeters(0.05)
     .setEndRotationToleranceDeg(1);


  /** Creates a new IntakeHandler. */
  private DriveHandler() {
  }

  public void initialize(CommandSwerveDrivetrain drivetrain, FollowPath.Builder pathBuilder){
    this.drivetrain = drivetrain;
    this.pathBuilder = pathBuilder;
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
            case DRIVETOPOINTLOW:
                Pose2d DTPoint = DTPshi.findClosestLowCave(drivetrain.getSimPose());
                DogLog.log("Drive/DTPoint", DTPoint);
                Path DTPPath = new Path(
                  new Path.Waypoint(DTPoint));
                DTPPath.setPathConstraints(DTPconstraints); //slow it down 
                Command driveToPointCommand = pathBuilder.build(DTPPath);
                CommandScheduler.getInstance().schedule(driveToPointCommand);
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