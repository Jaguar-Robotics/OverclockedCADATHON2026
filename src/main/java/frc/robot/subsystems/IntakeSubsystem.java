// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class IntakeSubsystem extends SubsystemBase {

  SwerveDriveSimulation drivesim;
  CommandSwerveDrivetrain drivetrain;
  SimulatedArena areana;
  IntakeSimulation intakesim;
  public boolean NoteInEE = false;
  public boolean NoteinIntake = false;
  
 

  public IntakeSubsystem(SwerveDriveSimulation drivesim, SimulatedArena areana, CommandSwerveDrivetrain drivetrain) {
    this.drivesim = drivesim;
    this.areana =  areana;
    this.drivetrain = drivetrain;

    intakesim = IntakeSimulation.InTheFrameIntake("Crystal", drivesim, Inches.of(25.5), IntakeSimulation.IntakeSide.BACK, 1);
  }

    public final StructPublisher<Pose3d> heldGemstonePublisher =
    NetworkTableInstance.getDefault()
        .getStructTopic("FieldSimulation/Held Gemstone", Pose3d.struct)
        .publish();

  

  public void startIntake(){
    intakesim.startIntake();
  }

  public void stopIntake(){
    intakesim.stopIntake();
  }

  public boolean getNote(){
    NoteInEE = false;
    heldGemstonePublisher.set(new Pose3d()); 
    return intakesim.obtainGamePieceFromIntake();
  }

  public boolean hasNote(){
    return intakesim.getGamePiecesAmount() != 0;
  }

  public void upDateIntakes(){
    if (intakesim.getGamePiecesAmount() != 0 && !NoteInEE){
      NoteinIntake = true;
    }
    else {NoteinIntake = false;}
  }

  @Override
  public void periodic() {
    upDateIntakes();
    DogLog.log("HasCrystal", hasNote());
    DogLog.log("Intake/NoteInEE", NoteInEE);
    DogLog.log("Intake/NoteInIntake", NoteinIntake);
    if (NoteinIntake){
      heldGemstonePublisher.set(new Pose3d(drivetrain.getSimPose()).transformBy(new Transform3d(Constants.IntakeConstants.GemstoneOffset.getTranslation(), Constants.IntakeConstants.GemstoneOffset.getRotation())));
    }
  }
}
