// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import static edu.wpi.first.units.Units.*;


import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  SwerveDriveSimulation drivesim;
  SimulatedArena areana;
  IntakeSimulation intakesim;
  
 

  public IntakeSubsystem(SwerveDriveSimulation drivesim, SimulatedArena areana) {
    this.drivesim = drivesim;
    this.areana =  areana;

    intakesim = IntakeSimulation.InTheFrameIntake("Crystal", drivesim, Meters.of(0.65), IntakeSimulation.IntakeSide.BACK, 1);
  }

  

  public void startIntake(){
    intakesim.startIntake();
  }

  public void stopIntake(){
    intakesim.stopIntake();
  }

  public boolean getNote(){
    return intakesim.obtainGamePieceFromIntake();
  }

  public boolean hasNote(){
    return intakesim.getGamePiecesAmount() != 0;
  }

  @Override
  public void periodic() {
    DogLog.log("HasCrystal", hasNote());
    // This method will be called once per scheduler run
  }
}
