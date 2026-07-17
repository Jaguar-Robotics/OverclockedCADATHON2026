// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.simulation.crystalcaverns.CrystalOnFly;

public class ShooterSubsystem extends SubsystemBase {

  CommandSwerveDrivetrain drivetrain;
  /** Creates a new EESubsystem. */
  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
  this.drivetrain = drivetrain;
}

  public void shoot(){
    GamePieceProjectile shotNote;

    shotNote = new CrystalOnFly(
      drivetrain.getSimPose().getTranslation(),
      Translation2d.kZero,
      drivetrain.getSimChassisSpeed(),
      drivetrain.getSimPose().getRotation(),
      Inches.of(12.225),
      FeetPerSecond.of(40), //SHOOTING SPEED CHANGE ME LATER 
      Degrees.of(38));

      SimulatedArena.getInstance().addGamePieceProjectile(shotNote);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
