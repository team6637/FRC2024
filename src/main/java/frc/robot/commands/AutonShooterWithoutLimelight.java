// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutonShooterWithoutLimelight extends Command {
  
    Shooter shooter;
    int counter;

  public AutonShooterWithoutLimelight(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setLiftPosition(235);
   
    boolean shooting = shooter.shoot();
    if(shooting) {
        counter++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        shooter.stopIndexer();
        shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter > 10;
  }
}
