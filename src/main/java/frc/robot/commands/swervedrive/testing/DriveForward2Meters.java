// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.testing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveForward2Meters extends SequentialCommandGroup {

    public DriveForward2Meters(SwerveSubsystem drivebase) {
    addCommands(
        new InstantCommand(()->{
            drivebase.resetOdometry(new Pose2d(new Translation2d(2.0, 5.0), drivebase.getHeading()));
        }),
        drivebase.getAutonomousCommand("driveForward2meters", false)
    );
  }
}
