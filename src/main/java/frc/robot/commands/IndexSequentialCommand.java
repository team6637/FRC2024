// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IndexSequentialCommand extends SequentialCommandGroup {

    public IndexSequentialCommand(Intake intake, Shooter shooter) {        
        addCommands(
            new InstantCommand(()->{
                intake.up();
                //shooter.goToIndexPosition();
            }, intake, shooter),
            new WaitUntilCommand(()->intake.isUp()),
            new ParallelRaceGroup(
                new WaitUntilCommand(()->!intake.noteIsSeen()),
                new RunCommand(()->shooter.runIndexFromIntake(), shooter),
                new RunCommand(()->intake.intakeToIndex(), intake)
            ),
            new InstantCommand(()->intake.stopSpinMotor(), intake),
            //new RunCommand(()->shooter.runIndexFromIntakeInverted(), shooter).withTimeout(0.2),
            new InstantCommand(()->shooter.stopIndexer(), shooter),
            new InstantCommand(()->shooter.goToDownPosition(), shooter)
        );

    }
}
