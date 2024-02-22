// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
//import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IndexSequentialCommand;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.commands.swervedrive.testing.DriveBack2Meters;
import frc.robot.commands.swervedrive.testing.DriveForward2Meters;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {

    // public enum State {
    //     HUNGRY,
    //     INTAKE_HAS_NOTE,
    //     INDEXING,
    //     NOTE_IS_IN_INDEXER,
    //     SHOOTING
    // }
    // public static State robotState = State.HUNGRY;

    private final SwerveSubsystem drivebase = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(),"swerve")
    );
    
    Joystick joystick = new Joystick(0);
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake(shooter);


    SendableChooser<Command> chooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        TeleopDrive teleopFieldRelativeCommand = new TeleopDrive(
            drivebase,
            () -> -MathUtil.applyDeadband(joystick.getY(), OperatorConstants.Y_DEADBAND),
            () -> -MathUtil.applyDeadband(joystick.getX(), OperatorConstants.Y_DEADBAND),
            () -> -MathUtil.applyDeadband(joystick.getTwist(), OperatorConstants.TWIST_DEADBAND),
            () -> true
        );

        drivebase.setDefaultCommand(teleopFieldRelativeCommand);

        chooser.setDefaultOption("Reset Pose", new InstantCommand(()->drivebase.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), drivebase.getHeading()))));
        chooser.addOption("Go Forward 2M", new DriveForward2Meters(drivebase));
        chooser.addOption("Go Backwards 2M", new DriveBack2Meters(drivebase));

        SmartDashboard.putData(chooser);

    }

    private void configureBindings() {

        // while intake button is held
        new JoystickButton(joystick, 3).onTrue(
            // run the intake down and spin wheels
            new SequentialCommandGroup(
                new RunCommand(()->intake.down(), intake).until(()->intake.noteIsSeen()),
                new InstantCommand(()->intake.up(), intake)
            )

        // when button is let up, lift up intake ONLY if state is still HUNGRY
        ).onFalse(
          new ConditionalCommand(
                new IndexSequentialCommand(intake, shooter),
                new InstantCommand(()->intake.up(), intake), 
                ()->intake.noteIsSeen()
            )
        );




        // INTAKE
        new JoystickButton(joystick, 4).onTrue(
            new RunCommand(()->intake.out(),intake)
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->intake.up(), intake),
                new InstantCommand(()->intake.stopSpinMotor(), intake),    
                new InstantCommand(()->shooter.stopIndexer(), shooter)
            )
        );
        
        // SHOOT
        new JoystickButton(joystick, 1).onTrue(
            new RunCommand(()->shooter.shoot(),shooter)
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->shooter.stop(), shooter),    
                new InstantCommand(()->shooter.stopIndexer(), shooter)
            )
        );

        // INDEX
        // new JoystickButton(joystick, 8).whileTrue(
        //     new RunCommand(()->shooter.runIndexFromIntake(),shooter))
        //     .onFalse(new InstantCommand(()->shooter.stopIndexer()));

    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

}
