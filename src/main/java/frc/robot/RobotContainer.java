// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonIndexAndShoot;
import frc.robot.commands.AutonIndexFromIntake;
import frc.robot.commands.AutonIntake;
import frc.robot.commands.AutonShoot;
import frc.robot.commands.AutonShooterPrime;
import frc.robot.commands.IndexSequentialCommand;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem drivebase = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(),"swerve")
    );
    
    Joystick joystick = new Joystick(0);
    public LimeLight limelight = new LimeLight();
    public Shooter shooter = new Shooter(limelight);
    public Intake intake = new Intake(shooter);


    SendableChooser<Command> chooser = new SendableChooser<>();

    public RobotContainer() {
        NamedCommands.registerCommand("autonShoot", new AutonShoot(shooter));
        NamedCommands.registerCommand("autonIntake", new AutonIntake(intake));
        NamedCommands.registerCommand("autonIndexFromIntake", new AutonIndexFromIntake(shooter, intake));
        NamedCommands.registerCommand("autonShooterPrime", new AutonShooterPrime(shooter));
        NamedCommands.registerCommand("autonIndexAndShoot", new AutonIndexAndShoot(shooter, intake));

        configureBindings();

        TeleopDrive teleopFieldRelativeCommand = new TeleopDrive(
            drivebase,
            () -> -MathUtil.applyDeadband(joystick.getY(), OperatorConstants.Y_DEADBAND),
            () -> -MathUtil.applyDeadband(joystick.getX(), OperatorConstants.Y_DEADBAND),
            () -> -MathUtil.applyDeadband(joystick.getTwist(), OperatorConstants.TWIST_DEADBAND),
            () -> true
        );

        drivebase.setDefaultCommand(teleopFieldRelativeCommand);

        chooser.setDefaultOption("Front Notes", drivebase.getPathPlannerAuto("front-notes", true));
        chooser.addOption("Test Index And Shoot", new AutonIndexAndShoot(shooter, intake));

        SmartDashboard.putData(chooser);
    }

    private void configureBindings() {

        // INTAKE IN
        new JoystickButton(joystick, 3).onTrue(
            // run the intake down and spin wheels
            new SequentialCommandGroup(
                new InstantCommand(()->shooter.goToIntakePosition(), shooter),
                new RunCommand(()->intake.down(), intake).until(()->intake.noteIsSeen()),
                //new RunCommand(()->intake.out(), intake).withTimeout(0.05),
                new InstantCommand(()->intake.up(), intake)

            )

        ).onFalse(
            new ConditionalCommand(
                new IndexSequentialCommand(intake, shooter),
                new SequentialCommandGroup(
                    new InstantCommand(()->shooter.goToDownPosition(), shooter),
                    new InstantCommand(()->intake.up(), intake)
                ),
                ()->intake.noteIsSeen()
            )
        );

        // INTAKE OUT
        new JoystickButton(joystick, 4).onTrue(
           new ParallelCommandGroup(
                new RunCommand(()->intake.out(), intake),
                new RunCommand(()->shooter.out(), shooter)
                )
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->intake.up(), intake),
                new InstantCommand(()->intake.stopSpinMotor(), intake),    
                new InstantCommand(()->shooter.stopIndexer(), shooter),
                new InstantCommand(()->shooter.stop(), shooter)
            )
        );

        new JoystickButton(joystick, 2).onTrue(
            new RunCommand(()->shooter.indexGo(), shooter)
        ).onFalse(
            new InstantCommand(()->shooter.indexStop(), shooter)
        );
        
        // SHOOT
        new JoystickButton(joystick, 1).onTrue(
                new RunCommand(()->{
                    boolean isAtSpeed = shooter.shoot();
                    if(isAtSpeed) intake.indexForShooter();
                }, shooter, intake)
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->shooter.stop(), shooter),    
                new InstantCommand(()->shooter.stopIndexer(), shooter),
                new InstantCommand(()->intake.stopSpinMotor(), intake)
            )
        );

        // go to speaker angle
        new JoystickButton(joystick, 7).onTrue(
            new InstantCommand(()->shooter.setLiftPosition(55), shooter)
        ).onFalse(
            new InstantCommand(()->shooter.goToDownPosition(), shooter)
        );
      
      
        // go to sweet spot button 6
        new JoystickButton(joystick, 6).onTrue(
            new InstantCommand(()->shooter.setLiftPosition(30), shooter)
        ).onFalse(
            new InstantCommand(()->shooter.goToDownPosition(), shooter)
        );

       //go to apriltag angle 
        new JoystickButton(joystick, 5).onTrue(
            new InstantCommand(()->shooter.setLiftPositionFromDistance(), shooter)
        ).onFalse(
            new InstantCommand(()->shooter.goToDownPosition(), shooter)
        );


        





       // go to amp angle
        new JoystickButton(joystick, 8).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()->shooter.setLiftPosition(45), shooter),
                new InstantCommand(()->shooter.setIsTargettingAmp(true), shooter)
            )
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->shooter.goToDownPosition(), shooter),
                new InstantCommand(()->shooter.setIsTargettingAmp(false), shooter)
            )
        );
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }

}
