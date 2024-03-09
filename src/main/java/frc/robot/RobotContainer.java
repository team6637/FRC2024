// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

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
import frc.robot.commands.IndexSequentialCommand;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.Climber;
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
    public Climber climber = new Climber();

    public boolean autoCenter = false;

    SendableChooser<Command> chooser = new SendableChooser<>();

    public RobotContainer() {
        NamedCommands.registerCommand("autonShooterGoDown", new InstantCommand(()->shooter.goToDownPosition(), shooter));
        NamedCommands.registerCommand("autonShoot", new AutonShoot(shooter));
        NamedCommands.registerCommand("autonIntake", new AutonIntake(intake));
        NamedCommands.registerCommand("autonIndexFromIntake", new AutonIndexFromIntake(shooter, intake));
        NamedCommands.registerCommand("autonIndexAndShoot", new AutonIndexAndShoot(shooter, intake));

        configureBindings();

        TeleopDrive teleopFieldRelativeCommand = new TeleopDrive(
            drivebase,
            () -> -MathUtil.applyDeadband(joystick.getY(), OperatorConstants.Y_DEADBAND),
            () -> -MathUtil.applyDeadband(joystick.getX(), OperatorConstants.Y_DEADBAND),
            () -> -MathUtil.applyDeadband(joystick.getTwist(), OperatorConstants.TWIST_DEADBAND),
            () -> true,
            () -> autoCenter,
            limelight
        );

        drivebase.setDefaultCommand(teleopFieldRelativeCommand);

        chooser.setDefaultOption("Three Front Notes", drivebase.getPathPlannerAuto("three-front-notes", true));
        chooser.addOption("Front Notes", drivebase.getPathPlannerAuto("front-notes", true));
        chooser.addOption("One Close One far (left)", drivebase.getPathPlannerAuto("one-close-one-far left", true));
        chooser.addOption("Pick Up Two far (right)", drivebase.getPathPlannerAuto
        ("pick-up-two-far right", true));
        chooser.addOption("Shoot and Cross", drivebase.getPathPlannerAuto
        ("Shoot and Cross ", true));
        chooser.addOption("Cross Line", drivebase.getPathPlannerAuto
        ("Cross Line", true));
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

        //INDEX
        new JoystickButton(joystick, 2).onTrue(
            new RunCommand(()->shooter.indexGo(), shooter)
        ).onFalse(
            new InstantCommand(()->shooter.stopIndexer(), shooter)
        );
        
        // SHOOT
        new JoystickButton(joystick, 1).onTrue(
                new RunCommand(()->{
                    limelight.setVisionMode("april");
                    boolean isAtSpeed = shooter.shoot();
                    if(isAtSpeed) intake.indexForShooter();
                }, shooter, intake)
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->shooter.stop(), shooter),    
                new InstantCommand(()->shooter.stopIndexer(), shooter),
                new InstantCommand(()->intake.stopSpinMotor(), intake),
                new InstantCommand(()->limelight.setVisionMode("off"))
            )
        );

        // Climber Up
        new JoystickButton(joystick, 13).onTrue(
            new InstantCommand(()->climber.extend(), climber)
        ).onFalse(
            new InstantCommand(()->climber.stop(), climber)
        );
        
        // Climber Down 
        new JoystickButton(joystick, 12).onTrue(
            new InstantCommand(()->climber.retract(), climber)
        ).onFalse(
            new InstantCommand(()->climber.stop(), climber)
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


       // go to apriltag angle 
        new JoystickButton(joystick, 5).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()->shooter.setLiftPositionFromDistance(), shooter),
                new RunCommand(()->{
                    autoCenter = true;
                })
            )
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->shooter.goToDownPosition(), shooter),
                new InstantCommand(()->{
                    autoCenter = false;
                })
            )
        );


       // go to amp angle
        new JoystickButton(joystick, 8).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(()->shooter.setIsTargettingAmp(true), shooter),
                new InstantCommand(()->shooter.setLiftPosition(46), shooter)
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
