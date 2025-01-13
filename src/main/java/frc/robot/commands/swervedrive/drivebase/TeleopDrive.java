// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class TeleopDrive extends Command
{

    private final SwerveSubsystem  swerve;
    private final DoubleSupplier   vX;
    private final DoubleSupplier   vY;
    private final DoubleSupplier   omega;
    private final BooleanSupplier  driveMode;
    private final SwerveController controller;
    private final BooleanSupplier autoCenter;
    double autoCenterKp = 0.028;
    double autoTurnKp = 0.02;
    LimeLight limeLight;
    BooleanSupplier isTurningToSource;
    BooleanSupplier isTurningToSpeaker;

    public TeleopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
                        BooleanSupplier driveMode, BooleanSupplier autoCenter, LimeLight l, BooleanSupplier isTurningToSource, BooleanSupplier isTurningToSpeaker)
    {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.controller = swerve.getSwerveController();
        this.autoCenter = autoCenter;
        this.limeLight = l;
        this.isTurningToSource = isTurningToSource;
        this.isTurningToSpeaker = isTurningToSpeaker;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        double angVelocity;

        if(autoCenter.getAsBoolean() && limeLight.isTarget()) {
            angVelocity = autoCenterKp * limeLight.getTx() * -1;
        
        } else if(isTurningToSource.getAsBoolean()) {
            double angle = swerve.getAllianceColor() == "red" ? -120.0 : 120.0;
            angVelocity = autoTurnKp * Math.IEEEremainder(angle - swerve.getHeading().getDegrees(), 360);

        } else if(isTurningToSpeaker.getAsBoolean()) {
            angVelocity = autoTurnKp * Math.IEEEremainder(0 - swerve.getHeading().getDegrees(), 360);       
        } else {
            angVelocity = Math.pow(omega.getAsDouble(), 3);
        }

        double xVelocity   = Math.pow(vX.getAsDouble(), 3);
        double yVelocity   = Math.pow(vY.getAsDouble(), 3);

        if(swerve.getAllianceColor() == "red") {
            xVelocity = xVelocity * -1;
            yVelocity = yVelocity * -1;
        }
        
        // Drive using raw values.
        swerve.drive(
            new Translation2d(xVelocity * swerve.maximumSpeed * 0.8, yVelocity * swerve.maximumSpeed * 0.8),
            angVelocity * controller.config.maxAngularVelocity * .75,
            driveMode.getAsBoolean()
        );
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
