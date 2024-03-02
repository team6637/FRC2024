// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutonIndexAndShoot extends Command {
    Shooter shooter;
    Intake intake;
    int counter;

    public AutonIndexAndShoot(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter, intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        counter = 0;

        intake.up();
        //shooter.goToIntakePosition();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean limelightWorked = shooter.setLiftPositionFromDistance();
        if(!limelightWorked) shooter.setLiftPosition(42);

        boolean shooting = shooter.shoot();

        if(intake.isUp() && shooting) {
            intake.intakeToIndex();
            counter++;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stopIndexer();
        shooter.stop();
        intake.stopSpinMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return counter > 30;
    }
}
