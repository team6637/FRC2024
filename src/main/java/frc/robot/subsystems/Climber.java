// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final TalonFX liftLeft = new TalonFX(0);
    private final TalonFX liftRight = new TalonFX(0);

    private final double minSetpoint = 0;
    private final double maxSetpoint = 100;

    private double setpoint = minSetpoint;
    private double kP = 0.0;
    private boolean usingPID = true;
    private double setpointIncrementer = 1.0;
    //private final PidConfig PidConfig = new PidConfig("extender", kp, Constants.ExtenderConstants.isTunable);
    private final PIDController pid = new PIDController(kP, 0.0, 0.0);
    
    public Climber() {
        liftLeft.setInverted(true);
        liftRight.setInverted(false);

        setSetpoint(minSetpoint);
    }
    
    public void extend() {
        if (usingPID) {
            setSetpoint(setpoint += setpointIncrementer);
        } else {
            liftLeft.set(0.3);
            liftRight.set(0.3);
        }
    }


    public void retract() {
        if (usingPID) {
            setSetpoint(setpoint -= setpointIncrementer);
        } else {
            liftLeft.set(-0.3);
            liftRight.set(-0.3);
        }
    }
        
        
        public void stop() {
            if (!usingPID) {
                liftLeft.set(0.0);
                liftRight.set(0.0);
            }
        }


    public void setSetpoint(double newSetpoint) {
        if (newSetpoint > maxSetpoint) {
            setpoint = maxSetpoint;
        } else if (newSetpoint < minSetpoint) {
            setpoint = minSetpoint;
        } else {
            setpoint = newSetpoint;
        }
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public double getPosition() {
        return 0.0;
    }

    @Override
    public void periodic() {
        double speed = pid.calculate(getPosition(), setpoint);
        if (usingPID) {
            liftLeft.set(speed);
            liftRight.set(speed);
        }
    }
}
