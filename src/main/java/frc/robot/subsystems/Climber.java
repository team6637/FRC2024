// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final TalonFX liftLeft = new TalonFX(17, "Gary");
    private final TalonFX liftRight = new TalonFX(16, "Gary");

    private final double minSetpoint = 0;
    private final double maxSetpoint = 96;

    private double setpoint = minSetpoint;
    private double kP = 0.0;
    private boolean usingPID = false;
    private double setpointIncrementer = 1.0;
    private final PIDController leftPid = new PIDController(kP, 0.0, 0.0);
    private final PIDController rightPid = new PIDController(kP, 0.0, 0.0);
    
    public Climber() {
        liftLeft.setInverted(true);
        //liftLeft.setNeutralMode();
        
        liftRight.setInverted(true);

        setSetpoint(minSetpoint);

        SmartDashboard.putNumber("climber kp", kP);
    }
    
    public void extend() {
        if (usingPID) {
            setSetpoint(setpoint += setpointIncrementer);
        } else {
            if (liftLeft.getPosition().getValueAsDouble() >= 96) {
                liftLeft.set(0.0);
            } else {
                liftLeft.set(1.0);
            }

            if (liftRight.getPosition().getValueAsDouble() >= 96) {
                liftRight.set(0.0);
            } else {
                liftRight.set(1.0);
            }    
        }
    }

    public void retract() {
       if (usingPID) {
           setSetpoint(setpoint -= setpointIncrementer);
        } else {
            if (liftLeft.getPosition().getValueAsDouble() <= 0) {
                liftLeft.set(0.0);
            } else {
                liftLeft.set(-1.0);
            }
            if (liftRight.getPosition().getValueAsDouble() <= 0) {
                liftRight.set(0.0);
            } else {
                liftRight.set(-1.0);
            }
        }
    }
        
    public void stop() {
        liftLeft.set(0.0);
        liftRight.set(0.0);
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
        return leftPid.atSetpoint() && rightPid.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber left", liftLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("climber right", liftRight.getPosition().getValueAsDouble());
        if (usingPID) {

            // delete this group after tuning
            double kP = SmartDashboard.getNumber("climber kp", this.kP);
            leftPid.setP(kP);
            rightPid.setP(kP);

            double leftSpeed = leftPid.calculate(liftLeft.getPosition().getValueAsDouble(), setpoint);
            liftLeft.set(leftSpeed);

            double rightSpeed = rightPid.calculate(liftRight.getPosition().getValueAsDouble(), setpoint);
            liftRight.set(rightSpeed);
        }
    }
}
