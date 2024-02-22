// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
   
    private final CANSparkMax liftMotor = new CANSparkMax (1,MotorType.kBrushless);
    private final CANSparkMax spinMotor = new CANSparkMax (2,MotorType.kBrushless );
    private final DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(0);
    public final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    
    private double upKp = 0.004;
    private double downKp = 0.003;
    private double upPosition = 67.5;
    private double downPosition = 169.0;
    private double intakePositionTarget = upPosition;
    private final PIDController pid = new PIDController(upKp, 0.0, 0.0);

    // create a max up in case the lift is up, this has to get reduced in periodic
    private double maxUpPosition = upPosition;

    private double maxLiftSpeedDown = 0.2;
    private double maxLiftSpeedUp = 0.5;
    private boolean isGoingDown = false;

    private final Shooter shooter;

    public Intake(Shooter shooter) {
        this.shooter = shooter;
        SmartDashboard.putNumber("intake kp up", upKp);
        SmartDashboard.putNumber("intake kp down", downKp);
        SmartDashboard.putNumber("lift speed up", maxLiftSpeedUp);
        liftMotor.setInverted(false);
        spinMotor.setInverted(true);
        distanceSensor.setAutomaticMode(true);
    }
    
    
    public void stopSpinMotor() {
        spinMotor.set(0.0);
    }

    public double getSensorAsDegrees() {
        return throughboreEncoder.getAbsolutePosition() * 360.0;
    }
    public double getDistanceSensorValue() {
       return distanceSensor.GetRange();
    }
    public boolean noteIsSeen() {
        //return noteSensorInput.get() ? false : true;
        return getDistanceSensorValue()<=16 ? true : false;
        
    }

        

    public void out() { 
        //this.intakePositionTarget = downPosition;
        spinMotor.set(-1.0);
    }

    public void down() {
        this.intakePositionTarget = downPosition;
        this.isGoingDown = true;
        spinMotor.set(1.0);
    }
    
    public void up() { 
        this.intakePositionTarget = upPosition;
        this.isGoingDown = false;
        this.stopSpinMotor();
    }

    public boolean isUp() {
        return (getSensorAsDegrees() < upPosition + 10);
    }
    public void intakeToIndex() {
        spinMotor.set(0.5);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("intake distance sensor", getDistanceSensorValue());
        // double upKp = SmartDashboard.getNumber("intake kp up", this.upKp);
        // double downKp = SmartDashboard.getNumber("intake kp down", this.downKp);
        // if(this.isGoingDown) {
        //     pid.setP(downKp);
        // } else {
        //     pid.setP(upKp);
        // }

        //SmartDashboard.putBoolean("note sensor", noteIsSeen());

        // uncomment when shooter lift is finished
        // maxUpPosition = (shooter.getLiftPosition > xyz) ? upPosition - 20 : upPosition;
        
        if(this.intakePositionTarget > maxUpPosition) this.intakePositionTarget = maxUpPosition;
        double speed = pid.calculate(this.getSensorAsDegrees(), this.intakePositionTarget);
        double upFromSD = SmartDashboard.getNumber("lift speed up", maxLiftSpeedUp);
        double maxLiftSpeed = (isGoingDown) ? maxLiftSpeedDown : upFromSD;
        if(Math.abs(speed) > maxLiftSpeed) speed = maxLiftSpeed * Math.signum(speed);

        if(isGoingDown && this.intakePositionTarget > 130.0 && this.intakePositionTarget < 160.0) {
            speed = -.1;
        }

        liftMotor.set(speed);

        SmartDashboard.putNumber("intake sensor", this.getSensorAsDegrees());
        SmartDashboard.putNumber("intake pid target", this.intakePositionTarget);
        SmartDashboard.putNumber("intake pid error", this.pid.getPositionError());
        SmartDashboard.putNumber("intake speed", speed);
    }
}
