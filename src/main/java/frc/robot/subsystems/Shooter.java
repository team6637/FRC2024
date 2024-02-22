// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
    
    //private final TalonFX shooterA = new TalonFX(4, "Gary");
    //private final TalonFX shooterB = new TalonFX(5, "Gary");
    private final CANSparkMax shooterA = new CANSparkMax(4, MotorType.kBrushless);
    private final CANSparkMax shooterB = new CANSparkMax(5, MotorType.kBrushless);
    private final CANSparkMax index = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax lift = new CANSparkMax(6, MotorType.kBrushless);
    private final DutyCycleEncoder liftThroughbore = new DutyCycleEncoder(1);
    private final Encoder shooterThroughbore = new Encoder(2, 3);
    private final BangBangController shooterController = new BangBangController();
   
    private double kP = 0.0;
    private double liftIndexPosition = 63.0;
    private double liftAmpPosition = 83.0;
    private double liftSpeakerPosition = 169.0;
    private final PIDController pid = new PIDController(kP, 0.0, 0.0);
    private double liftPositionTarget = liftSpeakerPosition;
    private double shooterSetPoint = 5000.0;
    //need to change 2000 for later


    public Shooter() {
        SmartDashboard.putNumber("shooter setpoint",shooterSetPoint);
        SmartDashboard.putNumber("lift kP", kP);
        shooterA.setInverted(false);
        shooterB.setInverted(false);
        lift.setInverted(true);
        stopIndexer();
    }
    public void stop() {
        shooterA.set(0.0);
        shooterB.set(0.0);
    }
    public void shoot() {
        double setpoint = SmartDashboard.getNumber("shooter setpoint", shooterSetPoint);
        double power = shooterController.calculate(getSpeed(),setpoint);
        SmartDashboard.putNumber("shooter bang bang power ",power);
        shooterA.set(power);
        shooterB.set(power);
        
        if (getSpeed() >= setpoint) {
            index.set(0.5);
        } else {
            index.set(0.0);
        }
    }
    

    public double getSensorAsDegrees() {
        return liftThroughbore.getAbsolutePosition() * 360;
    }

    // todo after shooter lift is in place
    // move to index position
    public void goToIndexPosition() {

    }
    
    public void setSetpoints(double s){
        SmartDashboard.putNumber("shooter target speed", s);

    }
    public void stopIndexer(){
        index.set(0.0);
    }


    public void runIndexFromIntake() {
        index.set(0.5);
    }
    public void runIndexFromIntakeInverted(){
        index.set(-0.3);
    }

    public double getSpeed() {
        return shooterThroughbore.getRate()/29;
      }

    
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter speed", getSpeed());
        pid.setP(kP);
        double kP = SmartDashboard.getNumber("lift kP", this.kP);
        double speed = pid.calculate(this.getSensorAsDegrees(), this.liftPositionTarget);
        lift.set(speed);


       
        //speed = 0.6;
        //index.set(speed);
    }
}
