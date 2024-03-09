// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.linearInterpolator;

public class Shooter extends SubsystemBase {
  
    private final CANSparkMax shooterA = new CANSparkMax(4, MotorType.kBrushless);
    private final CANSparkMax shooterB = new CANSparkMax(5, MotorType.kBrushless);
    private final CANSparkMax index = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax lift = new CANSparkMax(6, MotorType.kBrushless);
    private final DutyCycleEncoder liftThroughbore = new DutyCycleEncoder(1);
    private final Encoder shooterThroughbore = new Encoder(2, 3);
    private final BangBangController shooterController = new BangBangController();
   
    private double kP = 0.015;
    private double minLiftPosition = 20.5;
    private double maxLiftPosition = 56.0;
    private double liftPositionTarget = 48.0;

    private boolean isGoingDown = false;
    private double maxSpeedUp = 0.2;
    private double maxSpeedDown = 0.05;

    private final PIDController liftPid = new PIDController(kP, 0.0, 0.0);
    private double minRpm = 3000.0;
    private double maxRpm = 5300.0;
    private double liftDegreeOffset = 84.4;
    private double powerToHoldLiftLevel = 0.08;

    private boolean isTargettingAmp = false;

    private double tempAmpRpm = 800;
    private double tempAmpAngle = 48;

    private double [][] liftAngleData = {
        {50.0, 50.0},
        {72.0, 40.0},
        {90.0, 35.0},
        {110.0, 32.0},
        {140.0, 28},
        {155.0, 25},
        {175.0, 25},
        {180.0, 21.5}
    };

    private double [][] liftRpmData = {
        {58, 3400.0},
        {45, 3600.0},
        {39, 3800.0},
        {32, 4200.0},
        {28, 4700.0},
        {25, 5300.0},
        {21.5, 5300.0}
    };

    private final linearInterpolator liftAngleInterpolator = new linearInterpolator(liftAngleData);
    private final linearInterpolator liftRpmInterpolator = new linearInterpolator(liftRpmData);

    private LimeLight limeLight;

    public Shooter(LimeLight limelight) {
        this.limeLight = limelight;
        SmartDashboard.putNumber("lift kP", kP);
        liftPid.setTolerance(5, 8);


        shooterA.setInverted(false);
        shooterB.setInverted(false);
        lift.setInverted(true);
        stopIndexer();
        stop();

        // remove after amp testing
        // SmartDashboard.putNumber("temp amp rpm", tempAmpRpm);
        // SmartDashboard.putNumber("temp amp angle", tempAmpAngle);
    }

    public void 
    stop() {
        shooterA.set(0.0);
        shooterB.set(0.0);
    }
    public void out() {
        shooterA.set(-0.3);
        shooterB.set(-0.3);
        index.set(-0.2);
    }

    public double calculateTargetRpm() {
        double speed;
        if(isTargettingAmp) {
            speed = 1000;

            // remove after amp testing
           // speed = SmartDashboard.getNumber("temp amp rpm", tempAmpRpm);

    
        } else {
            // double liftRange = maxLiftPosition - minLiftPosition;
            // double positionInRange = liftPositionTarget - minLiftPosition;
            // double percentageUp = positionInRange / liftRange;
            // double rpmRange = maxRpm - minRpm;
            // speed = maxRpm -(rpmRange * percentageUp);

            speed = liftRpmInterpolator.getInterpolatedValue(liftPositionTarget);
            SmartDashboard.putNumber("shooter calculated speed", speed);
        }
        return speed;
    }

    public void setIsTargettingAmp(boolean v) {
        isTargettingAmp = v;
    }

    public boolean shoot() {
        double power = shooterController.calculate(getSpeed(), calculateTargetRpm());
        shooterA.set(power);
        shooterB.set(power);

        

        if (shooterIsAtTargetSpeed() && shooterLiftIsAtTarget()) {
            index.set(0.5);
        } else {
            index.set(0.0);
        }

        return shooterIsAtTargetSpeed() && shooterLiftIsAtTarget();
    }

    public boolean shooterIsAtTargetSpeed() {
        return getSpeed() >= calculateTargetRpm() * 0.96;
    }

    public double getSpeed() {
        return shooterThroughbore.getRate()/29;
    }

    public void indexGo() {
        index.set(0.2);
    }
    
    public void indexBack() {
        index.set(-0.2);
    }

    public void runIndexFromIntake() {
        index.set(0.3);
    }
    public void runIndexFromIntakeInverted(){
        index.set(-0.3);
    }

    public void stopIndexer(){
        index.set(0.0);
    }

    public double getLiftSensorAsDegrees() {
        return (liftThroughbore.getAbsolutePosition() * 360) - liftDegreeOffset;
    }

    public double liftFeedForward() {
        if(isGoingDown) return 0;
        return Math.cos(Math.toRadians(getLiftSensorAsDegrees())) * powerToHoldLiftLevel;
    }

    public void goToDownPosition() {
        setLiftPosition(minLiftPosition);
    }

    public void goToIntakePosition() {
        setLiftPosition(30);
    }
    
    public void setLiftPosition(double newPosition) {
        // remove after amp testing
        //if(isTargettingAmp) newPosition = SmartDashboard.getNumber("temp amp angle", tempAmpAngle);


        // is lift going up or down?
        isGoingDown = newPosition < getLiftSensorAsDegrees() ? true : false;
 
        this.liftPositionTarget = newPosition;
    }

    public boolean setLiftPositionFromDistance() {
        double distance = limeLight.getDistance();
        if(distance > 0 ) {
            double angle = liftAngleInterpolator.getInterpolatedValue(distance);

            isGoingDown = angle < getLiftSensorAsDegrees() ? true : false;

            this.liftPositionTarget = angle;
            SmartDashboard.putNumber("shooter april tag angle", angle);
        }

        // return whether or not the limelight saw a target
        return distance > 0;
    }

    public void moveLift() {
         // set min and max boundaries
         if(liftPositionTarget > maxLiftPosition) liftPositionTarget = maxLiftPosition;
         if(liftPositionTarget < minLiftPosition) liftPositionTarget = minLiftPosition;

        // if going down, handle speed
        if( isGoingDown && getLiftSensorAsDegrees() > 60 && liftPositionTarget < 25)  {
            lift.set(-.05);

        } else if(isGoingDown && getLiftSensorAsDegrees() > 40 && liftPositionTarget < 25) {
            lift.set(0.0);
        } else if(isGoingDown && getLiftSensorAsDegrees() > 25 && liftPositionTarget < 25) {
            lift.set(0.005);
        } else {
        // calculate power
            double kP = SmartDashboard.getNumber("lift kP", this.kP);
            liftPid.setP(kP);
            double power = liftPid.calculate(this.getLiftSensorAsDegrees(), this.liftPositionTarget) + liftFeedForward();

            double powerLimit = isGoingDown? maxSpeedDown : maxSpeedUp;

            power = power > Math.abs(powerLimit) ? powerLimit * Math.signum(power) : power;
            
            // set max power for moving lift
            SmartDashboard.putNumber("lift power", power);
            lift.set(power);
        }
    }

    public boolean shooterLiftIsAtTarget() {
        return liftPid.atSetpoint();
    }

    public void stopLift() {
        lift.set(0.0);
    }
    
    @Override
    public void periodic() {
        moveLift();

        SmartDashboard.putNumber("shooter speed", getSpeed());
        SmartDashboard.putNumber("lift degrees", getLiftSensorAsDegrees());
        SmartDashboard.putBoolean("lift is at target", liftPid.atSetpoint());
        SmartDashboard.putNumber("lift position target", liftPositionTarget);
    }
}
