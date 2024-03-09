// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {

    private final NetworkTable table;

    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

        setVisionMode("april");
		setStream(2);
        setPipeline(0);
        setCameraMode(0);

    }

    public boolean isTarget() {
        return getValue("tv").getDouble(0.0) == 1;
    }

    public double getTx() {
        return getValue("tx").getDouble(0.0);    
    }

    public Boolean atTxTarget() {
		return this.getTx() < 1;
    }

    public double getTy() {
		return getValue("ty").getDouble(0.0);
	}

    public double getTa() {
		return getValue("ta").getDouble(0.0);
	}

    public double getTs() {
		return getValue("ts").getDouble(0.0);
	}

    public void setLedMode(int ledMode) {
		getValue("ledMode").setNumber(ledMode);
  	}
  
	public void setCameraMode(int cameraMode) {
		getValue("camMode").setNumber(cameraMode);
 	}

	public void setStream(int streamMode) {
		getValue("stream").setNumber(streamMode);
 	}
	
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

  	private NetworkTableEntry getValue(String key) {
		return table.getEntry(key);
	}

    public void setVisionMode(String visionMode) {
		setCameraMode(0);
        setPipeline(0);

		if (visionMode == "april") { //april
            setStream(1);
		} else {
            setStream(2);		
		}
    }
    double height = 57.5 - 10;
    

    public double getDistance() {
        if(!isTarget()) return -1;
        double angle = 21 + getTy();
        double distance = (height/Math.tan(Math.toRadians(angle)));
        return distance;
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("limelight distance", getDistance());
    }

}
    // This method will be called once per scheduler run
  

