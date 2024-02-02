package frc.robot.subsystems.nav;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    public static enum LimelightOn {
        On,
        Off,
        Blink
    }
    public class limelightData {
        private double[] data;
        public limelightData(double [] data) {
            this.data = data;
        }
        public double getX(){
            return data[0];
        }
        public double getY() {
            return data[1];
        }
        public double getZ() {
            return data[2];
        }
        public double getRoll() {
            return data[3];
        }
        public double getPitch() {
            return data[4];
        }
        public double getYaw() {
            return data[5];
        }
    }

    private NetworkTable limelight;
    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        
    } 

    public void setLED(LimelightOn value) {
        if (value == LimelightOn.Off) {
            limelight.getEntry("ledMode").setNumber(1);
        } else if (value == LimelightOn.Blink) {
            limelight.getEntry("ledMode").setNumber(2);
        } else {
            limelight.getEntry("ledMode").setNumber(3);
        }
    }
    public void setPipeline(int pipeline) {
		NetworkTableEntry pipelineEntry = limelight.getEntry("pipeline");
    	pipelineEntry.setNumber(pipeline);
    }

    public void setAprilTagPipeline() {
        setPipeline(0);
    }
    public double getPipeline() {
        double pipeline = limelight.getEntry("getpip").getDouble(0);
        return pipeline;
    }
    
    public limelightData getTargetSpace(){
        return new limelightData(limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6])); 
    }
    
    
}
