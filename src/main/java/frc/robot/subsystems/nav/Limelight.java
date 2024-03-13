package frc.robot.subsystems.nav;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Limelight {
    protected Transform3d cameraInstallationToRobotPose;
    private double lastUpdate_timestamp = 0;
    private double latency_cal = 0.0;
    private double total_latency;
    private double holdover_time;

    public enum LimelightPipeline {
        Unknown(-1),
        AprilTag(0), 
        NoteTracker(1);

        private final int value;
        private LimelightPipeline(int value) {
            this.value = value;
        }
        public int toInt(){
            return value;
        }
        public static LimelightPipeline fromInt(int val) {
            for(LimelightPipeline e:LimelightPipeline.values()) {
                if(e.value == val) {
                    return e;
                }
            }
            throw new IllegalArgumentException("Invalid Pipeline ID Integer");
        }
    }
    protected LimelightPipeline currentPipeline = LimelightPipeline.Unknown;
    
    public enum LimelightOn {
        BasedOnPipeline(0),
        Off(1),
        Blink(2),
        On(3);

        public final int value;
        private LimelightOn(int value) {
            this.value = value;
        }
    }

    protected String net_table_name;
    protected NetworkTable net_table;
    public Limelight(String network_table_key, Map<String,Double> calibration) {
        net_table_name = network_table_key;
        net_table = NetworkTableInstance.getDefault().getTable(network_table_key);

        holdover_time = calibration.getOrDefault("note holdover", 0.25);

       this.cameraInstallationToRobotPose = new Transform3d(new Translation3d(calibration.getOrDefault(network_table_key + " X",0.0),
                                                                       calibration.getOrDefault(network_table_key + " Y",0.0),
                                                                       calibration.getOrDefault(network_table_key + " Z",0.0)),
                                                     new Rotation3d(Math.toRadians(calibration.getOrDefault(network_table_key + " roll",0.0)),
                                                                    Math.toRadians(calibration.getOrDefault(network_table_key + " pitch",0.0)),
                                                                    Math.toRadians(calibration.getOrDefault(network_table_key + " yaw",0.0)))).inverse();
        latency_cal = calibration.getOrDefault(network_table_key + "latency", 0.02);
    } 

    public void setLED(LimelightOn value) {
        net_table.getEntry("ledMode").setNumber(value.value);
    }

    public void setPipeline(LimelightPipeline pipeline) {
		NetworkTableEntry pipelineEntry = net_table.getEntry("pipeline");
    	pipelineEntry.setNumber(pipeline.toInt());
    }

    public LimelightPipeline getPipeline() {
        int pipeline = (int)net_table.getEntry("getpipe").getInteger(-1);
        return LimelightPipeline.fromInt(pipeline);
    }
    
    // Returns the time delay from when the last reading was valid.
    public double getTimeOfMeasurement(){
        return lastUpdate_timestamp;
    }
    
    public double getLatency(){
        return total_latency;
    }

    /* Update the timestamp of when the last measurement was valid in seconds */
    protected void updateTimestamp(double latency){
        total_latency = latency + latency_cal;
        lastUpdate_timestamp = Timer.getFPGATimestamp() - total_latency;
    } 
    
    public boolean isTracking(){
        return Timer.getFPGATimestamp() - getTimeOfMeasurement() < holdover_time;
    }
}