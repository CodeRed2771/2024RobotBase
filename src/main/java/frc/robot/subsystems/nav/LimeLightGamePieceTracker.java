package frc.robot.subsystems.nav;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.libs.LimelightHelpers;

public class LimeLightGamePieceTracker extends Limelight {
    
    private double area_threshold;
    private double last_ty;
    private double last_tx;
    private Translation2d last_note_estimate;

    public LimeLightGamePieceTracker(String network_table_key, Map<String,Double> calibration) {
        super(network_table_key,calibration);

        holdover_time = 0.75;
        LimelightHelpers.setLEDMode_ForceOff(network_table_key);
        LimelightHelpers.setPipelineIndex(network_table_key,LimelightPipeline.NoteTracker.toInt());

        area_threshold = calibration.getOrDefault("area threshold", 0.0);
    } 

    public void update() {
        if( isNetTableValid() ) {
            SmartDashboard.putBoolean("note tracking", true);
            last_tx = LimelightHelpers.getTX(net_table_name);
            last_ty = LimelightHelpers.getTY(net_table_name);
            last_note_estimate = estimateNotePosition(last_tx, last_ty);
            updateTimestamp(0.0);
        }
        else
            SmartDashboard.putBoolean("note tracking", false);

            SmartDashboard.putNumber("note angle", last_tx);
    }
    
    public boolean isNetTableValid() {
        boolean valid = true;
        valid = valid && getPipeline() == LimelightPipeline.NoteTracker;
        valid = valid && LimelightHelpers.getTV(net_table_name); // valid targets seen
        //valid = valid && LimelightHelpers.getTA(net_table_name) > area_threshold; // Area of seen target
        valid = valid && Math.abs(LimelightHelpers.getTX(net_table_name)) < 27.0; // Degrees off center
        valid = valid && Math.abs(LimelightHelpers.getTY(net_table_name)) < 25.0; // Degrees off center
        return valid;
    }

    public double getBearingToTargetDegrees() {
        return last_tx;
    }

    private double estimateNoteDistance(){
        // TODO: estimate distance by using trig from note size in view
        return 5.0;
    }

    private Translation2d estimateNotePosition(double delta_x, double delta_y){
        // Pick a point just in front of the camera
        Pose3d target = new Pose3d(estimateNoteDistance(),0,0,new Rotation3d());
        // point it at the offset angle of the visual target
        target = target.rotateBy(new Rotation3d(0,Math.toRadians(delta_y),Math.toRadians(delta_x)));

        // rotate it around to the robot body frame
        return target.transformBy(cameraInstallationToRobotPose).getTranslation().toTranslation2d();
    }

    public Translation2d getNoteEstimatedRobotPosition(){
        return last_note_estimate;
    }
}