package frc.robot.subsystems.nav;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.libs.LimelightHelpers;

public class LimeLightPoseEstimator extends Limelight {
    private static final double METERS_TO_INCHES = (100.0/2.54);
    
    private int last_tid;
    private double area_threshold;

    private boolean bUseRed = false;
    public void useBlueTargets(){bUseRed = false;}
    public void useRedTargets(){bUseRed = true;}

    protected Pose3d fieldPosition;
    public LimeLightPoseEstimator(String network_table_key, Map<String,Double> calibration) {
        super(network_table_key,calibration);

        LimelightHelpers.setLEDMode_ForceOff(network_table_key);
        LimelightHelpers.setPipelineIndex(network_table_key,LimelightPipeline.AprilTag.toInt());

        area_threshold = calibration.getOrDefault("area threshold", 0.5);
        fieldPosition = new Pose3d();
    } 

    public void update() {
        double[] data;
        Translation3d new_pos;
        Rotation3d new_rotation;

        currentPipeline = getPipeline();
        if(currentPipeline == LimelightPipeline.AprilTag) {
            data = getNTPoseData();
            new_pos = new Translation3d(data[0], data[1], data[2]).times(METERS_TO_INCHES);
            new_rotation = new Rotation3d(Math.toRadians(data[3]), Math.toRadians(data[4]), Math.toRadians(data[5]));
            if(isValidMetadata(data) && isValidPosition(new_pos) && isValidRotation(new_rotation))
            {
                last_tid = getNTAprilTagID();
                fieldPosition = new Pose3d(new_pos,new_rotation).transformBy(cameraInstallationToRobotPose);
                updateTimestamp(data[6]/1000.0);
            }
        }
    }

    public int getNTAprilTagID() {
        if(currentPipeline == LimelightPipeline.AprilTag) {
            return (int)net_table.getEntry("tid").getDouble(0);
        }
        return 0;
    }

    public Pose3d getEstimatedPosition() {
        return fieldPosition;
    }

    private boolean isValidPosition(Translation3d position){
        return Crescendo.isValidPosition(position.toTranslation2d()) &&
               MathUtil.isNear(0,position.getZ(), 12*4.0); // near the ground

    }

    private boolean isValidRotation(Rotation3d rotation){
        boolean valid = true;
        valid = valid && rotation.getX() <= 15*Math.PI/180.0;
        valid = valid && rotation.getX() >= -15*Math.PI/180.0;
        valid = valid && rotation.getY() <= 15*Math.PI/180.0;
        valid = valid && rotation.getY() >= -15*Math.PI/180.0;
        return valid;
    }

    private boolean isValidMetadata(double[] data){
        boolean valid = true;
        valid = valid && data[7] <= 0.5; // latency
        valid = valid && data[8] >= 1; // Tag count
        valid = valid && data[9] <= 5; // Average Tag distance
        valid = valid && data[10] >= area_threshold; // average Tag Area
        return valid;

    }

    public int getTagID() {
        return last_tid;
    }

    private double[] getNTPoseData(){
        if(bUseRed)
            return LimelightHelpers.getBotPose_wpiRed(net_table_name);
        else
            return LimelightHelpers.getBotPose_wpiBlue(net_table_name);
    }

    public Pose2d getLimelightPositionInField() { 
        return fieldPosition.toPose2d();
    }
}