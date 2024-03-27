package frc.robot.subsystems.nav;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

    protected Pose3d last_fieldPoseEstimate;
    protected Matrix<N3,N1> last_fieldPoseStandardDevs;
    protected boolean last_fieldPoseEstimate_Valid;

    public LimeLightPoseEstimator(String network_table_key, Map<String,Double> calibration) {
        super(network_table_key,calibration);

        LimelightHelpers.setLEDMode_ForceOff(network_table_key);
        LimelightHelpers.setPipelineIndex(network_table_key,LimelightPipeline.AprilTag.toInt());

        area_threshold = calibration.getOrDefault("area threshold", 0.2);
        last_fieldPoseEstimate = new Pose3d();
        last_fieldPoseStandardDevs = VecBuilder.fill(100000.0,100000.0,100000.0);
    } 

    public void update() {
        double[] data;
        Translation3d new_pos;
        Rotation3d new_rotation;
        Pose3d new_pose;

        currentPipeline = getPipeline();
        if(currentPipeline == LimelightPipeline.AprilTag) {
            data = getNTPoseData();
            new_pos = getTranslationFromNTData(data);
            new_rotation = getRotationFromNTData(data);
            new_pose = new Pose3d(new_pos,new_rotation).transformBy(cameraInstallationToRobotPose);

            SmartDashboard.putNumber("Camera X", new_pose.getX());
            SmartDashboard.putNumber("Camera Y", new_pose.getY());
            SmartDashboard.putNumber("Camera Z", new_pose.getZ());
            SmartDashboard.putNumber("Camera R", new_pose.getRotation().getX());
            SmartDashboard.putNumber("Camera P", new_pose.getRotation().getY());
            SmartDashboard.putNumber("Camera Y", new_pose.getRotation().getZ());

            if(isValidMetadata(data) && isValidPosition(new_pose.getTranslation()) && isValidRotation(new_pose.getRotation()))
            {
                last_tid = getNTAprilTagID();
                last_fieldPoseEstimate = new_pose;
                last_fieldPoseEstimate_Valid = true;
                last_fieldPoseStandardDevs = estimateStandardDeviations(data);
                updateTimestamp(getLatencyFromNTData(data));
            }
        }
    }

    private double getLatencyFromNTData(double[] data){
        return data[6]/1000.0;
    }

    private double getTargetCountFromNTData(double[] data){
        return data[7];
    }

    private double getTagSeparationFromNTData(double[] data){
        return data[8];
    }

    private double getTargetDistanceFromNTData(double[] data){
        return data[9];
    }

    private double getTargetAreaFromNTData(double[] data){
        return data[10];
    }

    private Translation3d getTranslationFromNTData(double[] data)
    {
        return new Translation3d(data[0], data[1], data[2]).times(METERS_TO_INCHES);
    }

    private Rotation3d getRotationFromNTData(double[] data)
    {
        return new Rotation3d(Math.toRadians(data[3]), Math.toRadians(data[4]), Math.toRadians(data[5]));
    }

    private Matrix<N3, N1> estimateStandardDeviations(double[] data){
        double xyStds = 1000000.0;
        double degStds = 10000.0;

        if(getTargetCountFromNTData(data) >= 2)
        {
            xyStds = 0.5*METERS_TO_INCHES;
            degStds = 6;
        }
        else if(getTargetAreaFromNTData(data) >= 0.8)
        {
            xyStds = 1.0*METERS_TO_INCHES;
            degStds = 12;
        }
        else if(getTargetAreaFromNTData(data) >= area_threshold)
        {
            xyStds = 2.0*METERS_TO_INCHES;
            degStds = 30;
        }

        return VecBuilder.fill(xyStds,xyStds,Math.toRadians(degStds));
    }

    public void checkUpdatePoseEstimator(SwerveDrivePoseEstimator poseEstimator){

        update();
        SmartDashboard.putBoolean("Camera Pose", last_fieldPoseEstimate_Valid);
        if(last_fieldPoseEstimate_Valid)
            poseEstimator.addVisionMeasurement(last_fieldPoseEstimate.toPose2d(),getTimeOfMeasurement(),last_fieldPoseStandardDevs);

    }

    public int getNTAprilTagID() {
        if(currentPipeline == LimelightPipeline.AprilTag) {
            return (int)net_table.getEntry("tid").getDouble(0);
        }
        return 0;
    }

    public Pose3d getEstimatedPosition() {
        return last_fieldPoseEstimate;
    }
    
    public boolean isValid()
    {
        return last_fieldPoseEstimate_Valid;
    }
    
    private boolean isValidPosition(Translation3d position){
        boolean valid =  Crescendo.isValidPosition(position.toTranslation2d()) &&
               MathUtil.isNear(0,position.getZ(), 12*4.0); // near the ground

        SmartDashboard.putBoolean("C Pos Valid", valid);
        return valid;
    }

    private boolean isValidRotation(Rotation3d rotation){
        boolean valid = true;
        valid = valid && rotation.getX() <= 15*Math.PI/180.0;
        valid = valid && rotation.getX() >= -15*Math.PI/180.0;
        valid = valid && rotation.getY() <= 15*Math.PI/180.0;
        valid = valid && rotation.getY() >= -15*Math.PI/180.0;
        SmartDashboard.putBoolean("C Rot Valid", valid);
        return valid;
    }

    private boolean isValidMetadata(double[] data){
        boolean valid = true;
        valid = valid && getLatencyFromNTData(data) <= 0.5; // latency
        valid = valid && getTargetCountFromNTData(data) >= 1; // Tag count
        valid = valid && getTargetDistanceFromNTData(data) <= 10; // Average Tag distance
        valid = valid && getTargetAreaFromNTData(data) >= area_threshold;
        SmartDashboard.putBoolean("C Meta Valid", valid);
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
        return last_fieldPoseEstimate.toPose2d();
    }
}