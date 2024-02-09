package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.nav.NavSubsystem.fieldPositions;

public class Limelight{
    private final double INCHES_TO_METERS = 39.3701;
    
    private Transform3d[] aprilTagPositions = new Transform3d[17];

    private Pose3d currentPose;
    
    public static enum Target {
        SPEAKER,
        AMP, 
    }

    public static enum LimelightPipeline {
        Unknown(-1),
        AprilTag(1), 
        NoteTracker(2);

        public final int value;
        private LimelightPipeline(int value) {
            this.value = value;
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
    private LimelightPipeline currentPipeline = LimelightPipeline.Unknown;
    public static enum LimelightOn {
        BasedOnPipeline(0),
        Off(1),
        Blink(2),
        On(3);

        public final int value;
        private LimelightOn(int value) {
            this.value = value;
        }
    }

    private NetworkTable limelight;
    private Pose3d fieldPosition;
    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");

        aprilTagPositions[1] = new Transform3d(593.68,9.68,53.38, new Rotation3d(0,0,120));
        aprilTagPositions[2] = new Transform3d(637.21,34.79,53.38,new Rotation3d(0,0,120));
        aprilTagPositions[3] = new Transform3d(652.21,196.17,57.13, new Rotation3d(0,0,180));
        aprilTagPositions[4] = new Transform3d(652.73,218.42,57.13,new Rotation3d(0,0,180));
        aprilTagPositions[5] = new Transform3d(578.77,323.00,53.38,new Rotation3d(0,0,270));
        aprilTagPositions[6] = new Transform3d(72.5,323.00,53.38,new Rotation3d(0,0,270));
        aprilTagPositions[7] = new Transform3d(-1.50,218.42,57.13,new Rotation3d(0,0,0));
        aprilTagPositions[8] = new Transform3d(-1.50,196.17,57.13,new Rotation3d(0,0,0));
        aprilTagPositions[9] = new Transform3d(14.02,34.79,53.38,new Rotation3d(0,0,60));
        aprilTagPositions[10] = new Transform3d(57.54,9.68,53.38,new Rotation3d(0,0,60));
        aprilTagPositions[11] = new Transform3d(468.69,146.19,52.00,new Rotation3d(0,0,300));
        aprilTagPositions[12] = new Transform3d(468.69,177.10,52.00,new Rotation3d(0,0,60));
        aprilTagPositions[13] = new Transform3d(441.74,161.62,52.00,new Rotation3d(0,0,180));
        aprilTagPositions[14] = new Transform3d(209.48,161.62,52.00,new Rotation3d(0,0,0));
        aprilTagPositions[15] = new Transform3d(182.73,177.10,52.00,new Rotation3d(0,0,120));
        aprilTagPositions[16] = new Transform3d(182.73,146.19,52.00,new Rotation3d(0,0,240));
    } 

    public void setLED(LimelightOn value) {
        limelight.getEntry("ledMode").setNumber(value.value);
    }

    public void setPipeline(LimelightPipeline pipeline) {
		NetworkTableEntry pipelineEntry = limelight.getEntry("pipeline");
    	pipelineEntry.setNumber(pipeline.value);
    }
    public void pollLimelight() {
        currentPipeline = getPipeline();
        if(currentPipeline == LimelightPipeline.AprilTag) {
            updatePose();
        }
    }

    public LimelightPipeline getPipeline() {
        int pipeline = (int)limelight.getEntry("getpip").getInteger(0);
        return LimelightPipeline.fromInt(pipeline);
    }
    
    public double getArea() {
        return limelight.getEntry("ta").getDouble(0);
    }

    private void updatePose(){
        double[] data;
        double area;
        boolean seesSomething;
        int aprilTagID;
        final double VALID_AREA = 0.25;
        Transform3d aprilTagTransmorm3d;

        data = limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        area = getArea();
        seesSomething = seesSomething();
        aprilTagID = getAprilTagID();

        Pose3d currentReading = new Pose3d(data[0], data[1], data[2], new Rotation3d(data[3], data[4], data[5]));
        /*  gain read
         * get validation info (area/valid/target id)
         * set valid flag to true - check each validation info - if false set value to false
         * if valid set field position based on reading transformed by april tag position n
         */
        if(seesSomething && area > VALID_AREA) {
            aprilTagTransmorm3d = aprilTagPositions[aprilTagID];
            fieldPosition = currentReading.transformBy(aprilTagTransmorm3d);
        }
    }
    

    public Pose3d getFieldPose() {
        return fieldPosition;
    }

    public int getAprilTagID() {
        if(currentPipeline == LimelightPipeline.AprilTag) {
            return (int)limelight.getEntry("tid").getDouble(0);
        }
        return -100;
    }
    
    public boolean seesSomething() {
        double value = limelight.getEntry("tv").getDouble(0);
        if(value == 1) {
            return true;
        }
        return false;
    }

    public double horizontalOffset() {
        return limelight.getEntry("tx").getDouble(0);
    }
    public double verticalOffset() {
        return limelight.getEntry("ty").getDouble(0);
    }
    
    public Transform3d getOffsetToTarget(Target target, fieldPositions targetPositions) {
        Transform3d pose = new Transform3d();
        switch (target) {
            case AMP:
                pose = new Transform3d(getFieldPose(), targetPositions.ampPose);
                break;
            case SPEAKER:
                pose = new Transform3d(getFieldPose(), targetPositions.supwofferPose);
                break;
        }
        return pose;
    }
    
}