package frc.robot.subsystems.nav;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Limelight {
    private static final double METERS_TO_INCHES = (100.0/2.54);
    
    private Transform3d[] aprilTagPositions = new Transform3d[17];

    private Transform3d  cameraInstallationPose;
    private Pose3d robotRelativeToAprilTag;

    public enum LimelightPipeline {
        Unknown(-1),
        AprilTag(0), 
        NoteTracker(1);

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

    private NetworkTable limelight;
    private Pose3d fieldPosition;
    public Limelight(Transform3d cameraPose) {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        fieldPosition = new Pose3d();

        // Layout Markings Locations: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
        // April Tag Relative to Elements: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/Apriltag_Images_and_User_Guide.pdf 
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
    
        this.cameraInstallationPose = cameraPose;
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
        int pipeline = (int)limelight.getEntry("getpipe").getInteger(-1);
        return LimelightPipeline.fromInt(pipeline);
    }
    
    public double getArea() {
        return limelight.getEntry("ta").getDouble(0);
    }
    Pose3d currentReading;
    private void updatePose(){
        double[] data;
        double area;
        boolean seesSomething;
        int aprilTagID;
        final double VALID_AREA = 0.25;
        Transform3d aprilTagTransmorm3d;

        data = limelight.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);


        area = getArea();
        seesSomething = seesSomething();
        aprilTagID = getAprilTagID();

        currentReading = new Pose3d(data[0], data[1], data[2], new Rotation3d(Math.toRadians(data[3]), Math.toRadians(data[4]), Math.toRadians(data[5])));
        /*  gain read
         * get validation info (area/valid/target id)
         * set valid flag to true - check each validation info - if false set value to false
         * if valid set field position based on reading transformed by april tag position n
         */
        if(seesSomething && area > VALID_AREA) {
            aprilTagTransmorm3d = aprilTagPositions[aprilTagID];
            robotRelativeToAprilTag = currentReading.transformBy(cameraInstallationPose);
            fieldPosition = robotRelativeToAprilTag.transformBy(aprilTagTransmorm3d);
        }
    }
    
    public Pose3d getRawData() {
        return currentReading;
    }
    public Pose3d getFieldPose() {
        return fieldPosition;
    }

    public boolean isPoseValid() {
        boolean valid = true;
        Rotation3d orientation=fieldPosition.getRotation();
        valid &= seesSomething();
        valid &= getPipeline() == LimelightPipeline.AprilTag;
        valid &= getArea() > .40;
        valid &= Math.abs(horizontalOffset()) <25;
        valid &= Math.abs(verticalOffset()) <20;
        valid &= orientation.getX() <= 15*Math.PI/180.0;
        valid &= orientation.getX() >= -15*Math.PI/180.0;
        valid &= orientation.getY() <= 15*Math.PI/180.0;
        valid &= orientation.getY() >= -15*Math.PI/180.0;
        valid &= fieldPosition.getX() >=-2.0;
        valid &= fieldPosition.getX() <=8.21055+1;
        valid &= fieldPosition.getY() >=-2.0;
        valid &= fieldPosition.getY() <=16.54175+1;
        return valid;
    }

    public boolean isNoteValid() {
        boolean valid = true;
        valid &= seesSomething();
        valid &= getPipeline() == LimelightPipeline.NoteTracker;
        valid &= getArea() > .40;
        valid &= Math.abs(horizontalOffset()) < 25;
        valid &= Math.abs(verticalOffset()) < 20;
        return valid;
    }

    public int getAprilTagID() {
        if(currentPipeline == LimelightPipeline.AprilTag) {
            return (int)limelight.getEntry("tid").getDouble(0);
        }
        return -100;
    }
    
    public boolean seesSomething() {
        double value = limelight.getEntry("tv").getDouble(0);
        return value > 0.5;
    }

    public double horizontalOffset() {
        return limelight.getEntry("tx").getDouble(0);
    }
    public double verticalOffset() {
        return limelight.getEntry("ty").getDouble(0);
    }
    
    public Pose3d getRawRedAllaince() {
        double[] data = limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        return new Pose3d(new Translation3d(data[0], data[1], data[2]).times(METERS_TO_INCHES), 
                                    new Rotation3d(Math.toRadians(data[3]), Math.toRadians(data[4]), Math.toRadians(data[5])));
    }

    public Pose3d getRawBlueAllaince() {
        double[] data = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose3d(new Translation3d(data[0], data[1], data[2]).times(METERS_TO_INCHES), 
                                    new Rotation3d(Math.toRadians(data[3]), Math.toRadians(data[4]), Math.toRadians(data[5])));
    }

    public Pose2d getLimelightPositionInField() { 
        Pose3d fieldPose;
        Optional<Alliance> myAlliance = DriverStation.getAlliance(); 
        if(myAlliance.isPresent() && myAlliance.get() == Alliance.Red){
            fieldPose = getRawRedAllaince();
        } else {
            fieldPose = getRawBlueAllaince();
        }
        fieldPose = fieldPose.plus(cameraInstallationPose.inverse());
        return fieldPose.toPose2d();
    }

    // Returns the time delay from when the last reading was valid.
    public double getLatency(){
        return (limelight.getEntry("tl").getDouble(0)
         + limelight.getEntry("cl").getDouble(0)
         + 150)/1000.0;
    }    
}