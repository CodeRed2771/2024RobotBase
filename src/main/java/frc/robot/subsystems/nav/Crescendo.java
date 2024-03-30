// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.nav;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class Crescendo {

    // Layout Markings Locations: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
    // April Tag Relative to Elements: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/Apriltag_Images_and_User_Guide.pdf 
    /* April tags coordinates are XYZ of the center of the tag with the rotation being the X coordinate out the image to the field.
     * Coordinates are given in the BLUE team frame in NWE coordinates.  The ORIGIN is the bottom left corner of the image 
     * X: 0 at driver glass, positive looking across the field.
     * Y: 0 at right wall/side of driver stations, positive left.
     * Z: 0 at floor, positive up.
     */
    public static final Pose3d[] aprilTag = {
        new Pose3d( 593.68,   9.68, 53.38, new Rotation3d(0,0,Math.toRadians(120))),
        new Pose3d( 637.21,  34.79, 53.38, new Rotation3d(0,0,Math.toRadians(120))),
        new Pose3d( 652.21, 196.17, 57.13, new Rotation3d(0,0,Math.toRadians(180))),
        new Pose3d( 652.73, 218.42, 57.13, new Rotation3d(0,0,Math.toRadians(180))),
        new Pose3d( 578.77, 323.00, 53.38, new Rotation3d(0,0,Math.toRadians(270))),
        new Pose3d(   72.5, 323.00, 53.38, new Rotation3d(0,0,Math.toRadians(270))),
        new Pose3d(  -1.50, 218.42, 57.13, new Rotation3d(0,0,  Math.toRadians(0))),
        new Pose3d(  -1.50, 196.17, 57.13, new Rotation3d(0,0,  Math.toRadians(0))),
        new Pose3d(  14.02,  34.79, 53.38, new Rotation3d(0,0, Math.toRadians(60))),
        new Pose3d(  57.54,   9.68, 53.38, new Rotation3d(0,0, Math.toRadians(60))),
        new Pose3d( 468.69, 146.19, 52.00, new Rotation3d(0,0,Math.toRadians(300))),
        new Pose3d( 468.69, 177.10, 52.00, new Rotation3d(0,0, Math.toRadians(60))),
        new Pose3d( 441.74, 161.62, 52.00, new Rotation3d(0,0,Math.toRadians(180))),
        new Pose3d( 209.48, 161.62, 52.00, new Rotation3d(0,0,  Math.toRadians(0))),
        new Pose3d( 182.73, 177.10, 52.00, new Rotation3d(0,0,Math.toRadians(120))),
        new Pose3d( 182.73, 146.19, 52.00, new Rotation3d(0,0,Math.toRadians(240))),
    };

    /* Standard Nav algorithms for transformations is to find the Pose3d in the Origin frame of the other and then invert the result */
    public static final Transform3d BlueToRed = new Transform3d(new Translation3d(652.73,323.00,0.00),
                                                                new Rotation3d(0,0,Math.toRadians(180))).inverse();

    public enum PointsOfInterest {
        SAFE_ZONE(0),
        FEEDER_LEFT(1),
        FEEDER_CENTER(2),
        FEEDER_RIGHT(3),
        AMP(4),
        SUBWOOFER_LEFT(5),
        SUBWOOFER_CENTER(6),
        SUBWOOFER_RIGHT(7),
        SPEAKER(8),
        TRAP_CENTER(9),
        TRAP_FEEDER_SIDE(10),
        TRAP_AMP_SIDE(11);

        private int index;
        private PointsOfInterest(int val){
            index = val;
        }
        public int toInt() {return index;}
    }

    private static final Pose3d[] Blue_POIs = {
        new Pose3d(new Translation3d(121,161.5,0), new Rotation3d(0,0,180)), // Safe zone
        new Pose3d(new Translation3d(637.21,34.79,0), new Rotation3d(0,0,120)), // Feeder Left
        new Pose3d(new Translation3d(615.445,22.235,0), new Rotation3d(0,0,120)), // Feeder Center
        new Pose3d(new Translation3d(593.68,9.68,0), new Rotation3d(0,0,120)), // Feeder Right
        new Pose3d(new Translation3d(72.5,323,26), new Rotation3d(0,0,270)), // AMP
        new Pose3d(new Translation3d(32,151,0), new Rotation3d(0,0,60)), // Subwoofer Left
        new Pose3d(new Translation3d(47.5,104.58,0), new Rotation3d(0,0,0)), // Subwoofer Center
        new Pose3d(new Translation3d(32,59,0), new Rotation3d(0,0,-60)), // Subwoofer Right
        new Pose3d(new Translation3d(5,104.58,80), new Rotation3d(0,0,180)), // Speaker
        new Pose3d(new Translation3d(209.48,161.62,56.5), new Rotation3d(0,0,0)), // Trap Center
        new Pose3d(new Translation3d(182.73,177.1,56.5), new Rotation3d(0,0,120)), // Trap AMP
        new Pose3d(new Translation3d(182.73,146.19,56.5), new Rotation3d(0,0,240)), // Trap Feeder
    };

    private static final Pose3d[] Red_POIs = {
        new Pose3d(new Translation3d(121,161.5,0), new Rotation3d(0,0,0)), // Safe zone
        new Pose3d(new Translation3d(637.21,288.21,0), new Rotation3d(0,0,300)), // Feeder Left
        new Pose3d(new Translation3d(615.445,300.765,0), new Rotation3d(0,0,300)), // Feeder Center
        new Pose3d(new Translation3d(593.68,313.32,0), new Rotation3d(0,0,300)), // Feeder Right
        new Pose3d(new Translation3d(72.5,0,26), new Rotation3d(0,0,90)), // AMP
        new Pose3d(new Translation3d(32,264,0), new Rotation3d(0,0,60)), // Subwoofer Left
        new Pose3d(new Translation3d(47.5,218.42,0), new Rotation3d(0,0,0)), // Subwoofer Center
        new Pose3d(new Translation3d(32,172,0), new Rotation3d(0,0,-60)), // Subwoofer Right
        new Pose3d(new Translation3d(5,218.42,80), new Rotation3d(0,0,0)), // Speaker
        new Pose3d(new Translation3d(209.48,161.38,56.5), new Rotation3d(0,0,180)), // Trap Center
        new Pose3d(new Translation3d(182.73,145.9,56.5), new Rotation3d(0,0,300)), // Trap AMP
        new Pose3d(new Translation3d(182.73,176.81,56.5), new Rotation3d(0,0,60)), // Trap Feeder
    };

    public static Pose3d getBluePOI(PointsOfInterest poi){return Blue_POIs[poi.toInt()];}
    public static Pose3d getRedPOI(PointsOfInterest poi){return Blue_POIs[poi.toInt()];}

    private static boolean bUseRed = false;
    public static void useBlueTargets(){bUseRed = false;}
    public static void useRedTargets(){bUseRed = true;}
    public static Pose3d getPose3d(PointsOfInterest poi){
        Pose3d result;
        if(bUseRed)
            result = getRedPOI(poi);
        else
            result = getBluePOI(poi);
        return result;
    }

    public static boolean isValidPosition(Translation2d position){
        boolean valid = true;
        valid = valid && position.getX() >= -10.0;
        valid = valid && position.getX() <= Crescendo.BlueToRed.getX() + 10.0;
        valid = valid && position.getY() >= -10.0;
        valid = valid && position.getY() <= Crescendo.BlueToRed.getY() + 10.0;
        return valid;
    }
}
