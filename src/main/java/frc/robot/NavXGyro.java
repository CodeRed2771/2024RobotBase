package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class NavXGyro {
    private AHRS mGyro;
    private double pitchAdjust = 0;

    /* Use a singleton design pattern to assist in migrating from ubiquitous static class operations */
    private static class NavXGyroSingleton {
        private static final NavXGyro instance = new NavXGyro(SPI.Port.kMXP);
    }

    private NavXGyro(SPI.Port port){
        mGyro = new AHRS(port);
    }

    public static NavXGyro getInstance(){
        return NavXGyroSingleton.instance;
    }

    public void init() {
        pitchAdjust = mGyro.getPitch(); // reads the pitch while at rest on flat surface
                                        // will be used to offset values to return a relative pitch
    }

    public AHRS getGyro() {
        return mGyro;
    }

    /***
     * 
     * @return gyro angle This angle can include multiple revolutions so if the
     *         robot has rotated 3 1/2 times, it will return a value of 1260. This
     *         is useful for turning but not as useful for figuring out which
     *         direction you're facing. Use getRelativeAngle for that.
     */
    public double getAngle() {
        return mGyro.getAngle();
    }


    static class Position {
        double x;
        double y;
        double z;
        public Position(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }
    public static Position position = new Position(0, 0, 0);
    private static final double CYCLE_TIME = 0.02;
    /***
     * ƒ
     * 
     * @return gyro angle 0 to 360
     */
    public double getRelativeAngle() {
        if (getAngle() < 0) {
            return 360 + (getAngle() % 360);
        } else {
            return getAngle() % 360;
        }
    }

    public double velocityX() {
        return mGyro.getVelocityX();
    }

    public double velocityY() {
        return mGyro.getVelocityY();
    }

    public double velocityZ() {
        return mGyro.getVelocityZ();
    }

    public double pitch_raw() {
        return mGyro.getPitch();
    }

    public double pitch() {
        return mGyro.getPitch()-pitchAdjust;
    }

    public double roll() {
        return mGyro.getRoll();
    }

    public double yaw() {
        return mGyro.getYaw();
    }
    /***
     * 
     * @param desiredPosition - desired 0 to 360 position
     * @return amount to turn to get there the quickest way
     */
    public double getClosestTurn(double desiredPosition) {
        double distance = 0;
        double currentPosition = getRelativeAngle();

        if (currentPosition - desiredPosition >= 0)
            if (currentPosition - desiredPosition > 180)
                distance = (360 - currentPosition) + desiredPosition;
            else
                distance = desiredPosition - currentPosition;
        else if (desiredPosition - currentPosition > 180)
            distance = -((360 - desiredPosition) + currentPosition);
        else
            distance = desiredPosition - currentPosition;

        return distance;
    }

    public void reset() {
        mGyro.reset();
        pitchAdjust = mGyro.getPitch(); // reads the pitch while at rest on flat surface
        // will be used to offset values to return a relative pitch

    }

    public double getGyroAngleInRad() {
        double adjustedAngle = -Math.floorMod((long) mGyro.getAngle(), 360);
        if (adjustedAngle > 180)
            adjustedAngle = -(360 - adjustedAngle);

        return adjustedAngle * (Math.PI / 180d);
    }
    
    public double pidGet() {
        return mGyro.getAngle();
    }
    
    public void position() {
        position.x += mGyro.getVelocityX() * CYCLE_TIME;
        position.y += mGyro.getVelocityY() * CYCLE_TIME;
        position.x = mGyro.getVelocityZ() * CYCLE_TIME;
    }
    public Position getPosition() {
        return position;
    }
}
