package frc.robot.subsystems.nav;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SPI;

public class NavXGyro {
    private AHRS mGyro;
    private double pitchAdjust = 0;

    public NavXGyro(SPI.Port port){
        mGyro = new AHRS(port);
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

    /***
     * ƒ
     * 
     * @return gyro angle 0 to 360
     */
    public double getRelativeAngle() {
        return MathUtil.inputModulus(getAngle(), 0.0, 360.0);
    }

    public Translation3d getVelocity3d(){
        return new Translation3d(velocityX(), 
                                 velocityY(),
                                 velocityZ());
    }

    public double velocityX() {
        return mGyro.getVelocityX()*100/2.54;
    }

    public double velocityY() {
        return mGyro.getVelocityY()*100/2.54;
    }

    public double velocityZ() {
        return mGyro.getVelocityZ()*100/2.54;
    }

    public double pitch_raw() {
        return mGyro.getPitch();
    }

    public Rotation3d getRotation(){
        return new Rotation3d(pitch(),roll(),yaw());
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
    public double yaw_rate(){
        return mGyro.getRate();
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

    public void zeroYaw()
    {
        mGyro.zeroYaw();
    }

    public double getAngleAdjustment(){
        return mGyro.getAngleAdjustment();
    }

    public void setAngleAdjustment(double angle){
        mGyro.setAngleAdjustment(angle);
    }

    public void reset() {
        mGyro.reset();
        pitchAdjust = mGyro.getPitch(); // reads the pitch while at rest on flat surface
        // will be used to offset values to return a relative pitch
    }

    public double getGyroAngleInRad() {
        return MathUtil.angleModulus(Math.toRadians(getAngle()));
    }
}
