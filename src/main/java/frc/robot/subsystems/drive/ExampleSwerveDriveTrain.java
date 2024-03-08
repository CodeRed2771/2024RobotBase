package frc.robot.subsystems.drive;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Calibration;

public class ExampleSwerveDriveTrain extends DriveSubsystem {

  private static final double kMaxSpeed = 200.0; // normalized full Speed Inches/sec

  private NewSwerveModuleVortex m_frontLeft; // Front Left
  private NewSwerveModuleVortex m_backRight; // Back Right
  private NewSwerveModuleVortex m_backLeft; // Back Left
  private NewSwerveModuleVortex m_frontRight; // Front Right

  // Robot Center is 0,0 anchor point for coordinates (where NavX is)
  // +X = out intake
  // +Y = out right side of robot
  private final double wheel_position_offset = (15.0 - 3.25);
  private final double wheel_position_offset_radius =  (Math.sqrt(2)*wheel_position_offset);
  // Max Angle rate in Rad = (Max linear speed / Circumference (2PI *R)) for rotations * 2PI (for Radians)
  // Max Angle Rate = Max speed / radius
  private final double kMaxAngleRate = kMaxSpeed / wheel_position_offset_radius;
  private final Translation2d m_frontLeftLocation = new Translation2d(wheel_position_offset, wheel_position_offset);
  private final Translation2d m_frontRightLocation = new Translation2d(wheel_position_offset, -wheel_position_offset);
  private final Translation2d m_backLeftLocation = new Translation2d(-wheel_position_offset, wheel_position_offset);
  private final Translation2d m_backRightLocation = new Translation2d(-wheel_position_offset, -wheel_position_offset);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
      m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public ExampleSwerveDriveTrain(Map<String, Integer> wiring, Map<String,Double> calibration) {
    super();


    // Circumference in cm to Radius in inches
    m_frontLeft = new NewSwerveModuleVortex(wiring, calibration, "A"); // Front right
    m_backRight = new NewSwerveModuleVortex(wiring, calibration,"B"); // Back left
    m_backLeft = new NewSwerveModuleVortex(wiring, calibration, "C"); // Back right
    m_frontRight = new NewSwerveModuleVortex(wiring, calibration, "D"); // Front left

    Calibration.loadSwerveCalibration();
    setTurnOffsets( Calibration.getTurnZeroPos('A'), 
                    Calibration.getTurnZeroPos('B'), 
                    Calibration.getTurnZeroPos('C'),
                    Calibration.getTurnZeroPos('D'));

    this.addChild(m_frontLeft.getName(), m_frontLeft);
    this.addChild(m_backRight.getName(), m_backRight);
    this.addChild(m_backLeft.getName(), m_backLeft);
    this.addChild(m_frontRight.getName(), m_frontRight);

    SmartDashboard.putNumber("TURN P", Calibration.getTurnP());
    SmartDashboard.putNumber("TURN I", Calibration.getTurnI());
    SmartDashboard.putNumber("TURN D", Calibration.getTurnD());

    Calibration.initializeSmartDashboard();
    SmartDashboard.putBoolean("Use Raw Encoder", false);
    SmartDashboard.putBoolean("Use Offset Encoder", false);

  }

  @Override
  public void doArm() {
    m_frontLeft.arm();
    m_backRight.arm();
    m_backLeft.arm();
    m_frontRight.arm();
  }

  @Override
  public void doDisarm() {
    m_frontLeft.disarm();
    m_backRight.disarm();
    m_backLeft.disarm();
    m_frontRight.disarm();
  }

  public SwerveModulePosition[] getOdomotry(){
    return new SwerveModulePosition[] { // Maintain order from m_odometry creation
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()};
  }
  
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * Method to drive the robot with robot framed commands.
   *
   * @param xSpeed Speed of the robot in the x direction (forward) (-1,1).
   * @param ySpeed Speed of the robot in the y direction (sideways)(-1,1).
   * @param rot    Angular rate of the robot.
   */
  public void driveSpeedControl(double xSpeed, double ySpeed, double rot, double periodSeconds) {
    var swerveModuleStates = m_kinematics
        .toSwerveModuleStates(ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed*kMaxSpeed, ySpeed*kMaxSpeed, rot*kMaxAngleRate), periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    // Maintain order from m_kinematics creation
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveFixedSpeedOrientation(double xSpeed, double ySpeed)
  {
    Translation2d Speed = new Translation2d(xSpeed,ySpeed);
    SwerveModuleState desiredState = new SwerveModuleState(Speed.getNorm(), Speed.getAngle());

    m_frontLeft.setDesiredState(desiredState);
    m_frontRight.setDesiredState(desiredState);
    m_backLeft.setDesiredState(desiredState);
    m_backRight.setDesiredState(desiredState);
  }

  public void driveFixedPositionOffsetInches(double xInches, double yInches){
    Translation2d target = new Translation2d(xInches, yInches);
    SwerveModulePosition targetOffset = new SwerveModulePosition(target.getNorm(), target.getAngle());

    m_frontLeft.commandSwervePositionOffset(targetOffset);
    m_frontRight.commandSwervePositionOffset(targetOffset);
    m_backLeft.commandSwervePositionOffset(targetOffset);
    m_backRight.commandSwervePositionOffset(targetOffset);
  }

  public boolean atFixedPosition(double allowedError){
    boolean result = m_frontLeft.atSwervePosition(allowedError);
    result = result && m_frontRight.atSwervePosition(allowedError);
    result = result && m_backLeft.atSwervePosition(allowedError);
    result = result && m_backRight.atSwervePosition(allowedError);
    return result;
  }

  public void driveParkControl(){
    m_frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    m_backLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    m_backRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
  }

  public void driveFixedRotatePosition(double degrees){
    double distance = (degrees/180.0 * Math.PI) * wheel_position_offset_radius;

    m_frontLeft.commandSwervePositionOffset(new SwerveModulePosition(distance, Rotation2d.fromDegrees(-45.0)));
    m_frontRight.commandSwervePositionOffset(new SwerveModulePosition(-distance, Rotation2d.fromDegrees(45.0)));
    m_backLeft.commandSwervePositionOffset(new SwerveModulePosition(distance, Rotation2d.fromDegrees(45.0)));
    m_backRight.commandSwervePositionOffset(new SwerveModulePosition(-distance, Rotation2d.fromDegrees(-45.0)));
  }

  @Override
  public void periodic() {

    if (isDisarmed()) {
      if (Calibration.shouldCalibrateSwerve()) {
        double[] pos = getAllAbsoluteTurnOrientations();
        Calibration.saveSwerveCalibration(pos[0], pos[1], pos[2], pos[3]);
        ApplyCalibration();
      }

      // see if we want to reset the calibration to whatever is in the program
      // basically setting "Delete Swerve Calibration" to true will trigger
      // this, which deletes the calibration file.
      Calibration.checkIfShouldDeleteCalibration();
    }

    boolean doEncoderAction = SmartDashboard.getBoolean("Use Raw Encoder", false);
    if (doEncoderAction) {
      setTurnOffsets( 0,0,0,0);
      SmartDashboard.putBoolean("Use Raw Encoder", false);
    }
    doEncoderAction = SmartDashboard.getBoolean("Use Offset Encoder", false);
    if (doEncoderAction){
      ApplyCalibration();
      SmartDashboard.putBoolean("Use Offset Encoder", false);
    }
  }

  public void ApplyCalibration(){
    setTurnOffsets( Calibration.getTurnZeroPos('A'), 
                    Calibration.getTurnZeroPos('B'), 
                    Calibration.getTurnZeroPos('C'),
                    Calibration.getTurnZeroPos('D'));
  }

  public double[] getAllAbsoluteTurnOrientations() {
    return new double[] {m_frontLeft.getAbsOrentation().getRotations(), m_backRight.getAbsOrentation().getRotations(),
        m_backLeft.getAbsOrentation().getRotations(), m_frontRight.getAbsOrentation().getRotations()};
  }

  public void setTurnOffsets(double FL, double BR, double BL, double FR) {
    m_frontLeft.setTurnOffset(FL);
    m_backRight.setTurnOffset(BR);
    m_backLeft.setTurnOffset(BL);
    m_frontRight.setTurnOffset(FR);
    resetEncoders();
  }

  private void resetEncoders(){
    m_frontLeft.resetEncoders();
    m_backRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
  }

  @Override
  public void driveInches(double inches, double speedFactor){
      driveInches(inches, speedFactor, 0);
  }


  @Override
  public void driveInches(double inches, double speedFactor, double turnAngle){

    Translation2d target = new Translation2d(inches, Rotation2d.fromDegrees(turnAngle));
    driveFixedPositionOffsetInches(target.getX(), target.getY());
  }

  @Override
  public boolean driveCompleted(double inchesError) {
    return atFixedPosition(inchesError);
  }

  @Override
  public void rotateDegrees(double degrees, double speedFactor){

      driveFixedRotatePosition(degrees);
  }

}
