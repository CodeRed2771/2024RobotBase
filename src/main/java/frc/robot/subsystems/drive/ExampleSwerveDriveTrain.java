package frc.robot.subsystems.drive;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Calibration;

public class ExampleSwerveDriveTrain extends DriveSubsystem {

  private static final double kMaxSpeed = 1.0; // normalized full Speed

  private NewSwerveModuleVortex m_frontLeft; // Front Left
  private NewSwerveModuleVortex m_backRight; // Back Right
  private NewSwerveModuleVortex m_backLeft; // Back Left
  private NewSwerveModuleVortex m_frontRight; // Front Right

  // Robot Center is 0,0 anchor point for coordinates (where NavX is)
  // +X = out intake
  // +Y = out right side of robot
  private final double wheel_position_offset = (15.0 - 3.25) * (2.54 / 100.0);

  private final Translation2d m_frontLeftLocation = new Translation2d(wheel_position_offset, wheel_position_offset);
  private final Translation2d m_frontRightLocation = new Translation2d(wheel_position_offset, -wheel_position_offset);
  private final Translation2d m_backLeftLocation = new Translation2d(-wheel_position_offset, wheel_position_offset);
  private final Translation2d m_backRightLocation = new Translation2d(-wheel_position_offset, -wheel_position_offset);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
      m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private SwerveDriveOdometry m_odometry;

  public ExampleSwerveDriveTrain(Map<String, Integer> wiring) {
    super();

    Calibration.loadSwerveCalibration();

    int TURN_ABS_ENC_A = wiring.get("A turn enc");
    int TURN_ABS_ENC_B = wiring.get("B turn enc");
    int TURN_ABS_ENC_C = wiring.get("C turn enc");
    int TURN_ABS_ENC_D = wiring.get("D turn enc");

    int DT_A_DRIVE_ID = wiring.get("A drive");
    int DT_A_TURN_ID = wiring.get("A turn");
    int DT_B_DRIVE_ID = wiring.get("B drive");
    int DT_B_TURN_ID = wiring.get("B turn");
    int DT_C_DRIVE_ID = wiring.get("C drive");
    int DT_C_TURN_ID = wiring.get("C turn");
    int DT_D_DRIVE_ID = wiring.get("D drive");
    int DT_D_TURN_ID = wiring.get("D turn");

    m_frontLeft = new NewSwerveModuleVortex(DT_A_DRIVE_ID, DT_A_TURN_ID, TURN_ABS_ENC_A, "A"); // Front right
    m_backRight = new NewSwerveModuleVortex(DT_B_DRIVE_ID, DT_B_TURN_ID, TURN_ABS_ENC_B,"B"); // Back left
    m_backLeft = new NewSwerveModuleVortex(DT_C_DRIVE_ID, DT_C_TURN_ID, TURN_ABS_ENC_C, "C"); // Back right
    m_frontRight = new NewSwerveModuleVortex(DT_D_DRIVE_ID, DT_D_TURN_ID, TURN_ABS_ENC_D, "D"); // Front left

    setTurnOffsets( Calibration.getTurnZeroPos('A'), 
                    Calibration.getTurnZeroPos('B'), 
                    Calibration.getTurnZeroPos('C'),
                    Calibration.getTurnZeroPos('D'));

    m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});

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

  /** Updates the field relative position of the robot. */
  public void updateOdometry(Rotation2d robot_orentiation) {
    m_odometry.update(robot_orentiation, new SwerveModulePosition[] { // Maintain order from m_odometry creation
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()});
  }

  public Pose2d getOdometryPosition(){
    return m_odometry.getPoseMeters();
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
        .toSwerveModuleStates(ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, rot), periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    // Maintain order from m_kinematics creation
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveFixedOrientation(double xSpeed, double ySpeed)
  {
    Rotation2d ang = new Rotation2d(xSpeed, ySpeed);
    Translation2d Speed = new Translation2d(xSpeed,ySpeed);
    SwerveModuleState desiredState = new SwerveModuleState(Speed.getNorm(), ang);

    m_frontLeft.setDesiredState(desiredState);
    m_frontRight.setDesiredState(desiredState);
    m_backLeft.setDesiredState(desiredState);
    m_backRight.setDesiredState(desiredState);

  }

  public void driveParkControl(){
    m_frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    m_backLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    m_backRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
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
}
