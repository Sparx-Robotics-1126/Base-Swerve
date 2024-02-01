package frc.lib.swervelib.imu;

// import com.ctre.phoenix.sensors.Pigeon2Configuration;
// import com.ctre.phoenix.sensors.Pigeon2;

// import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

//import com.ctre.phoenix.sensors.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * SwerveIMU interface for the Pigeon2
 */
public class Pigeon2Swerve extends SwerveIMU
{
  // private static Pigeon2Swerve m_instance = null;
  /**
   * Pigeon2 IMU device.
   */
  Pigeon2 imu;
  /**
   * Offset for the Pigeon 2.
   */
  private Rotation3d offset = new Rotation3d();

  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid  CAN ID for the pigeon
   * @param canbus CAN Bus name the pigeon resides on.
   */
  public Pigeon2Swerve(int canid, String canbus)
  {
    imu = new Pigeon2(canid, canbus);
    // Pigeon2Configuration config = new Pigeon2Configuration();
    // imu.configAllSettings(config);



    var toApply = new Pigeon2Configuration();
    // var mountPose = toApply.MountPose;
    toApply.MountPose.MountPosePitch = 0;
    toApply.MountPose.MountPoseRoll=0;
    toApply.MountPose.MountPoseYaw=0;
    // toApply.MountPose.MountPoseRoll = 0;
    // toApply.MountPose.MountPoseYaw = -180;//-90;  This might have been wrong if upside down
    /*
     * User can change the configs if they want, or leave it empty for
     * factory-default
     */

    imu.getConfigurator().apply(toApply);


    imu.getPitch().setUpdateFrequency(1000);
    imu.setYaw(0, .1);

    imu.getYaw().setUpdateFrequency(1000);
    imu.getYaw().waitForUpdate(.1);

    SmartDashboard.putData(imu);

  }
  // public static Pigeon2Swerve getInstance() {
  //   if (m_instance == null) {
  //     m_instance = new Pigeon2Swerve(4);
  //   }
  //   return m_instance;
  // }
  /**
   * Generate the SwerveIMU for pigeon.
   *
   * @param canid CAN ID for the pigeon
   */
  public Pigeon2Swerve(int canid)
  {
    this(canid, "");
  }

  /**
   * Reset IMU to factory default.
   */
  @Override
  public void factoryDefault()
  {
    // imu..configFactoryDefault();
    // imu.configEnableCompass(false); // Compass utilization causes readings to jump dramatically in some cases.
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
    imu.clearStickyFaults();
  }

  /**
   * Set the gyro offset.
   *
   * @param offset gyro offset as a {@link Rotation3d}.
   */
  public void setOffset(Rotation3d offset)
  {
    this.offset = offset;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRawRotation3d()
  {
    double[] wxyz = new double[4];
    
    // imu.getQuatW().get6dQuaternion(wxyz);
    return new Rotation3d(new Quaternion(imu.getQuatW().getValue(),imu.getQuatX().getValue(),imu.getQuatY().getValue(), imu.getQuatZ().getValue()));
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRotation3d()
  {
    // return getRawRotation3d();
    return getRawRotation3d().minus(offset);
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel()
  {
    short[] initial = new short[3];

    return Optional.of(new Translation3d(imu.getAccelerationX().getValue(), imu.getAccelerationY().getValue(), imu.getAccelerationZ().getValue()).times(9.81 / 16384.0));
  }

  @Override
  public Optional<Rotation3d> getAngularVel()
  {
    double[] initial = new double[3];
    initial[0] = imu.getRoll().getValue();
    initial[1] = imu.getPitch().getValue();
    initial[2] = imu.getYaw().getValue();
    return Optional.of(new Rotation3d(Math.toRadians(initial[0]), Math.toRadians(initial[1]), Math.toRadians(initial[2])));
  }

  @Override
  public double getRoll() {
  return imu.getRoll().getValue();
  }

  /**
   * Get the instantiated IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }
}
