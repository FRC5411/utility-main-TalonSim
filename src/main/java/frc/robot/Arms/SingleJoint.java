package frc.robot.Arms;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SingleJoint extends SubsystemBase {
  // ARM PARAMETERS
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 2;
  private static final int kEncoderBChannel = 4;

  public static final String kArmPositionKey = "Single/ArmPosition";
  public static final String kArmPKey = "Single/ArmP";

  private static double kArmKp = 0.066;

  private static double armPositionDeg = 75.0;

  private static final double kArmEncoderDistPerPulse = 360 / 4096.0;

  private final DCMotor m_armGearbox = DCMotor.getNEO(1);

  private static final double m_armReduction = 192;
  private static final double m_armMass = Units.lbsToKilograms(16.0);
  private static final double m_armLength = Units.inchesToMeters(30);

  // CONTROLLER
  private final PIDController m_controller = new PIDController(kArmKp, 0, 0);
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);

  // SIMULATOR
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          m_armReduction,
          SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
          m_armLength,
          Units.degreesToRadians(-720),
          Units.degreesToRadians(720),
          m_armMass,
          false,
          VecBuilder.fill(0.01)
          );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  private double m_lastTime = 0.0;

  // ARM VISUALIZER
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              Units.metersToInches(m_armLength),
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  public SingleJoint() {
    Init();
  }

  public void Init() {
    m_controller.enableContinuousInput(-180, 180);
    m_encoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    SmartDashboard.putNumber("Single/Arm PID Output", 0);
    SmartDashboard.putData("1.0 JOINTS", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(Math.toDegrees(m_armSim.getAngleRads()));
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Math.toDegrees(m_armSim.getAngleRads()));

    m_lastTime += 0.02;

    SmartDashboard.putNumber("Single/Last Time", m_lastTime);
  }

  @Override
  public void periodic() {
          // Here, we run PID control like normal, with a constant setpoint of 75 degrees.
          var pidOutput =
          MathUtil.clamp(m_controller.calculate(m_encoder.getDistance(), armPositionDeg), -12, 12);
      SmartDashboard.putNumber("Single/Arm PID Output", pidOutput);

      m_motor.setVoltage(pidOutput);
  }
}
