package frc.robot.Arms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Libs.TProfile.Constraints;
import frc.robot.Libs.ProfilePIDController;

public class TelescopingJoint extends SubsystemBase {
   // CONTROLLERS
   private final ProfilePIDController elevatorController =
      new ProfilePIDController(
         1,
         0,
         0.1,
       new Constraints(2.45, 2.45));
   private ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
         0,
         0,
         0,
         0);

   private final Encoder elevatorEncoder =
      new Encoder(1, 8);
   private final PWMSparkMax elevatorMotor = new PWMSparkMax(17);

   private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);
   private final double elevatorGearing = 8;
   private final double elevatorMassKg = 25;
   private final double elevatorDrumRadiusMeters = 0.5;
   private final double elevatorMinHeightMeters = 1;
   private final double elevatorMaxHeightMeter = 25;
   private final boolean simGravity = false;
   private final Vector<N1> stdDevs = VecBuilder.fill(0);

   private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
         m_elevatorGearbox, 
         elevatorGearing, 
         elevatorMassKg, 
         elevatorDrumRadiusMeters, 
         elevatorMinHeightMeters, 
         elevatorMaxHeightMeter, 
         simGravity, 
         stdDevs);

   private final EncoderSim elevatorEncoderSim = new EncoderSim(elevatorEncoder);

   private static final int kMotorPort = 15;
   private static final int kEncoderAChannel = 5;
   private static final int kEncoderBChannel = 6;
 
   public static final String kArmPositionKey = "Single/ArmPosition";
   public static final String kArmPKey = "Single/ArmP";
 
   private static double kArmKp = 0.066;
 
   private static double armPositionDeg = 75.0;
 
   private static final double kArmEncoderDistPerPulse = 360 / 4096.0;
 
   private final DCMotor armGearbox = DCMotor.getNEO(1);
 
   private static final double armReduction = 192;
   private static final double armMass = Units.lbsToKilograms(16.0);
   private static final double armLength = Units.inchesToMeters(30);
 
   // CONTROLLER
   private final PIDController armController = new PIDController(kArmKp, 0, 0);
   private final Encoder armEncoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
   private final PWMSparkMax armMotor = new PWMSparkMax(kMotorPort);
 
   // SIMULATOR
   private final SingleJointedArmSim m_armSim =
       new SingleJointedArmSim(
           armGearbox,
           armReduction,
           SingleJointedArmSim.estimateMOI(armLength, armMass),
           armLength,
           Units.degreesToRadians(-720),
           Units.degreesToRadians(720),
           armMass,
           false,
           VecBuilder.fill(0.01)
           );
   private final EncoderSim m_encoderSim = new EncoderSim(armEncoder);

   // VISUALIZER \\
   private final Mechanism2d m_mech2d = new Mechanism2d(22, 50);
   private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Telescope Root", 10, 0);
   private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
         new MechanismLigament2d("Telescope", 10, 90));
   private final MechanismLigament2d m_AppendageMech2d = 
      m_elevatorMech2d.append(
         new MechanismLigament2d("Telescope Appendage", 
         m_elevatorSim.getPositionMeters(), 
         Math.toDegrees(m_armSim.getAngleRads())));

   private final double goal = 5;

   private double lastTime = 0;

   public TelescopingJoint() {
      elevatorEncoder.setDistancePerPulse((2 * Math.PI * elevatorDrumRadiusMeters) / elevatorGearing);
      elevatorController.reset(m_encoderSim.getDistance());
      SmartDashboard.putData("telescoping arm", m_mech2d);

      armController.enableContinuousInput(-180, 180);
      armEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
      SmartDashboard.putNumber("Single/Arm PID Output", 0);
   }

   @Override
   public void periodic() {
      elevatorController.setGoal(goal);

      double elevatorOutput = elevatorController.calculate(m_elevatorSim.getPositionMeters());
      double feedforwardOutput = m_feedforward.calculate(elevatorController.getSetpoint().velocity);

      elevatorMotor.setVoltage(elevatorOutput + feedforwardOutput);
      SmartDashboard.putNumber("Telescop/Output", elevatorOutput + feedforwardOutput);

      double armOutput =
         MathUtil.clamp(armController.calculate(armEncoder.getDistance(), armPositionDeg), -12, 12);
            SmartDashboard.putNumber("Single/Arm PID Output", armOutput);
      
      armMotor.setVoltage(armOutput);
   }

   @Override
   public void simulationPeriodic() {
      m_elevatorSim.setInput(elevatorMotor.get() * RobotController.getBatteryVoltage());
      m_elevatorSim.update(0.020);

      m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
      RoboRioSim.setVInVoltage(
         BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

      SmartDashboard.putNumber("dist", elevatorEncoder.getDistance());
      m_AppendageMech2d.setLength(m_elevatorSim.getPositionMeters());

      m_armSim.setInput(armMotor.get() * RobotController.getBatteryVoltage());

      m_armSim.update(0.020);

      m_encoderSim.setDistance(Math.toDegrees(m_armSim.getAngleRads()));
      RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

      m_AppendageMech2d.setAngle(Math.toDegrees(m_armSim.getAngleRads()) - 90);

      lastTime += 0.02;

    SmartDashboard.putNumber("Telescope/Last Time", lastTime);
   }

   public void updateTelemetry() {}
}