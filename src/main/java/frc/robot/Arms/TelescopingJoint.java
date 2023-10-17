package frc.robot.Arms;

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

public class TelescopingJoint extends SubsystemBase {
   // CONTROLLERS
   private final ProfiledPIDController m_controller =
   new ProfiledPIDController(
       1,
       0,
       0,
       new TrapezoidProfile.Constraints(2.45, 2.45));
   private ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
         0,
         0,
         0,
         0);



// REAL LIFE MOTORS
   private final Encoder m_encoder =
      new Encoder(21, 28);
   private final PWMSparkMax m_motor = new PWMSparkMax(6);

// PARAMETERS
   private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);
   private final double gearing = 8;
   private final double massKg = 25;
   private final double drumRadiusMeters = 0.5;
   private final double minHeightMeters = 1;
   private final double maxHeightMeter = 25;
   private final boolean simGravity = false;
   private final Vector<N1> stdDevs = VecBuilder.fill(0);

// Sim Objects
   private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
         m_elevatorGearbox, 
         gearing, 
         massKg, 
         drumRadiusMeters, 
         minHeightMeters, 
         maxHeightMeter, 
         simGravity, 
         stdDevs);

   private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

// VISUALIZER \\
   private final Mechanism2d m_mech2d = new Mechanism2d(22, 50);
   private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
   private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
         new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));
   private final MechanismLigament2d m_AppendageMech2d = 
      m_elevatorMech2d.append(
         new MechanismLigament2d("Elevator Appendage", 0.3, 30 - 90));

   private final double goal = 5;

   public TelescopingJoint() {
      m_encoder.setDistancePerPulse((2 * Math.PI * drumRadiusMeters) / gearing);
      m_controller.reset(m_encoderSim.getDistance());
      SmartDashboard.putData("Elevator Sim", m_mech2d);
   }

   @Override
   public void periodic() {
      m_controller.setGoal(goal);

      double pidOutput = m_controller.calculate(m_encoderSim.getDistance());
      double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

      m_motor.setVoltage(pidOutput + feedforwardOutput);
   }

   @Override
   public void simulationPeriodic() {
      m_elevatorSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
      m_elevatorSim.update(0.020);

      m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
      RoboRioSim.setVInVoltage(
         BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

      m_elevatorMech2d.setLength(m_encoder.getDistance());
   }

   public void stop() {
      m_controller.setGoal(0.0);
      m_motor.set(0.0);
   }

   public void updateTelemetry() {}
}