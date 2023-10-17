package frc.robot.Arms;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.ProfilePIDController;
import frc.robot.Libs.TProfile;
import frc.robot.Libs.DoubleJointedArmDynamics;

public class DoubleJoint extends SubsystemBase {
  // ARM PARAMETERS
  private double wireMass = Units.lbsToKilograms(2.84);

  private double joint1Mass = Units.lbsToKilograms(3.54) + wireMass/2;
  private double joint1Length = Units.inchesToMeters(25);
  private double joint1CgRadius = Units.inchesToMeters(12.5);
  private double joint1Moi = SingleJointedArmSim.estimateMOI(joint1Length, joint1Mass);
  private double joint1Gearing = 100;

  private double joint2Mass = Units.lbsToKilograms(3.08) + wireMass/2;
  private double joint2Length = Units.inchesToMeters(30);
  private double joint2CgRadius = Units.inchesToMeters(15);
  private double joint2Moi = SingleJointedArmSim.estimateMOI(joint2Length, joint2Mass);
  private double joint2Gearing = 100;

  // ARM DYNAMICS
  // Only partially accurate to the real arm parameters
  DoubleJointedArmDynamics armDynamics = new DoubleJointedArmDynamics(
    new DoubleJointedArmDynamics.JointConfig(
      joint1Mass, 
      joint1CgRadius, 
      joint1Length, 
      joint1Moi,
      joint1Gearing, 
      DCMotor.getNEO(1)), 
    new DoubleJointedArmDynamics.JointConfig(
      joint2Mass,
      joint2CgRadius,
      joint2Length,
      joint2Moi,
      joint2Gearing,
      DCMotor.getNEO(1))
    );

    // ARM CONTROLLERS
    ProfilePIDController joint1Controller = new ProfilePIDController(0.1100, 0.02, 0.0003, 
                                              new TProfile.Constraints(600, 140));
    ProfilePIDController joint2Controller = new ProfilePIDController(0.03, 0.2, 0.0056, 
                                              new TProfile.Constraints(600, 160));

    ArmFeedforward stage1FF = new ArmFeedforward(0.0, 1.1, 0.0, 0.0);
    ArmFeedforward stage2FF = new ArmFeedforward(0.0, 0.175, 0.0, 0.0);

    // ARM STATES
    // N1 - N2 -> Position
    // N3 - N4 -> Velocity
    Vector<N4> armState = VecBuilder.fill(
      Math.toRadians(0),
      Math.toRadians(0),
      0.0, 0.0);

    // Colors
    Color8Bit yellow = new Color8Bit(Color.kYellow);
    Color8Bit green = new Color8Bit(Color.kGreen);
    Color8Bit blue = new Color8Bit(Color.kBlue);
   
    // ARM VISUALIZER
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivotDJ", 30, 25);
    private final MechanismLigament2d m_armTower =
        m_armPivot.append(new MechanismLigament2d("ArmTowerDJ", 50/2, -90));
    private final MechanismLigament2d m_arm =
        m_armPivot.append(
            new MechanismLigament2d(
              "ArmDJ",
              Units.metersToInches(joint1Length)/2,
              Units.radiansToDegrees(armState.get(0, 0)),
              10,
              yellow));
    private final MechanismLigament2d m_arm2 = 
        m_arm.append(
            new MechanismLigament2d(
              "Arm2DJ",
              Units.metersToInches(joint2Length)/2,
              Units.radiansToDegrees(armState.get(1, 0)),
              10,
              green
              ));

    // DUMMY CONTROLLERS
    private PWMSparkMax joint1Motor = new PWMSparkMax(1);
    private PWMSparkMax joint2Motor = new PWMSparkMax(2);

  public DoubleJoint() {
    configPID(joint1Controller);
    configPID(joint2Controller);

    joint1Controller.setIntegratorRange(-0.08, 0.09);
    joint2Controller.setIntegratorRange(-0.9, 0.7);

    joint1Controller.reset(armState.get(0, 0));
    joint2Controller.reset(armState.get(1, 0));



    SmartDashboard.putData("2.0 JOINTS", m_mech2d);
  }

  // TODO: add a lot more telemetry and fix the current telemetry
  // Bugs: THE controller errors erros are not being repored correctly
  // Math might be wrong

  @Override
  public void periodic() {
    armSetpoints();

    TProfile.State state1 = joint1Controller.getSetpoint();
    TProfile.State state2 = joint2Controller.getSetpoint();

    // Temporarily Commented out ka and kv terms
    Vector<N2> FF = armDynamics.calculate(
      VecBuilder.fill(
        Math.toRadians(state1.position), 
        Math.toRadians(state2.position)));
      // VecBuilder.fill(
      //   Math.toRadians(state1.velocity),
      //   Math.toRadians(state2.velocity),
      // VecBuilder.fill(
      //   Math.toRadians(state1.acceleration), 
      //   Math.toRadians(state2.acceleration)));

    double PID1 = joint1Controller.calculate(
      Math.toDegrees(armState.get(0, 0)), 135);

    double PID2 = joint1Controller.calculate(
      Math.toDegrees(armState.get(1, 0)), 30);

    double PID3 = joint1Controller.calculate(
      Math.toDegrees(armState.get(2, 0)), 0);

    double armFF1 = stage1FF.calculate(
      Math.toRadians(state1.position), 
      Math.toRadians(state1.velocity));

    double armFF2 = stage2FF.calculate(
      Math.toRadians(state2.position), 
      Math.toRadians(state2.velocity));

    double WPImath1 = PID1 + armFF1;
    double WPImath2 = PID2 + armFF2;

    double ANSmath1 = PID1 + FF.get(0, 0);
    double ANSmath2 = PID2 + FF.get(1, 0);

    joint1Motor.setVoltage(2);
    joint2Motor.setVoltage(2);

    SmartDashboard.putNumber("Double/PID1", PID1);
    SmartDashboard.putNumber("Double/PID2", PID2);
    SmartDashboard.putNumber("Double/PID3", PID3);

    SmartDashboard.putNumber("Double/Gravity 1", FF.get(0, 0));
    SmartDashboard.putNumber("Double/Gravity 2", FF.get(1, 0));

    armStates();
  }

  @Override
  public void simulationPeriodic() {
    armState = armDynamics.simulate(
      armState, 
      VecBuilder.fill(
        joint1Motor.get() * 12.0, 
        joint2Motor.get() * 12.0),
      0.02);

    // Subtracts the previous angle because each ligament is relative to the previous ligament. had to make them all relative to the x  axis
    m_arm.setAngle(Units.radiansToDegrees(armState.get(0, 0)));
    m_arm2.setAngle(Units.radiansToDegrees(armState.get(1, 0) - armState.get(0, 0)));
  }

  public void configPID(ProfilePIDController PID) {
    PID.enableContinuousInput(-180, 180);
  }

  public void armStates() {
    SmartDashboard.putNumber("Double/Measure 1", Units.radiansToDegrees(armState.get(0, 0)));
    SmartDashboard.putNumber("Double/Measure 2", Units.radiansToDegrees(armState.get(1, 0)));
  }

  public void armSetpoints() {
    SmartDashboard.putNumber("Double/Setpoint 1", joint1Controller.getSetpoint().position);
    SmartDashboard.putNumber("Double/Setpoint 2", joint2Controller.getSetpoint().position);
  }
}