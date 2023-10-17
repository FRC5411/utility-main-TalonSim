package frc.robot.Arms;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
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
import frc.robot.Libs.TripleJointedArmDynamics;

public class TripleJoint extends SubsystemBase {
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

  private double joint3Mass = Units.lbsToKilograms(7.78);
  private double joint3Length = Units.inchesToMeters(20);
  private double joint3CgRadius = Units.inchesToMeters(10);
  private double joint3Moi = SingleJointedArmSim.estimateMOI(joint3Length, joint3Mass);
  private double joint3Gearing = 20;

  // ARM DYNAMICS
  // Only partially accurate to the real arm parameters
  TripleJointedArmDynamics armDynamics = new TripleJointedArmDynamics(
    new TripleJointedArmDynamics.JointConfig(
      joint1Mass, 
      joint1CgRadius, 
      joint1Length, 
      joint1Moi,
      joint1Gearing, 
      DCMotor.getNEO(1)), 
    new TripleJointedArmDynamics.JointConfig(
      joint2Mass,
      joint2CgRadius,
      joint2Length,
      joint2Moi,
      joint2Gearing,
      DCMotor.getNEO(1)),    
    new TripleJointedArmDynamics.JointConfig(
      joint3Mass,
      joint3CgRadius,
      joint3Length,
      joint3Moi,
      joint3Gearing,
      DCMotor.getNEO(1))
    );

    // ARM CONTROLLERS
    ProfilePIDController joint1Controller = new ProfilePIDController(0.1100, 0.02, 0.0003, 
                                              new TProfile.Constraints(600, 140));
    ProfilePIDController joint2Controller = new ProfilePIDController(0.03, 0.2, 0.0056, 
                                              new TProfile.Constraints(600, 160));
    ProfilePIDController joint3Controller = new ProfilePIDController(0.1, 0.0, 0.0001,
                                              new TProfile.Constraints(500, 300));

    ArmFeedforward stage1FF = new ArmFeedforward(0.0, 1.1, 0.0, 0.0);
    ArmFeedforward stage2FF = new ArmFeedforward(0.0, 0.175, 0.0, 0.0);
    ArmFeedforward stage3FF = new ArmFeedforward(0.0, 0.05, 0.0, 0.0);

    // ARM STATES
    // N1 - N3 -> Position
    // N4 - N6 -> Velocity
    Vector<N6> armState = VecBuilder.fill(
      Math.toRadians(0),
      Math.toRadians(0),
      Math.toRadians(0), 0.0, 0.0, 0.0);

    // Colors
    Color8Bit yellow = new Color8Bit(Color.kYellow);
    Color8Bit green = new Color8Bit(Color.kGreen);
    Color8Bit blue = new Color8Bit(Color.kBlue);

    // ARM VISUALIZER
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 35);
    private final MechanismLigament2d m_armTower =
        m_armPivot.append(new MechanismLigament2d("ArmTower", 35, -90));
    private final MechanismLigament2d m_arm =
        m_armPivot.append(
            new MechanismLigament2d(
              "Arm",
              Units.metersToInches(joint1Length)/2,
              Units.radiansToDegrees(armState.get(0, 0)),
              10,
              yellow));
    private final MechanismLigament2d m_arm2 = 
        m_arm.append(
            new MechanismLigament2d(
              "Arm2",
              Units.metersToInches(joint2Length)/2,
              Units.radiansToDegrees(armState.get(1, 0)),
              10,
              green
              ));
    private final MechanismLigament2d m_arm3 = 
        m_arm2.append(
            new MechanismLigament2d(
              "Arm3",
              Units.metersToInches(joint3Length)/2,
              Units.radiansToDegrees(armState.get(2, 0)),
              50,
              blue
              ));

    // DUMMY CONTROLLERS
    private PWMSparkMax joint1Motor = new PWMSparkMax(3);
    private PWMSparkMax joint2Motor = new PWMSparkMax(4);
    private PWMSparkMax joint3Motor = new PWMSparkMax(5);

  public TripleJoint() {
    configPID(joint1Controller);
    configPID(joint2Controller);
    configPID(joint3Controller);

    joint1Controller.setIntegratorRange(-0.08, 0.09);
    joint2Controller.setIntegratorRange(-0.9, 0.7);
    joint3Controller.setIntegratorRange(-0.3, 0.5);

    joint1Controller.reset(armState.get(0, 0));
    joint2Controller.reset(armState.get(1, 0));
    joint3Controller.reset(armState.get(2, 0));



    SmartDashboard.putData("3.0 JOINTS", m_mech2d);
  }

  // TODO: add a lot more telemetry and fix the current telemetry
  // Bugs: THE controller errors erros are not being repored correctly
  // Math might be wrong

  @Override
  public void periodic() {
    armSetpoints();

    TProfile.State state1 = joint1Controller.getSetpoint();
    TProfile.State state2 = joint2Controller.getSetpoint();
    TProfile.State state3 = joint3Controller.getSetpoint();

    // Temporarily Commented out ka and kv terms
    Vector<N3> FF = armDynamics.calculate(
      VecBuilder.fill(
        Math.toRadians(state1.position), 
        Math.toRadians(state2.position), 
        Math.toRadians(state3.position)));
      // VecBuilder.fill(
      //   Math.toRadians(state1.velocity),
      //   Math.toRadians(state2.velocity),
      //   Math.toRadians(state3.velocity)),
      // VecBuilder.fill(
      //   Math.toRadians(state1.acceleration), 
      //   Math.toRadians(state2.acceleration), 
      //   Math.toRadians(state3.acceleration)));

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

    double armFF3 = stage3FF.calculate(
      Math.toRadians(state3.position), 
      Math.toRadians(state3.velocity));

    double WPImath1 = PID1 + armFF1;
    double WPImath2 = PID2 + armFF2;
    double WPImath3 = PID3 + armFF3;

    double ANSmath1 = PID1 + FF.get(0, 0);
    double ANSmath2 = PID2 + FF.get(1, 0);
    double ANSmath3 = PID3 + FF.get(2, 0);

    // joint1Motor.setVoltage(ANSmath1);
    // joint2Motor.setVoltage(ANSmath2);
    // joint3Motor.setVoltage(ANSmath3);

    joint1Motor.setVoltage(2);
    joint2Motor.setVoltage(2);
    joint3Motor.setVoltage(2);

    SmartDashboard.putNumber("Triple/PID1", PID1);
    SmartDashboard.putNumber("Triple/PID2", PID2);
    SmartDashboard.putNumber("Triple/PID3", PID3);

    SmartDashboard.putNumber("Triple/Gravity 1", FF.get(0, 0));
    SmartDashboard.putNumber("Triple/Gravity 2", FF.get(1, 0));
    SmartDashboard.putNumber("Triple/Gravity 3", FF.get(2, 0));

    armStates();
  }

  @Override
  public void simulationPeriodic() {
    armState = armDynamics.simulate(
      armState, 
      VecBuilder.fill(
        joint1Motor.get() * 12, 
        joint2Motor.get() * 12, 
        joint3Motor.get() * 12),
      0.02);

    // Subtracts the previous angle because each ligament is relative to the previous ligament. had to make them all relative to the x  axis
    m_arm.setAngle(Units.radiansToDegrees(armState.get(0, 0)));
    m_arm2.setAngle(Units.radiansToDegrees(armState.get(1, 0) - armState.get(0, 0)));
    m_arm3.setAngle(Units.radiansToDegrees(armState.get(2, 0) - armState.get(1, 0)));
  }

  public void configPID(ProfilePIDController PID) {
    PID.enableContinuousInput(-180, 180);
  }

  public void armStates() {
    SmartDashboard.putNumber("Triple/Measure 1", Units.radiansToDegrees(armState.get(0, 0)));
    SmartDashboard.putNumber("Triple/Measure 2", Units.radiansToDegrees(armState.get(1, 0)));
    SmartDashboard.putNumber("Triple/Measure 3", Units.radiansToDegrees(armState.get(2, 0)));
  }

  public void armSetpoints() {
    SmartDashboard.putNumber("Triple/Setpoint 1", joint1Controller.getSetpoint().position);
    SmartDashboard.putNumber("Triple/Setpoint 2", joint2Controller.getSetpoint().position);
    SmartDashboard.putNumber("Triple/Setpoint 3", joint3Controller.getSetpoint().position);
  }
}