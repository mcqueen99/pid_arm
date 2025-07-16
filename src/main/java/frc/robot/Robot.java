
package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



@SuppressWarnings("PMD.RedundantFieldInitializer")
public class Robot extends TimedRobot {
  private static double kDt = 0.02; //ms
  private static double kMaxVelocity = 2; //max hız
  private static double kMaxAcceleration = 0.75;
  private static double kP = 0.0;
  private static double kI = 0.0;
  private static double kD = 0.0;
  private static double kS = 1.1;
  private static double kG = 1.2;
  private static double kV = 1.3;

  private static final int kMotorPort= ;
  private double minAngle;
  private double maxAngle;

  private final Joystick m_joystick = new Joystick(1);
  private final Encoder m_encoder = new Encoder(channelA:, channelV:);
  private final PWMSparkMax m_motor = new PWMSparkMax(1);

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

  public Robot() {
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
  }
 
  @Override
  public void teleopPeriodic() {

    double joystickValue = m_joystick.getY();
    double motorSpeed = joystickValue*0.5;

    if (m_joystick.getRawButtonPressed(1)) {
      minAngle = 0;
      maxAngle = 45;

      m_controller.setGoal(1.5);

      double currentAngle = m_encoder.getDistance();
      if(currentAngle <= minAngle && motorSpeed < 0){
      motorSpeed = 0;}
      else if (currentAngle >= maxAngle && motorSpeed >0) {
      motorSpeed = 0;
      }
    } 
    else if (m_joystick.getRawButtonPressed(2)) {
      minAngle = 0;
      maxAngle = 10;

      double currentAngle=m_encoder.getDistance();
      if(currentAngle <= minAngle && motorSpeed < 0){
        motorSpeed = 0;}
      else if (currentAngle >= maxAngle && motorSpeed >0) {
        motorSpeed = 0;

      m_controller.setGoal(1);
    }
    else if(m_joystick.getRawButtonPressed(3)) {
      m_controller.setGoal(0);
    }

    // Run controller and update motor output
    m_motor.setVoltage(
        m_controller.calculate(m_encoder.getDistance())
            + m_feedforward.calculate(m_controller.getSetpoint().velocity));
  }
}
}