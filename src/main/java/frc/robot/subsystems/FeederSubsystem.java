package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 */
public class FeederSubsystem extends TimedRobot {
//   private static final int leadDeviceID = 1;
//   private static final int followDeviceID = 2;
//   private static final int kJoystickPort = 0;

  private CANSparkMax m_leadMotor;
  private CANSparkMax m_followMotor;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final AnalogInput limitSwitch = new AnalogInput(0);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color OrangeTarget = new Color(0.546, 0.363, 0.091);


//   private Joystick m_joystick;

  @Override
  public void robotInit() {
    /**
     * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
     * 
     * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
     * first parameter
     * 
     * The motor type is passed as the second parameter. Motor type can either be:
     *  com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
     *  com.revrobotics.CANSparkLowLevel.MotorType.kBrushed
     * 
     * The example below initializes two brushless motors with CAN IDs 1 and 2. Change
     * these parameters to match your setup
     */
    m_leadMotor = new CANSparkMax(4, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(1,MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    // m_leadMotor.restoreFactoryDefaults();
    // m_followMotor.restoreFactoryDefaults();

    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     */
    m_followMotor.follow(m_leadMotor);

    // m_joystick = new Joystick(kJoystickPort);

    m_colorMatcher.addColorMatch(OrangeTarget);

  }

  @Override
  public void teleopPeriodic() {
    /**
     * m_followMotor will automatically follow whatever the applied output is on m_leadMotor.
     * 
     * Thus, set only needs to be called on m_leadMotor to control both of them
     */
    // m_leadMotor.set(m_joystick.getY());

    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == OrangeTarget && match.confidence > 0.85 ){

        colorString = "Orange";
  
      } else {
  
        colorString = "Unknown";
  
      }

      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putString("Detected Color", colorString);
  
      SmartDashboard.putNumber("Limit Switch", limitSwitch.getValue());
      boolean clicked;
      if (limitSwitch.getValue() <= 193) {
        clicked = true;
      } else {
        clicked = false;
      }
      SmartDashboard.putBoolean("Clicked", clicked);
  
    }

  }

