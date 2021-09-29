package org.team696.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.LogManager;

import org.team696.TCA9548ADriver.TCA9548A;
import org.team696.TCA9548ADriver.TCA9548ADevice;
import org.team696.TCS34725.Gain;
import org.team696.TCS34725.IntegrationTime;
import org.team696.TCS34725.TCS34725;
import org.team696.robot.Constants.SpindexerConstants;
import org.team696.utils.ColorUtils.Color;

public class BallSensors {
  private int num_sensors;
  private TCS34725[] sensors;
  private static final Logger logger = LogManager.getLogger(TCA9548ADevice.class);

  /**
   * Constructor that assumes a mux with the address given in Constants and sensors connected in order. 
   */
  public BallSensors() {
    num_sensors = SpindexerConstants.nPockets;
    sensors = new TCS34725[num_sensors];
    byte muxaddr = SpindexerConstants.ColorSensorsMuxAddress;
    TCA9548A mux = new TCA9548A(Port.kOnboard, muxaddr);
    int i;
    for (i = 0; i < num_sensors; i++) {
      TCA9548ADevice thisMuxChannel = new TCA9548ADevice(mux, TCS34725.I2C_constants.TCS34725_I2C_ADDR, i);
      TCS34725 thisSensor = new TCS34725(thisMuxChannel);
      sensors[i] = thisSensor;
    }
  }

  /**
   * Constructor that takes premade I2C channels.
   * @param channels I2C channels (should be num_sensors long)
   */
  public BallSensors(I2C[] channels) {
    num_sensors = SpindexerConstants.nPockets;
    sensors = new TCS34725[num_sensors];
    int i;
    for (i = 0; i < num_sensors; i++) {
      TCS34725 thisSensor = new TCS34725(channels[i]);
      sensors[i] = thisSensor;
    }
  }

  /**
   * Constructor that takes a mux and the indices of the mux channels going to the sensors. 
   * @param mux A TCA9548A mux object
   * @param muxChannels The mux channels to use (should be num_sensors long)
   */
  public BallSensors(TCA9548A mux, byte[] muxChannels) {
    num_sensors = SpindexerConstants.nPockets;
    sensors = new TCS34725[num_sensors];
    int i;
    for (i = 0; i < num_sensors; i++) {
      TCA9548ADevice thisMuxChannel = new TCA9548ADevice(mux, TCS34725.I2C_constants.TCS34725_I2C_ADDR, muxChannels[i]);
      TCS34725 thisSensor = new TCS34725(thisMuxChannel);
      thisSensor.setName(String.format("Pocket %d", i));
      sensors[i] = thisSensor;
    }
  }

  /**
   * Calls init() on all the sensors.
   */
  public void initSensors() {
    for (TCS34725 sensor : sensors) {
      sensor.init(IntegrationTime.IT_24MS, Gain.X1);
    }
  }


  public double[] getColor(int idx){
    double[] retval = new double[3];
    TCS34725 sensor = sensors[idx];
    if(sensor.readColors() == -1){
      sensor.init(IntegrationTime.IT_24MS, Gain.X1);
    }
    retval[0] = sensor.getRedVal();
    retval[1] = sensor.getGreenVal();
    retval[2] = sensor.getBlueVal();
    return retval;
  }
  /**
   * Gets whether each sensor sees a ball.
   * For each sensor, commands a reading, then compares the red, green, 
   * blue, and clear values to the limits in Constants to see if a ball 
   * is there.
   * @return An array of num_sensors booleans, each indicating if that sensor sees a ball
   */
  public boolean[] getResults() {
    boolean[] results = new boolean[num_sensors];
    int i;
    for (i = 0; i < num_sensors; i++) {
      TCS34725 sensor = sensors[i];
      if(sensor.readColors() == -1){
        sensor.init(IntegrationTime.IT_24MS, Gain.X1);
      }
      //TODO: figure out how to scale colors
      double red = sensor.getRedVal();
      double green = sensor.getGreenVal();
      double blue = sensor.getBlueVal();
      Color color = new Color(red, green, blue);
      double[] hsv = color.getHSV();
      boolean hasBall = true;
      if (hsv[0] < SpindexerConstants.HueMin || hsv[0] > SpindexerConstants.HueMax) {
        hasBall = false;
      }
      if (hsv[1] < SpindexerConstants.SatMin || hsv[1] > SpindexerConstants.SatMax) {
        hasBall = false;
      }
      if (hsv[2] < SpindexerConstants.ValueMin || hsv[2] > SpindexerConstants.ValueMax) {
        hasBall = false;
      }
      results[i] = hasBall;
    }

    return results;
  }

  public void setLEDs(boolean state){
    int i;
    for(i=0; i<num_sensors; i++){
      sensors[i].setLED(state);
    }
  }

  public void setLED(int idx, boolean state){
    sensors[idx].setLED(state);
  }

}