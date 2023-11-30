package org.firstinspires.ftc.teamcode;


public class Config {
  /*****************************************************************************
  ** DISTANCE CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int TICKS_PER_360_DEG = 2150; // TODO
  public static final int TILE_LENGTH = 800; // TODO

  public static final int DEFAULT_SPIKE_MARK = 3;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DRIVE SPEED CONSTANTS
  *****************************************************************************/
  public static final int MIN_SPEED = 450; // TODO
  public static final int CRUISE_SPEED = 1000; // TODO
  public static final int MAX_SPEED = 900; // TODO
  public static final int ARM_VELOCITY = 2000; // TODO
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** ARM CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int BOARD_HEIGHT = 70; // TODO
  
  public static final int BOTTOM = 0; // TODO
  public static final int CRUISING_HEIGHT = 30; // TODO
  public static final int TOP = 125; // TODO
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** CAMERA CALIBRATION CONSTANTS
  * Lens intrinsics
  * UNITS ARE PIXELS
  * NOTE: this calibration is for the C920 webcam at 800x448.
  * You will need to do your own calibration for other configurations!

  Resolution: 1280x720
  Pixel Size: 2.8um
  Sensor Size: 3.58x2.02mm
  Stock lens focal length: 4.2mm
  *****************************************************************************/
  public static final double FX = 1430;
  public static final double FY = 1430;
  public static final double CX = 480;
  public static final double CY = 620;

  public static final double TAGSIZE = 0.166;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DETECTION CONSTANTS
  *****************************************************************************/
  public static final float DECIMATION_HIGH = 3;
  public static final float DECIMATION_LOW = 2;
  public static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
  public static final int NUM_FRAMES_BEFORE_LOW_DECIMATION = 4;
  public static final int MAX_NUM_FRAMES_NO_DETECTION = 100; // How many attempts to detect before giving up
  // ---------------------------------------------------------------------------
}