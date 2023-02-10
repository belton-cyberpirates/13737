package org.firstinspires.ftc.teamcode;


public class Config {
  /*****************************************************************************
  ** DISTANCE CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int TICKS_PER_360_DEG = 2009;
  public static final int TILE_LENGTH = 800;
  public static final int INITIAL_CORRECTION = (int)(TILE_LENGTH * 0.10);
  public static final int BUMP = 270;
  public static final int DIAGONAL_BUMP = 400;

  public static final int DEFAULT_PARKING_SPOT = 3;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DRIVE SPEED CONSTANTS
  *****************************************************************************/
  
  public static final double MIN_SPEED = 0.45;
  public static final double MAX_SPEED = 0.6;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** ARM CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int LOW_POLE_HEIGHT = 70;
  public static final int MID_POLE_HEIGHT = 100;
  public static final int HIGH_POLE_HEIGHT = 120;
  public static final int SIDE_STACK_HEIGHT = 40;
  
  public static final int BOTTOM = 0;
  public static final int CRUISING_HEIGHT = 30;
  public static final int TOP = 125;
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