# Changelog

## v2.0.0
**New Features:**
- HW change: Added LIDAR interrupt and MOSFET to control LED bar power
- Modified code to put microcontoller in STOP mode and to turn-off LED bar when no movement is detected
- Added VL53L1X ultra-low-power drivers to further reduce consumption
  
**Improvements:**
- Added LIDAR temperature calibration upon startup
- Changed some parameters
- Updated dependencies

**Bugfix:**
- Removed code that turned LED off when LIDAR status was not correct

## v1.0.1

**Bugfix:**
- Changed `rangeStatus` check to avoid issues in case of LIDAR sensor wrap-around

## v1.0.0

**First Release**