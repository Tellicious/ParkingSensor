## HW and SW change to allow usage with battery power
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

See [Changelog](Changelog.md)