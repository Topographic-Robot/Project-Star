# Project Star Changelog

## 2025-03-2
- Implemented log compression for storage efficiency:
  - Added zlib compression support to log_storage module
  - Created binary file write capability in file_write_manager
  - Enabled automatic compression of log files with .gz extension
  - Added compression configuration API
  - Integrated compression with existing log rotation system

## 2025-02-26
- Added sequence numbers to all log messages
- Added timestamp to all log messages
- Added thread/task ID for multi-threaded debugging

## 2025-02-25
- FPGA Code Style Improvements:
  - Reformatted all Verilog files to follow consistent style guidelines:
    - Updated buffCapControl.v with consistent comment style and indentation
    - Updated vgaGen.v with proper formatting and comment style
    - Updated dataRegistering.v with consistent indentation and spacing
    - Updated testBench.v with proper formatting without cross-block alignment
  - Applied consistent style across all files:
    - Used /* */ style comments instead of // comments
    - Placed begin on the same line as if/else statements
    - Placed begin on the next line after always blocks
    - Used consistent 2-space indentation
    - Removed excessive alignment across different blocks
    - Used camelCase for signal names
    - Added proper spacing around operators

## 2025-02-24
- FPGA Project Cleanup:
  - Added .gitignore patterns for Quartus-generated files
  - Added .gitignore patterns for AMD/Xilinx-generated files
  - Removed tracked generated files from repository:
    - Removed db/, incremental_db/, output_files/, and greybox_tmp/ directories
    - Removed *_bb.v (black box) files
    - Removed .qws, .bak, .rpt, .ppf files
    - Removed other Quartus-generated temporary files
  - Added file headers to all Verilog files
  - Added new FPGA Code Style TODOs for all Verilog files

## 2025-02-24
- Updated sensor HALs to use new logging format:
  - Converted BH1750 sensor HAL from ESP_LOG* to log_*
  - Converted MQ135 sensor HAL from ESP_LOG* to log_*
  - MQ135 has better error handling and logging
- Updated controller HALs to use new logging format:
  - Converted PCA9685 motor controller HAL from ESP_LOG* to log_*
  - Converted EC11 encoder HAL from ESP_LOG* to log_*
- Updated common components to use new logging format:
  - Converted log_handler.c from ESP_LOG* to log_*
  - Converted error_handler.c from ESP_LOG* to log_*
  - Converted i2c.c from ESP_LOG* to log_*
  - Converted uart.c from ESP_LOG* to log_*
- Updated sd_card_hal.c to use new logging format:
  - Converted sd_card_hal.c from ESP_LOG* to log_*
- Updated time_manager.c to use new logging format:
  - Converted time_manager.c from ESP_LOG* to log_*
- Updated sensor_tasks.c to use new logging format:
  - Converted sensor_tasks.c from ESP_LOG* to log_*
- Updated webserver_tasks.c to use new logging format:
  - Converted webserver_tasks.c from ESP_LOG* to log_*
- Updated file_write_manager.c to use new logging format:
  - Converted file_write_manager.c from ESP_LOG* to log_*
- Updated main.c to use new logging format:
  - Converted main.c from ESP_LOG* to log_*
- Updated gait_movement.c to use new logging format:
  - Converted gait_movement.c from ESP_LOG* to log_*
- Updated hexapod_geometry.c to use new logging format:
  - Converted hexapod_geometry.c from ESP_LOG* to log_*
- Updated %d to %u in all log messages that include unsigned integers

## 2025-02-23 
- Updated sensor HALs to use new logging format:
  - Converted DHT22 sensor HAL from ESP_LOG* to log_*
  - Converted GY-NEO6MV2 GPS HAL from ESP_LOG* to log_*
  - Converted MPU6050 motion sensor HAL from ESP_LOG* to log_*
  - Converted QMC5883L compass HAL from ESP_LOG* to log_*
  - Converted OV7670 camera HAL from ESP_LOG* to log_*
  - Converted CCS811 air quality sensor HAL from ESP_LOG* to log_*
- Added descriptive action and status messages to all log calls
- Maintained existing functionality while updating log format

## 2025-02-22
- Initial TODO list creation
- Organized tasks by priority and component
- Added system-wide tasks for stability and testing
- Implemented core logging system with inline functions and proper variadic argument handling
