# Arduino compatible Sky Quality Meter using the TSL2591

This is an application of the TSL 2591 sensor to measuring sky brightness for astronomical applications.  It returns the sky brightness in units of magnitude per square arc-second.  Also accessible to the user are the raw counts of the visible and IR channels.  

Some features:
 - Adaptive exposure and gain control to reduce noise in low light environment.  
 - Calibration offset can be adjusted by user with ```setCalibrationOffset```
 - Noise estimates given assuming signal counts follow a Poisson process

This, along with other Arduino projects, can be placed on an ESP8266 module.  I have written a library for using these modules to push data to an influx time-series database and display these results using Grafana.  Below is a screenshot the Grafana frontend with of one night.  Shown are two sensors, one in a weatherstation taking measurements from the sky, and another in a cabinet to illustrate the noise floor.  


![SQM Chart](/screenshots/SQMdual.png?raw=true "SQM Chart")
