# Arduino pH sensor 

![pH sensor in the rocky intertidal during sunset](/media/alg_sunset-wide.png?raw=true)

## What's in this repository?
This repository contains the scripts and files related to the development of an intertidal Arduino pH sensor.

This repository is maintained by Hofmann Lab graduate student Amelia Ritger (GitHub: [@ameliaritger](https://github.com/ameliaritger)) at the University of California, Santa Barbara in the Department of Ecology, Evolution, & Marine Biology. Please direct any questions or comments about this repository to [Amelia Ritger](mailto:aritger@ucsb.edu).

## How is this repository structured?
```
.
├── documentation/                              # folder containing documentation for building the Arduino data logger for the pH sensor project
│   └── arduinoPHConstructionOperation.docx     # document for building the Arduino data logger
|
├── software/                                   # folder containing software relevant to the Arduino data logger
|   └── arduinoCode/                            # folder containing C++ code to program the Arduino
|      └── CONFIG.h                             # configuration script, to set filename, sampling interval, and sampling start date & time
|      └── SRC.h                                # source script, for useful functions
|      └── arduinoCode.ino                      # main script, to sample from the pH sensor and store the values on the Arduino
|           
|   └── rCode/                                  # folder containing R code to process pH sensor data. Note: currently formatted for project development, not finalized for public application!
|      └── Final_OMEGAS.Rmd                     # Step 3: Apply calibration constants to deployment data
|      └── Final_Tris_Cal.Rmd                   # STEP 2: Calculate calibration constants for the sensor
|      └── Final_deploy_voltages.Rmd            # STEP 4: Plot calibrated sensor data with HOBO logger temperature data
|      └── Final_standardize_data.Rmd           # STEP 1: Standardize Arduino (new design) and Madgetech (old design) data
|
├── media/                                      # folder containing media files for project repo
|
├── draft_files/                                # folder containing all draft project files related to initial testing and development
|
├── README.md
├── .gitignore        
└── ph-sensor-dev.Rproj
```