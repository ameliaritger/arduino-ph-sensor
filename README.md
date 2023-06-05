# Arduino pH sensor 

## What's in this repository?
This repository contains the scripts and files related to the development of an intertidal Arduino pH sensor.

This repository is maintained by Hofmann Lab graduate student Amelia Ritger (GitHub: [@ameliaritger](https://github.com/ameliaritger)) at the University of California, Santa Barbara in the Department of Ecology, Evolution, & Marine Biology. Please direct any questions or comments about this repository to [Amelia Ritger](mailto:aritger@ucsb.edu).

## How is this repository structured?
```
.
├── documentation/                                # folder containing documentation for building the Arduino data logger for the pH sensor project
│   └── arduinoPHConstructionOperation.docx       # document for building the Arduino data logger
|
├── software/                 # folder containing the shiny dashboard 
|   └── arduinoCode/                          # markdown files containing text to be used throughout the app
|      └── CONFIG.h                    # caption for danner boots photo in gear garage
|      └── SRC.h                 # title and caption for DT dataTable
|      └── arduinoCode.ino               # gear garage box title and description
|       
|   └── rCode/                           # special directory in shiny for images, stylesheets, etc. 
|      └── Final_OMEGAS.Rmd                       # photos & logos used throughout app
|      └── Final_Tris_Cal.Rmd            # photos to be used in the 'Photos' tab (NOT WORKING YET)
|      └── Final_deploy_voltages.Rmd                  # stylesheet for customzing dashboard
|      └── Final_standardize_data.Rmd             # created when `fresh_theme.R` is run (NOT WORKING/USED YET)
|
├── draft_files/                 # folder containing the draft project files related to initial testing and development 
|   └── c_code/                          # folder containing all draft C code
|   └── data/                          # folder containing all draft data collected during field and lab tests
|   └── figures/                          # folder containing all draft figures created from data collected during field and lab tests
|   └── r_code/                          # folder containing all draft R code
|
├── README.md
├── .gitignore        
└── ph-sensor-dev.Rproj
```

# DOCUMENTATION TO COME, STAY TUNED!
