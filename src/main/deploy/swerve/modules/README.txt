I modifed how PID, kS, kV, and kA values are passed to the modules.
Instead of using pidfproperties.json, pass the values into the module config josns themselves
Use:

"driveTuning": {
    "p": 0,
    "i": 0,
    "d": 0,
    "kS": 0,
    "kV": 0,
    "kA": 0
},
"angleTuning": {
    "p": 0,
    "i": 0,
    "d": 0,
    "kS": 0,
    "kV": 0,
    "kA": 0
}

in the module json config files. If the pidfproperties.json file is missing or its p value is 0, then the Swerve Drive will use these tuning configuration values instead.

- Jonah Kowal