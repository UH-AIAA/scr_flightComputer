# Flight Software Main Guide
#### Welcome to AIAA flight software. This is the main guide on how to setup and build the flight software to ensure functionality before uploading it to main for review and merge. 

**After cloning from github, run:**<br>
`git submodule update --init --recursive`<br>
This will update all the library dependencies for the sensors on your local machine. 

**To build the Flight Software, run:**<br>
`python -m platformio run`

**To upload the Flight Software, run:**<br>
`python -m platformio run --target upload`
