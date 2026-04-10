# TEST_CHECKLIST.md

## Purpose
This checklist serves as a comprehensive guide to ensure that all necessary tests for the climbing robot are accounted for and completed. It is designed to facilitate the testing process by providing a structured format for tracking the status of each test.

## Experimental Information
- **Experiment Date:** 
- **Tester Name:** 
- **Robot ID:** 
- **Testing Environment:** 
- **Software Version:** 
- **Configuration Version/Hash:** 
- **Logger Enabled:** 
- **Rosbag Recording:** 
- **Notes:** 

## Common Preparation Commands
Before starting the tests, execute the following commands in the terminal:

```bash
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

If the workspace has not been built yet, run:

```bash
cd ~/climbing_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

## Record Format
Each test should be recorded using the following format:

- **Test Name:** 
- **Status:** [ ] Passed [ ] Failed [ ] Blocked
- **Actual Outcome:** 
- **Key Observations:** 
- **Issues/Remarks:** 

## Environment and Build Checks
- **T01:** Workspace Build
- **T02:** Parameter File Loading
- **T03:** Launch File Validity

## Node Startup Checks
- **T04:** PC Node Startup Check
- **T05:** Jetson Node Startup Check
- **T06:** Logger Node Startup Check

## Message Interface Validation
- **T07:** Estimated State Message Integrity
- **T08:** Body Reference Publishing Correctness
- **T09:** Swing Leg Target Publishing Correctness
- **T10:** Stance Force Message Integrity
- **T11:** Jetson Bridge Message Link

## Single Node Functionality Verification
- **T12:** State Estimator Contact State Determination
- **T13:** Swing Leg Five-Stage Behavior Check
- **T14:** Mission Supervisor Fan Timing Check
- **T15:** Stance Force Optimizer Activation Logic
- **T16:** Leg IK Executor Mapping Check

## Conclusion
This checklist is intended to ensure thorough testing of the climbing robot's systems and functionalities. Each test should be marked as completed once verified, and any issues should be documented for further analysis.