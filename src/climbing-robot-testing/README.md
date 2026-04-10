# Climbing Robot Testing

## Overview

The Climbing Robot Testing project is designed to ensure the functionality, reliability, and performance of the climbing robot through a series of comprehensive tests. This project includes detailed documentation, scripts for automated testing, and configuration files necessary for the robot's operation.

## Purpose

The purpose of this project is to provide a structured approach to testing the climbing robot, ensuring that all components function as intended and meet the specified requirements. The testing process aims to identify any issues early in the development cycle, facilitating timely resolutions and improvements.

## Project Structure

The project is organized into the following directories and files:

- **docs/**: Contains all documentation related to the testing process.
  - `FUNCTIONAL_TEST_EXECUTION_SHEET.md`: A comprehensive testing guide detailing the purpose, experimental information, common preparation commands, record format, environment and build checks, node startup checks, message interface validation, and single node functionality verification.
  - `TEST_PLAN.md`: Outlines the overall testing strategy, including objectives, scope, resources required, and timelines for the testing process.
  - `TEST_CHECKLIST.md`: Provides a checklist of all tests to be performed, ensuring that all necessary tests are accounted for and completed.
  - `TEST_REPORT_TEMPLATE.md`: A template for reporting the results of the tests conducted, including sections for observations, conclusions, and recommendations.

- **scripts/**: Contains shell scripts for automated testing processes.
  - `run_precheck.sh`: Performs preliminary checks before running the main tests, ensuring that the environment is set up correctly.
  - `run_node_health_check.sh`: Checks the health of the various nodes in the climbing robot's system, verifying that they are running and responding as expected.
  - `run_message_validation.sh`: Validates the messages being published and subscribed to within the robot's communication framework, ensuring that they conform to expected formats and values.

- **configs/**: Contains configuration files for the climbing robot.
  - `robot.yaml`: A YAML configuration file containing parameters and settings specific to the climbing robot, such as sensor configurations, operational parameters, and other relevant settings.

## Setup Instructions

1. Clone the repository to your local machine.
2. Navigate to the project directory.
3. Ensure that all dependencies are installed as specified in the documentation.
4. Configure the `robot.yaml` file according to your robot's specifications.
5. Run the preliminary checks using the provided scripts before executing the tests.

## Usage

- To perform preliminary checks, run:
  ```bash
  ./scripts/run_precheck.sh
  ```

- To check the health of the nodes, execute:
  ```bash
  ./scripts/run_node_health_check.sh
  ```

- To validate message formats and values, use:
  ```bash
  ./scripts/run_message_validation.sh
  ```

## Conclusion

This project aims to provide a robust framework for testing the climbing robot, ensuring that it operates effectively in its intended environment. By following the guidelines and utilizing the provided resources, users can contribute to the ongoing development and improvement of the climbing robot.