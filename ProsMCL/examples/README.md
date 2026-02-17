# Examples

These examples are practical autonomous templates.
Important: odom + IMU only will drift over long runs; use distance and/or vision corrections.

- `03_mcl_intervenes_with_lemlib.cpp`: full LemLib autonomous flow with distance sensors, auto-feed pose provider, parallel correction, and live tuning telemetry.
- `02_using_distance_sensors.cpp`: vision-assisted autonomous flow that combines odometry + distance + optional vision fixes.
- `01_minimal_motor_encoders_imu.cpp`: minimal baseline if you need to start from a simple manual-odom structure.
- `04_tuning_wizard_session.cpp`: microSD tuning wizard session logger (no USB required).
