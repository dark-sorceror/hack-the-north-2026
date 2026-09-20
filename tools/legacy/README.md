# Legacy hardware experiments

These files were retained for reference from an earlier setup. They assume the old `VLA-HTN` checkout, machine names, or a separately built RealSense SDK. They are not part of the current startup instructions and are excluded from the automated test suite.

- `debug_realsense.py`: earlier SDK path inspection helper.
- `setup_realsense_pi.sh`: old SDK build/install helper.
- `hardware_checks/`: manual camera, device-enumeration, and SSH diagnostics.

Use [the development guide](../../docs/development.md) and `hardware/` for the current entry points. Do not run a provisioning script without reviewing its paths and target machine.

The `ssh/` directory preserves the original passwordless setup helpers. They change SSH key protection and grant unrestricted passwordless sudo; they are archived for reference. Use [the current SSH guide](../../docs/setup/ssh.md) for normal development.
