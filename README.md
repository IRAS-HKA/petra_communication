# PeTRA Communication

Service which provides abstract user dialog and tools for debugging

## Quickstart:
```bash
# Start Communication Service
ros2 run petra_communication Communication
```
Start Keyboard node in a separate shell to be able to type and send text
```bash
# (Optional) For cleaner output format
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

# Start Keyboard node for in- and output
ros2 run petra_communication Keyboard
```