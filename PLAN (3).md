# PLAN.md — ROS 2 Mock Bridge + Manual GUI

## Goal
Build a standalone ROS 2 Mock Bridge for testing the **real Orchestrator** without the VCU, STM32, UART, or physical vehicle. It must replace the real Bridge at the ROS interface without changes to the Orchestrator.

## Deployment
- ROS 2 Jazzy, Python (`rclpy`), simple desktop GUI (prefer PySide6).
- Mock ROS node and GUI run together in **their own Docker container**, separate from the vehicle stack; configure GUI display access and ROS networking.
- Never run the real Bridge and Mock Bridge as competing publishers during a test.

## Operating Mode
**Manual only.** No automatic VCU emulation, state transitions, validation, or response to Orchestrator commands. All three input channels are completely independent, including invalid combinations.

## Inputs / ROS Publishers
Three independent channels, each publishing at **10 Hz only while selected**:

| Channel | GUI controls | Behavior |
| --- | --- | --- |
| AS State | AS_OFF, AS_READY, AS_DRIVING, AS_FINISHED, AS_EMERGENCY | One active choice at most |
| Mission | NONE, ACCELERATION, SKIDPAD, AUTOCROSS, TRACKDRIVE | One active choice at most |
| GO | True, False | One active choice at most |

**Common toggle behavior:**
1. At startup, **no channel publishes anything**.
2. Click an inactive value: select it and continuously publish that value at 10 Hz.
3. Click the selected value again: deselect it and **stop publishing that channel entirely**.
4. Click a different value in the same channel: switch the published value immediately; never publish the old value again.
5. Stopping publication does **not** send OFF, NONE, False, an empty message, or a reset value.
6. Changing one channel never affects the other two.

## Subscriber / Feedback
- Subscribe to **one** Orchestrator feedback interface. In this project, "Orchestrator State" and "Override" refer to **the same information**, not separate subscriptions.
- Show the latest received value and timestamp in the GUI.
- Never act on that feedback; **display only**.

## GUI
- Three compact control groups: AS State, Mission, GO.
- Clearly highlight the active button in each group; show **PUBLISHING 10 Hz** or **STOPPED** per channel.
- One read-only field for the last Orchestrator State / Override received.
- Small event log for button selections, publication start/stop, and changed feedback values (do not log every 10 Hz message).
- On closing, stop the publishers cleanly and shut down ROS resources.

## Test Scenarios
1. Launch GUI: verify zero input messages until first click.
2. Select AS_OFF: verify continuous AS_OFF; click it again: verify **no further AS State messages**.
3. Select AS_READY and then AS_DRIVING: verify value switches without resetting GO or Mission.
4. Select GO=True, then GO=False, then deselect: verify continuous True, continuous False, then silence.
5. Change Mission while AS_READY / AS_DRIVING: observe the Orchestrator's response, including the emergency behavior defined in its specification.
6. Pause only one input through deselection; verify the other active channels keep publishing.
7. Verify received Orchestrator State / Override appears in GUI but never modifies any input.

## Implementation Steps
1. Implement a small state model: per channel, `selected_value: Optional[value]` (`None` means no publishing).
2. Implement ROS publishers with a 100 ms timer, publishing only selected channels; add the single read-only feedback subscriber.
3. Implement GUI buttons and event log; keep ROS callbacks off the GUI thread (QThread or equivalent safe integration).
4. Package in one Docker image with launch scripts and display/network configuration.
5. Validate against the real Orchestrator and add a minimal README with build/run/test instructions.

## Deferred — Must Match the Real Bridge
- **Exact ROS topic names, message types, field mappings, and QoS** for all three publishers and the single feedback subscriber.
- Definitive enum/value lists for AS State and Mission (the values above are provisional labels).
- Exact Orchestrator feedback message structure and display formatting.

**Do not invent final ROS interfaces or modify the Orchestrator to accommodate the mock.** Resolve the deferred items from the real Bridge / Orchestrator source code before wiring the integration.
