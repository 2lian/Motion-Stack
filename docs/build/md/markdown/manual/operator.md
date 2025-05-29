# Operator Guide

Welcome to the **Motion-Stack Operator** — a simple, text-based interface for interactively driving your robot’s legs, joints, wheels, and inverse-kinematics via ROS 2. This guide will walk you through launching, and basic usage of the TUI (textual user interface) and the keyboard inputs.

---

## Quick Start

1. **Launch the operator**
   From the `~/Motion-Stack/` directory:
   ```bash
   bash launch_operator.bash
   ```
2. **Enjoy the TUI**
   You’ll see a simple menu. Use your keyboard (or controller) to pick modes and drive your robot in real time.

---

## TUI Overview

When you launch the operator, the screen is divided into:

- **Header**:
  - **Mode** shows your current menu (`main`, `leg_select`, `joint_select`, etc.).
  - **Legs** shows which legs are selected (green) vs. unselected (red).

- **Body**: the menu or controls for the current mode.

- **Footer (“Logs”)**: the last few status messages (leg discovery, errors, recover/halt calls, etc.).

---

## Menu Walkthrough

### Main Menu

- **Leg Selection** — pick which leg(s) you want to control.
- **Joint Selection** — choose individual joints (and direction) for direct velocity control.
- **Wheel Selection** — choose wheel-mode (continuous turn) joints for linear/angular drive.
- **IK Selection** — choose legs with IK available for Cartesian control.
- **Recover / Halt** — send robot-specific “recover” or “halt” commands to selected legs.
- **Quit** — exit the operator cleanly.

_Navigate with the arrow keys (or mouse), then press **↵ Enter** to select._

### Leg Selection

1. You’ll see a **checkbox** next to each discovered leg (e.g. “leg 1”, “leg 2”).
2. Toggle individual legs on/off with **Space** or by clicking the checkbox.
3. (Optional) Press **✔ Confirm Selection** to apply.
   *Tip*: If you don’t pick any, all legs become selected by default.

### Joint Selection

1. A row of **speed radios** appears at the top: **Low • Med • High**.
   - Choose your desired joint velocity scale.
2. Below, joints are grouped by leg (color-coded).
   - Click a joint’s tri-state box:
     - **[ ]** = off
     - **[X]** = forward (left click)
     - **[R]** = reverse (right click)
3. You can select all joints of the specific leg just by clicking the checkbox next to the "Leg N".
4. Clearing or going back is as simple as the **✖ Clear All** or **← Back** buttons.
5. Joint-owned joints are **grayed out** in the Wheel pane.

### Wheel Selection

1. Same pattern as Joint Selection, but for wheel joints.
2. Choose your wheel-speed scale from the top radios.
3. You can select all joints of the specific leg just by clicking the checkbox next to the "Leg N".
4. Wheel-owned joints are **grayed out** in the Joint pane.

### IK Selection

1. Legs with a working IK solver show up as checkboxes.
2. Disabled legs read “Leg N (No IK)”.
3. Choose one or more legs, then drive their end-effector with your controller.

---

## Controls & Keybindings

> **Important**: When you launch the operator, a secondary ROS 2 “keyboard node” window (small red box) opens alongside the TUI. That window captures key events. Make sure it has focus when you press **w**, **s**, **o**, etc. The TUI window itself handles menu navigation (arrow keys, Enter, Space) and mouse clicks.

### TUI Navigation

| Key / Mouse           | Action                                           |
|-----------------------|--------------------------------------------------|
| **↑ / ↓**             | Move focus up/down through menus or lists        |
| **← / →**             | Move focus left/right between columns in grids   |
| **Enter / Space**     | Activate a button or toggle a checkbox           |
| **Mouse Left-Click**  | Click buttons / toggle checkboxes                |
| **Mouse Right-Click** | Toggle **reverse** for joints in joint and wheel modes|

### Drive Mode Keybindings

> **Ensure the keyboard ROS 2 node window is active for these!**

| Mode           | Keys               | Description                                |
|----------------|--------------------|--------------------------------------------|
| **Leg Select** | ↓ or **l**         | Select **all** legs               |
|                | 1–9                | Select that numbered leg                   |
| **Joint Mode** and **Wheel Mode**   | **w** / **s**      | Positive / negative joint velocity |
|                | **0**              | Send selected joints to zero position      |
|                | **o** / **l**      | Wheels forward / backward                  |
|                | **p**              | Stop all wheels                            |
| **IK Mode**    | Gamepad sticks & triggers | Cartesian end-effector control    |
|                | **x** / **o**      | Toggle IK frame: base-relative / ee-relative |

### Recover & Halt Commands

> **Ensure the keyboard ROS 2 node window is active for these!**

| Key                 | Action                         |
|---------------------|--------------------------------|
| **Enter**           | Recover **selected** legs      |
| **Shift + Enter**   | Recover **all** legs           |
| **Space**           | Halt **selected** legs         |
| **Shift + Space**   | Halt **all** legs              |

---

## Customizing & Extensions

- **Robot-specific subclasses**
  Create your own `OperatorNode` in a separate workspace to add:
  - New services (Recover/Halt)
  - Other operational modes
  - Additional keybindings
- **Environment variable**
  Set `OPERATOR` e.g. `export OPERATOR=operator1` to remap input topics.

---

## Troubleshooting

- **No legs discovered?**
  - Check your `/legN/joint_alive` services are running.
- **Buttons unresponsive**
  - Ensure your keyboard (or joystick) topic namespace matches `OPERATOR`.

---
