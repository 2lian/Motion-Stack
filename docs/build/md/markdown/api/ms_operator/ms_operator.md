# ms_operator package

## Submodules

## ms_operator.operator_node module

### *class* ms_operator.operator_node.TUIHandler(node)

Bases: `Handler`

A logging.Handler that redirects every record into the OperatorNode.log box.

#### emit(record)

Do whatever it takes to actually log the specified logging record.

This version is intended to be implemented by subclasses and so
raises a NotImplementedError.

### *class* ms_operator.operator_node.OperatorNode

Bases: `Node`

#### select_leg(leg_inds)

Pick which legs to control (None ⇒ all). Filters out legs that don’t exist
and logs warnings for any missing.

* **Parameters:**
  **leg_inds** (*List* *[**int* *]*  *|* *None*)

#### update_selections()

#### no_no_leg()

Ensure at least one leg is selected; if none are, auto-select all discovered legs.

#### enter_main_menu()

Switch the TUI into ‘main’ mode and install top-level key bindings
for entering each sub-menu.

#### enter_leg_mode()

Switch into ‘leg_select’ mode; map number keys (including multi-digit entry),
Backspace, Enter, and ‘L’/Down arrow to picking legs.

#### enter_joint_mode()

Switch into ‘joint_select’ mode; selection of joints for Joint Syncer.
Map W/S for joint speed, O/L/P for wheel commands, and 0 to zero selected joints.
Clears the Joint Syncer if it exists.

#### enter_wheel_mode()

Switch into ‘wheel_select’ mode; similar to joint mode but selected joints
are controlled by the Wheel Syncer.
Clears the Wheel Syncer if it exists.

#### enter_ik_mode()

Switch into ‘ik_select’ mode; map joystick axes/buttons to starting
the IK timer and toggling end‐effector vs base‐link reference.
Clears the IK Syncer if it exists.

#### move_joints(speed)

If any joints are selected, send constant-speed commands to the
JointSyncerRos using ‘speed_safe’, flipping sign for inverted joints.
Clears the Joint Syncer if the selected joints are changed.

* **Parameters:**
  **speed** (*float*)

#### move_wheels(v, omega=0.0)

Drive the wheel syncer: linear velocity v plus optional omega for
differential steering; cancel when both are zero.
Clears the Wheel Syncer if the selected joints are changed.

* **Parameters:**
  * **v** (*float*)
  * **omega** (*float*)

#### move_ik()

Fired by a 0.1 s timer when any relevant joystick input is active:
compute Δ-pose from sticks/triggers, assemble per-leg targets
(relative or absolute), and send to the IK syncer. Stops when sticks go idle.
Clears the IK Syncer if the selected legs are changed.

#### move_ik_keyboard(lin=[0.0, 0.0, 0.0], rvec=[0.0, 0.0, 0.0])

Send a small IK velocity update from keyboard input.

* **Parameters:**
  * **lin** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*) – 3-vector [vx, vy, vz] (mm/s) for linear velocity.
  * **rvec** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*) – 3-vector [wx, wy, wz] (rad/s) for angular velocity.

What it does:
: 1. Gathers all ready IK legs in self.selected_ik_legs.
  2. If that set changed, clears the previous syncer to avoid stale targets.
  3. Computes Δtime since last tick.
  4. Wraps (lin, rvec) into a VelPose stamped now (and transforms
     into base_link frame if not end-effector mode).
  5. Calls self.ik_syncer.speed_safe(…) with the per-leg velocities and Δtime.

Called on each IK-related key press or release.

#### move_zero()

Send a one-off lerp command to drive all selected joints to zero position.
Clears the Joint Syncer if the selected joints are changed.

#### switch_ik_mode(val=None)

Toggle or set whether IK deltas are interpreted relative to the end
effector (True) or to the base link (False).

* **Parameters:**
  **val** (*bool* *|* *None*)

#### start_ik_timer()

Helper to (re)start the periodic move_ik timer, ensuring a fresh IK
stream if control resumes after a pause.

#### key_downSUBCBK(msg)

A callback that connects the pressed key from the keyboard upon arrival of the msg.

* **Parameters:**
  **msg** (*Key*)

#### key_upSUBCBK(msg)

A callback that connects the realesed key from the keyboard upon arrival of the msg.

* **Parameters:**
  **msg** (*Key*)

#### joy_pressed(button_name)

Executes for each button that is pressed. Like a callback.

* **Parameters:**
  * **bits** – Should only have one single bit set to 1, for 1 single button
  * **button_name** (*Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]*)

#### joy_released(button_name)

Executes for each button that is released. Like a callback.

* **Parameters:**
  * **bits** – Should only have one single bit set to 1, for 1 single button
  * **button_name** (*Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]*)

#### joySUBCBK(msg)

Processes incomming joy messages.
Converts and stores the received state in self.joy_state .
executes self.joy_pressed and self.joy_released for each button that changed state

* **Parameters:**
  **msg** (*Joy*) – Ros2 Joy message type

#### stop_all_joints()

Utility to clear & cancel both joint and IK syncers’ last futures.

#### add_log(level, msg)

Append a timestamped log entry into the rolling message buffer
for display in the TUI footer.

* **Parameters:**
  * **level** (*str*)
  * **msg** (*str*)

#### recover_all()

#### recover()

#### halt_all()

#### halt()

#### create_main_map()

Build and return the global key map (Escape ⇒ main menu, Space/Return ⇒ stubs, etc.).
These keybinds work always.

* **Return type:**
  `Dict`[`Union`[`Tuple`[`int`, `Union`[`int`, `Literal`[`'ANY'`]]], `Tuple`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`], `Union`[`int`, `Literal`[`'ANY'`]]], `Literal`[`'ALWAYS'`]], `List`[`Callable`[[], `Any`]]]

#### loop()

Runs at ~30 Hz: execute whichever of the three syncers (joint, IK, wheel)
currently exists.

#### clear_screen()

### ms_operator.operator_node.main()

## ms_operator.operator_tui module

### *class* ms_operator.operator_tui.GorillaSplash(art, palette_cycle=('gorilla0', 'gorilla1', 'gorilla2', 'gorilla1'), amplitude=3, wavelength=8.0, speed=0.6)

Bases: `object`

amplitude: max spaces to shift
wavelength: how many lines per full sine cycle
speed: phase advance per tick (bigger = faster)

* **Parameters:**
  * **art** (*str*)
  * **amplitude** (*int*)
  * **wavelength** (*float*)
  * **speed** (*float*)

#### widget()

#### tick()

### *class* ms_operator.operator_tui.TriStateCheckbox(label: str | tuple[Hashable, str] | list[str | tuple[Hashable, str]], state: bool = False, has_mixed: Literal[False] = False, on_state_change: Callable[[Self, bool, \_T], Any] | None = None, user_data: \_T = ..., checked_symbol: str | None = ...)

### *class* ms_operator.operator_tui.TriStateCheckbox(label: str | tuple[Hashable, str] | list[str | tuple[Hashable, str]], state: bool = False, has_mixed: Literal[False] = False, on_state_change: Callable[[Self, bool], Any] | None = None, user_data: None = None, checked_symbol: str | None = ...)

### *class* ms_operator.operator_tui.TriStateCheckbox(label: str | tuple[Hashable, str] | list[str | tuple[Hashable, str]], state: Literal['mixed'] | bool = False, has_mixed: Literal[True] = True, on_state_change: Callable[[Self, bool | Literal['mixed'], \_T], Any] | None = None, user_data: \_T = ..., checked_symbol: str | None = ...)

### *class* ms_operator.operator_tui.TriStateCheckbox(label: str | tuple[Hashable, str] | list[str | tuple[Hashable, str]], state: Literal['mixed'] | bool = False, has_mixed: Literal[True] = True, on_state_change: Callable[[Self, bool | Literal['mixed']], Any] | None = None, user_data: None = None, checked_symbol: str | None = ...)

Bases: `CheckBox`

* **Parameters:**
  * **label** (*str* *|* *tuple* *[**Hashable* *,* *str* *]*  *|* *list* *[**str* *|* *tuple* *[**Hashable* *,* *str* *]* *]*)
  * **state** (*bool* *|* *Literal* *[* *'mixed'* *]*)
  * **has_mixed** (*Literal* *[**False* *,* *True* *]*)
  * **on_state_change** (*Callable* *[* *[**Self* *,* *bool* *,*  *\_T* *]* *,* *Any* *]*  *|* *Callable* *[* *[**Self* *,* *bool* *]* *,* *Any* *]*  *|* *Callable* *[* *[**Self* *,* *bool* *|* *Literal* *[* *'mixed'* *]* *,*  *\_T* *]* *,* *typing.Any* *]*  *|* *Callable* *[* *[**Self* *,* *bool* *|* *typing.Literal* *[* *'mixed'* *]* *]* *,* *typing.Any* *]*  *|* *None*)
  * **user_data** ( *\_T* *|* *None*)
  * **checked_symbol** (*str* *|* *None*)

#### states *= {'mixed': <SelectableIcon selectable fixed/flow widget '[R]'>, False: <SelectableIcon selectable fixed/flow widget '[ ]'>, True: <SelectableIcon selectable fixed/flow widget '[X]'>}*

**Type:**    `dict`

#### mouse_event(size, event, button, x, y, focus)

Toggle state on button 1 press.

```pycon
>>> size = (20,)
>>> cb = CheckBox("clickme")
>>> cb.state
False
>>> cb.mouse_event(size, 'mouse press', 1, 2, 0, True)
True
>>> cb.state
True
```

### *class* ms_operator.operator_tui.OperatorTUI(node)

Bases: `object`

A modular Urwid-based TUI for OperatorNode.
All menus are driven by self.menu_definitions, so subclassing or editing
menu_definitions is all you need to add/remove modes or items.

* **Parameters:**
  **node** ([*OperatorNode*](#ms_operator.operator_node.OperatorNode))

#### LEG_COLORS *= ['dark red', 'dark green', 'brown', 'dark blue', 'dark magenta', 'dark cyan', 'light gray', 'light red', 'light green', 'yellow', 'light blue', 'light magenta', 'light cyan', 'white']*

**Type:**    `list`

#### clear_screen()

#### on_input(key)

#### run()

#### rebuild_leg_select(legs)

* **Parameters:**
  **legs** (*List* *[**int* *]*)

#### rebuild_joint_select(\_)

* **Parameters:**
  **\_** (*List* *[**int* *]*)

#### rebuild_wheel_select(\_)

* **Parameters:**
  **\_** (*List* *[**int* *]*)

#### rebuild_ik_select(legs)

* **Parameters:**
  **legs** (*List* *[**int* *]*)

## ms_operator.operator_utils module

### *class* ms_operator.operator_utils.JoyState(bits=0, stickR=<factory>, stickL=<factory>, R2=0.0, L2=0.0)

Bases: `object`

* **Parameters:**
  * **bits** (*int*)
  * **stickR** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **stickL** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **R2** (*float*)
  * **L2** (*float*)

#### bits *= 0*

**Type:**    `int`

#### stickR

**Type:**    `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]

#### stickL

**Type:**    `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]

#### R2 *= 0.0*

**Type:**    `float`

#### L2 *= 0.0*

**Type:**    `float`

### ms_operator.operator_utils.msg_to_JoyBits(msg)

Converts a joy msg to a JoyState

* **Return type:**
  [`JoyState`](#ms_operator.operator_utils.JoyState)
* **Parameters:**
  **msg** (*Joy*)

### ms_operator.operator_utils.any_pressed(bits, button_names)

Checks if any button in the list is pressed.

* **Parameters:**
  * **bits** (*int*) – set of joybits to check against
  * **button_names** (*List* *[**Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *]*  *|*  *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]*) – list of button names to check if True
* **Returns:**
  True if any bit corresponding to a button is True.
* **Return type:**
  `bool`

### ms_operator.operator_utils.bits2name(bits)

Converts a bit field to a list of button names

* **Return type:**
  `List`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`]]
* **Parameters:**
  **bits** (*int*)

### ms_operator.operator_utils.one_bit2name(bits)

Converts a bit field with 1 bit to 1, to a single button name

* **Return type:**
  `Optional`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`]]
* **Parameters:**
  **bits** (*int*)

### ms_operator.operator_utils.collapseT_KeyCodeModifier(variable)

Collapses the variable onto a KeyCodeModifier type, or None

* **Returns:**
  None if variable is not a KCM
  The variable as a KCM type-hint if it is a KCM
* **Return type:**
  `Optional`[`Tuple`[`int`, `Union`[`int`, `Literal`[`'ANY'`]]]]
* **Parameters:**
  **variable** (*Any*)

### ms_operator.operator_utils.collapseT_JoyCodeModifier(variable)

Collapses the variable onto a JoyCodeModifier type, or None

* **Returns:**
  None if variable is not a JCM
  The variable as a JCM type-hint if it is a JCM
* **Return type:**
  `Optional`[`Tuple`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`], `Union`[`int`, `Literal`[`'ANY'`]]]]
* **Parameters:**
  **variable** (*Any*)

### ms_operator.operator_utils.remap_onto_any(mapping, input)

runs the input through the INPUTMap as if the key_modifier was any
if it is already, it does not run it.

* **Parameters:**
  * **mapping** (*Dict* *[**Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]* *,*  *~typing.List* *[* *~typing.Callable* *[* *[* *]* *,*  *~typing.Any* *]* *]* *]*)
  * **input** (*Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]*)

### ms_operator.operator_utils.connect_mapping(mapping, input)

Given the user input, executes the corresponding function mapping

* **Parameters:**
  * **mapping** (*Dict* *[**Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]* *,*  *~typing.List* *[* *~typing.Callable* *[* *[* *]* *,*  *~typing.Any* *]* *]* *]*) – Dict of function to execute
  * **input** (*Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]*) – key to the entry to execute

### ms_operator.operator_utils.rel_to_base_link(ik_syncer, offset)

### ms_operator.operator_utils.rel_vel_to_base_link(ik_syncer, vel_offset)

* **Parameters:**
  **vel_offset** (*dict* *[**int* *,* [*VelPose*](../motion_stack/motion_stack.core.utils.md#motion_stack.core.utils.pose.VelPose) *]*)

### ms_operator.operator_utils.delta_time(node, timer_ns)

* **Return type:**
  `Callable`[[], `float`]
