#!/usr/bin/env python3
from typing import Any, Dict, List, Optional, Tuple

import urwid

from .operator_node import OperatorNode


class TriStateCheckbox(urwid.CheckBox):
    states = {
        False: urwid.SelectableIcon("[ ]", 1),
        True: urwid.SelectableIcon("[X]", 1),
        "mixed": urwid.SelectableIcon("[R]", 1),
    }

    def mouse_event(self, size, event, button, x, y, focus):
        # right‐click toggles mixed ↔ False
        if event == "mouse press" and button == 3:
            current = self.get_state()
            new = "mixed" if self.state != "mixed" and current is not True else False
            self.state = new
            self._invalidate()
            self._emit("change", self, self.state)
            return True
        return super().mouse_event(size, event, button, x, y, focus)


class OperatorTUI:
    """
    A modular Urwid-based TUI for OperatorNode.
    All menus are driven by `self.menu_definitions`, so subclassing or editing
    `menu_definitions` is all you need to add/remove modes or items.
    """

    LEG_COLORS = [
        "dark red",
        "dark green",
        "brown",
        "dark blue",
        "dark magenta",
        "dark cyan",
        "light gray",
        "light red",
        "light green",
        "yellow",
        "light blue",
        "light magenta",
        "light cyan",
        "white",
    ]

    def __init__(self, node: OperatorNode):
        self.node = node

        # ───────────── HEADER / FOOTER ──────────────────────────
        self.mode_header = urwid.Text("", align="left")
        self.legs_header = urwid.Text("", align="right")
        header = urwid.Columns(
            [
                ("weight", 1, self.mode_header),
                ("weight", 1, self.legs_header),
            ],
            dividechars=1,
        )

        self.body = urwid.SimpleFocusListWalker([])
        listbox = urwid.ListBox(self.body)
        self.log_widget = urwid.Text("", align="left")
        log_box = urwid.LineBox(self.log_widget, title=" Logs ")

        self.frame = urwid.Frame(header=header, body=listbox, footer=log_box)

        # ───────────── STATE CACHING ───────────────────────────
        self.state: Dict[str, Any] = {
            "mode": None,  # last mode string
            "leg_list": [],  # last node.current_legs
            "selected_legs": [],  # last node.selected_legs
            "selected_joints": [],  # last node.selected_joints
            "selected_wheel_joints": [],  # last node.selected_wheel_joints
        }

        # ───────────── CHECKBOX STORAGE ────────────────────────
        self.leg_checkboxes: Dict[int, urwid.CheckBox] = {}
        self.joint_checkboxes: Dict[Tuple[int, str], urwid.CheckBox] = {}

        # ───────────── PALETTE ─────────────────────────────────
        self.palette: List[Tuple[str, str, str]] = [
            ("disabled", "dark gray", ""),
            ("leg_selected", "dark green", ""),
            ("leg_unselected", "dark red", ""),
        ]
        for i, col in enumerate(self.LEG_COLORS):
            self.palette.append((f"leg{i}", col, ""))
            self.palette.append((f"leg{i}_focus", col, ""))

        # ───────────── MENU DEFINITIONS ────────────────────────
        #
        # `menu_definitions` maps each mode‐name to either:
        #   • a list of (label, callback) for simple button menus, or
        #   • None, which indicates “I’ll implement a rebuild_<mode>() method manually.”
        #
        # The keys of `menu_definitions` must match .current_mode in OperatorNode,
        # and if the value is a list, this class will auto‐build a numbered list
        # of buttons for you. If the value is None, it falls back to a `rebuild_<mode>()`
        # method that you must implement below.
        #
        # Add/remove modes by editing this dict. For example, to drop “Recover” from the main
        # screen, simply delete that tuple from `main`’s list.
        #
        self.menu_definitions: Dict[str, Optional[List[Tuple[str, Any]]]] = {
            "main": [
                ("Leg Selection", lambda b: self.node.enter_leg_mode()),
                ("Joint Selection", lambda b: self.node.enter_joint_mode()),
                ("Wheel Selection", lambda b: self.node.enter_wheel_mode()),
                ("IK Selection", lambda b: self.node.enter_ik_mode()),
                ("Recover [NOT IMPLEMENTED]", lambda b: self.node.recover()),
                ("Halt [NOT IMPLEMENTED]", lambda b: self.node.halt()),
                ("Refresh Screen", lambda b: self.clear_screen()),
            ],
            "leg_select": None,
            "joint_select": None,
            "wheel_select": None,
            "ik_select": None,
        }

        # ───────────── MAINLOOP ────────────────────────────────
        self.loop = urwid.MainLoop(
            self.frame, self.palette, unhandled_input=self.on_input
        )

    def clear_screen(self):
        self.body.clear()
        self.state["mode"] = None

    def on_input(self, key):
        pass

    def run(self):
        self.loop.set_alarm_in(0, self._refresh)
        self.loop.run()

    def _refresh(self, loop, _data):
        mode = self.node.current_mode
        self.mode_header.set_text(f"Mode ▶ {mode}    (Ctrl + C to quit) 🦍")

        # display selected legs on the right
        leg_marks = []
        for leg in sorted(self.node.current_legs):
            attr = (
                "leg_selected" if leg in self.node.selected_legs else "leg_unselected"
            )
            leg_marks.append((attr, str(leg)))
            leg_marks.append(("default", " "))
        self.legs_header.set_text(leg_marks)

        legs = sorted(self.node.current_legs)

        # if mode changed, rebuild that screen
        if mode != self.state["mode"]:
            self.state["mode"] = mode
            self.state["leg_list"] = []
            self.state["selected_legs"] = []
            self.state["selected_joints"] = []
            if (
                mode in self.menu_definitions
                and self.menu_definitions[mode] is not None
            ):
                self._build_simple_menu(self.menu_definitions[mode])  # type: ignore
            else:
                # call rebuild_<mode>() if it exists
                method_name = f"rebuild_{mode}"
                if hasattr(self, method_name):
                    getattr(self, method_name)(legs)
                else:
                    self.body.clear()
            loop.draw_screen()

        # if in leg_select, rebuild if legs changed
        elif mode == "leg_select":
            if legs != self.state.get("leg_list", []):
                self.state["leg_list"] = legs
                self.rebuild_leg_select(legs)
                loop.draw_screen()
            else:
                sel = sorted(self.node.selected_legs)
                if sel != self.state["selected_legs"]:
                    self.state["selected_legs"] = sel
                    for leg, cb in self.leg_checkboxes.items():
                        cb.set_state(leg in sel)
                    loop.draw_screen()

        # if in joint_select, rebuild if legs changed
        elif mode == "joint_select":
            if legs != self.state.get("leg_list", []):
                self.state["leg_list"] = legs
                self.rebuild_joint_select(legs)
                loop.draw_screen()
            else:
                current = sorted(self.node.selected_joints) + sorted(
                    self.node.selected_joints_inv
                )
                if current != self.state["selected_joints"]:
                    self.state["selected_joints"] = current
                    # update each tri-state checkbox
                    for (leg, jn), cb in self.joint_checkboxes.items():
                        if (leg, jn) in self.node.selected_joints:
                            cb.set_state(True)
                        elif (leg, jn) in self.node.selected_joints_inv:
                            cb.set_state("mixed")
                        else:
                            cb.set_state(False)
                    loop.draw_screen()

        # if in wheel_select, rebuild if legs changed
        elif mode == "wheel_select":
            if legs != self.state.get("leg_list", []):
                self.state["leg_list"] = legs
                self.rebuild_wheel_select(legs)
            else:
                current = sorted(self.node.selected_wheel_joints) + sorted(
                    self.node.selected_wheel_joints_inv
                )
                if current != self.state["selected_wheel_joints"]:
                    self.state["selected_wheel_joints"] = current
                    # update each tri-state checkbox
                    for (leg, jn), cb in self.joint_checkboxes.items():
                        if (leg, jn) in self.node.selected_wheel_joints:
                            cb.set_state(True)
                        elif (leg, jn) in self.node.selected_wheel_joints_inv:
                            cb.set_state("mixed")
                        else:
                            cb.set_state(False)
                    loop.draw_screen()

        # if in ik_select, rebuild if legs changed
        elif mode == "ik_select":
            if legs != self.state.get("leg_list", []):
                self.state["leg_list"] = legs
                self.rebuild_ik_select(legs)

        # update logs and redraw
        self.log_widget.set_text("\n".join(self.node.log_messages))
        self.loop.draw_screen()
        loop.set_alarm_in(0.5, self._refresh)

    # ─────────────── SIMPLE BUTTON MENU BUILDER ─────────────────

    def _build_simple_menu(self, items: List[Tuple[str, Any]]):
        """
        Given a list of (label, callback), clear body and build a column of buttons.
        """
        self.body.clear()
        for label, cb in items:
            btn = urwid.Button(label)
            urwid.connect_signal(btn, "click", cb)
            self.body.append(urwid.AttrMap(btn, None, focus_map="reversed"))

    # ─────────────── LEG SELECT ──────────────────────────────────

    def rebuild_leg_select(self, legs: List[int]):
        self.body.clear()
        self.leg_checkboxes.clear()

        for leg in legs:
            state = leg in self.node.selected_legs
            cb = urwid.CheckBox(f" leg {leg}", state=state)

            def on_leg_change(cb, new, leg=leg):
                if new:
                    # add leg
                    if leg not in self.node.selected_legs:
                        self.node.selected_legs.append(leg)
                else:
                    # remove leg
                    if leg in self.node.selected_legs:
                        self.node.selected_legs.remove(leg)
                self.node.add_log(
                    "I", f"Selected leg(s): {sorted(self.node.selected_legs)}"
                )
                # force a repaint
                self.loop.draw_screen()

            urwid.connect_signal(cb, "change", on_leg_change)
            self.leg_checkboxes[leg] = cb
            self.body.append(urwid.AttrMap(cb, None, focus_map="reversed"))

        self.body.append(urwid.Divider())
        clear_btn = urwid.Button("✖ Clear All")

        def on_clear_all(_):
            self.node.selected_legs.clear()
            for c in self.leg_checkboxes.values():
                c.set_state(False)
            self.node.add_log("I", "Cleared all leg selections")
            self.loop.draw_screen()

        urwid.connect_signal(clear_btn, "click", on_clear_all)
        self.body.append(urwid.AttrMap(clear_btn, None, focus_map="reversed"))

        self.body.append(urwid.Divider())
        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: self.node.enter_main_menu())
        self.body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    # ─────────────── JOINT SELECT ────────────────────────────────

    def rebuild_joint_select(self, _: List[int]):
        self.body.clear()
        self.joint_checkboxes.clear()

        # ─── Joint & Wheel speed selectors ─────────────────────────
        joint_levels = [("Low", 0.05), ("Med", 0.15), ("High", 0.3)]
        wheel_levels = [("Low", 0.1), ("Med", 0.2), ("High", 0.5)]

        def make_speed_box(levels, attr_name: str, title: str):
            group = []
            buttons = []
            for lbl, val in levels:
                rb = urwid.RadioButton(
                    group, lbl, state=(getattr(self.node, attr_name) == val)
                )
                urwid.connect_signal(
                    rb,
                    "change",
                    lambda btn, new, v=val, a=attr_name: (
                        setattr(self.node, a, v) if new else None
                    ),
                )
                buttons.append(rb)
            grid = urwid.GridFlow(
                buttons,
                cell_width=max(len(l) for l, _ in levels) + 4,
                h_sep=1,
                v_sep=0,
                align="center",
            )
            return urwid.LineBox(
                grid,
                title=title,
                tlcorner="┌",
                tline="─",
                lline="│",
                trcorner="┐",
                blcorner="└",
                rline="│",
                bline="─",
                brcorner="┘",
            )

        joint_box = make_speed_box(joint_levels, "joint_speed", " Joint Speed ")
        wheel_box = make_speed_box(wheel_levels, "wheel_speed", " Wheel Speed ")

        speed_row = urwid.Columns(
            [
                ("weight", 1, urwid.Padding(joint_box, left=1, right=1)),
                ("weight", 1, urwid.Padding(wheel_box, left=1, right=1)),
            ],
            dividechars=2,
        )
        self.body.append(speed_row)
        self.body.append(urwid.Divider())

        ready_handlers = [
            jh
            for jh in self.node.joint_handlers
            if jh.limb_number in self.node.selected_legs and jh.ready.done()
        ]
        if not ready_handlers:
            self.body.append(urwid.Text("No ready legs to show — wait for joint data."))
            back = urwid.Button("← Back to Main")
            urwid.connect_signal(back, "click", lambda b: self.node.enter_main_menu())
            self.body.append(urwid.AttrMap(back, None, focus_map="reversed"))
            return

        CHUNK_SIZE = 3
        for idx, jh in enumerate(sorted(ready_handlers, key=lambda j: j.limb_number)):
            leg = jh.limb_number
            base_attr = f"leg{idx % len(self.LEG_COLORS)}"
            focus_attr = f"{base_attr}_focus"

            joint_list = sorted(jh.tracked)
            n_cols = (len(joint_list) + CHUNK_SIZE - 1) // CHUNK_SIZE

            cols: List[Any] = []

            wheel_claims = (
                self.node.selected_wheel_joints | self.node.selected_wheel_joints_inv
            )
            free_jns = [jn for jn in jh.tracked if (leg, jn) not in wheel_claims]

            if not free_jns:
                leg_widget = urwid.AttrMap(
                    urwid.Text(f"Leg {leg}", wrap="clip"),
                    "disabled",
                )
            else:
                initial = all((leg, jn) in self.node.selected_joints for jn in free_jns)
                cb_leg = urwid.CheckBox(f"Leg {leg}", state=initial)

                def on_leg_cb_change(cb, new, leg=leg, free_jns=free_jns):
                    for jn in free_jns:
                        key = (leg, jn)
                        if key in self.joint_checkboxes:
                            self.joint_checkboxes[key].set_state(new)

                urwid.connect_signal(cb_leg, "change", on_leg_cb_change)
                leg_widget = urwid.AttrMap(cb_leg, base_attr, focus_map=focus_attr)

            cols.append(("fixed", 12, leg_widget))

            for c in range(n_cols):
                pile_items = []
                for r in range(CHUNK_SIZE):
                    i = c * CHUNK_SIZE + r
                    if i < len(joint_list):
                        jn = joint_list[i]
                        if (leg, jn) in wheel_claims:
                            lbl = urwid.Text(f"{jn} (W)")
                            pile_items.append(urwid.AttrMap(lbl, "disabled"))
                        else:
                            init = (
                                True
                                if (leg, jn) in self.node.selected_joints
                                else (
                                    "mixed"
                                    if (leg, jn) in self.node.selected_joints_inv
                                    else False
                                )
                            )
                            cb = TriStateCheckbox(jn, state=init)

                            def on_state_change(cb, new, leg=leg, jn=jn):
                                if new is True:
                                    self.node.selected_joints.add((leg, jn))
                                    self.node.selected_joints_inv.discard((leg, jn))
                                elif new == "mixed":
                                    self.node.selected_joints_inv.add((leg, jn))
                                    self.node.selected_joints.discard((leg, jn))
                                else:
                                    self.node.selected_joints.discard((leg, jn))
                                    self.node.selected_joints_inv.discard((leg, jn))
                                self.node.add_log(
                                    "I",
                                    f"Joints: {sorted(self.node.selected_joints)} "
                                    f"Inv: {sorted(self.node.selected_joints_inv)}",
                                )

                            urwid.connect_signal(cb, "change", on_state_change)
                            self.joint_checkboxes[(leg, jn)] = cb
                            pile_items.append(
                                urwid.AttrMap(cb, base_attr, focus_map=focus_attr)
                            )
                    else:
                        pile_items.append(urwid.Text(""))

                pile = urwid.Pile(pile_items)
                cols.append(("weight", 1, pile))

            self.body.append(urwid.Columns(cols, dividechars=1))
            self.body.append(urwid.Divider())

        clear = urwid.Button("✖ Clear All")

        def on_clear_all(_):
            self.node.selected_joints.clear()
            self.node.selected_joints_inv.clear()
            for cb in self.joint_checkboxes.values():
                cb.set_state(False)

        urwid.connect_signal(clear, "click", on_clear_all)
        self.body.append(urwid.AttrMap(clear, None, focus_map="reversed"))

        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: self.node.enter_main_menu())
        self.body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    # ─────────────── WHEEL SELECT ────────────────────────────────

    def rebuild_wheel_select(self, _: List[int]):
        self.body.clear()
        self.joint_checkboxes.clear()

        joint_levels = [("Low", 0.05), ("Med", 0.15), ("High", 0.3)]
        wheel_levels = [("Low", 0.1), ("Med", 0.2), ("High", 0.5)]

        def make_speed_box(levels, attr_name: str, title: str):
            group = []
            buttons = []
            for lbl, val in levels:
                rb = urwid.RadioButton(
                    group, lbl, state=(getattr(self.node, attr_name) == val)
                )
                urwid.connect_signal(
                    rb,
                    "change",
                    lambda btn, new, v=val, a=attr_name: (
                        setattr(self.node, a, v) if new else None
                    ),
                )
                buttons.append(rb)
            grid = urwid.GridFlow(
                buttons,
                cell_width=max(len(l) for l, _ in levels) + 4,
                h_sep=1,
                v_sep=0,
                align="center",
            )
            return urwid.LineBox(
                grid,
                title=title,
                tlcorner="┌",
                tline="─",
                lline="│",
                trcorner="┐",
                blcorner="└",
                rline="│",
                bline="─",
                brcorner="┘",
            )

        joint_box = make_speed_box(joint_levels, "joint_speed", " Joint Speed ")
        wheel_box = make_speed_box(wheel_levels, "wheel_speed", " Wheel Speed ")

        speed_row = urwid.Columns(
            [
                ("weight", 1, urwid.Padding(joint_box, left=1, right=1)),
                ("weight", 1, urwid.Padding(wheel_box, left=1, right=1)),
            ],
            dividechars=2,
        )
        self.body.append(speed_row)
        self.body.append(urwid.Divider())

        ready_handlers = [
            jh
            for jh in self.node.joint_handlers
            if jh.limb_number in self.node.selected_legs and jh.ready.done()
        ]
        if not ready_handlers:
            self.body.append(urwid.Text("No ready legs to show — wait for joint data."))
            back = urwid.Button("← Back to Main")
            urwid.connect_signal(back, "click", lambda b: self.node.enter_main_menu())
            self.body.append(urwid.AttrMap(back, None, focus_map="reversed"))
            return

        CHUNK_SIZE = 3
        for idx, jh in enumerate(sorted(ready_handlers, key=lambda j: j.limb_number)):
            leg = jh.limb_number
            base_attr = f"leg{idx % len(self.LEG_COLORS)}"
            focus_attr = f"{base_attr}_focus"

            joint_list = sorted(jh.tracked)
            n_cols = (len(joint_list) + CHUNK_SIZE - 1) // CHUNK_SIZE

            cols: List[Any] = []

            joint_claims = self.node.selected_joints | self.node.selected_joints_inv
            free_jns = [jn for jn in jh.tracked if (leg, jn) not in joint_claims]

            if not free_jns:
                leg_widget = urwid.AttrMap(
                    urwid.Text(f"Leg {leg}", wrap="clip"),
                    "disabled",
                )
            else:
                initial = all(
                    (leg, jn) in self.node.selected_wheel_joints for jn in free_jns
                )
                cb_leg = urwid.CheckBox(f"Leg {leg}", state=initial)

                def on_leg_cb_change(cb, new, leg=leg, free_jns=free_jns):
                    for jn in free_jns:
                        key = (leg, jn)
                        if key in self.joint_checkboxes:
                            self.joint_checkboxes[key].set_state(new)

                urwid.connect_signal(cb_leg, "change", on_leg_cb_change)
                leg_widget = urwid.AttrMap(cb_leg, base_attr, focus_map=focus_attr)

            cols.append(("fixed", 12, leg_widget))

            for c in range(n_cols):
                pile_items = []
                for r in range(CHUNK_SIZE):
                    i = c * CHUNK_SIZE + r
                    if i < len(joint_list):
                        jn = joint_list[i]
                        if (leg, jn) in joint_claims:
                            lbl = urwid.Text(f"{jn} (J)")
                            pile_items.append(urwid.AttrMap(lbl, "disabled"))
                        else:
                            init = (
                                True
                                if (leg, jn) in self.node.selected_wheel_joints
                                else (
                                    "mixed"
                                    if (leg, jn) in self.node.selected_wheel_joints_inv
                                    else False
                                )
                            )
                            cb = TriStateCheckbox(jn, state=init)

                            def on_state_change(cb, new, leg=leg, jn=jn):
                                if new is True:
                                    self.node.selected_wheel_joints.add((leg, jn))
                                    self.node.selected_wheel_joints_inv.discard(
                                        (leg, jn)
                                    )
                                elif new == "mixed":
                                    self.node.selected_wheel_joints_inv.add((leg, jn))
                                    self.node.selected_wheel_joints.discard((leg, jn))
                                else:
                                    self.node.selected_wheel_joints.discard((leg, jn))
                                    self.node.selected_wheel_joints_inv.discard(
                                        (leg, jn)
                                    )
                                self.node.add_log(
                                    "I",
                                    f"Wheel joints: {sorted(self.node.selected_wheel_joints)} "
                                    f"Inv: {sorted(self.node.selected_wheel_joints_inv)}",
                                )

                            urwid.connect_signal(cb, "change", on_state_change)
                            self.joint_checkboxes[(leg, jn)] = cb
                            pile_items.append(
                                urwid.AttrMap(cb, base_attr, focus_map=focus_attr)
                            )
                    else:
                        pile_items.append(urwid.Text(""))

                cols.append(("weight", 1, urwid.Pile(pile_items)))

            self.body.append(urwid.Columns(cols, dividechars=1))
            self.body.append(urwid.Divider())

        clear = urwid.Button("✖ Clear All")

        def on_clear_wheels(_):
            self.node.selected_wheel_joints.clear()
            self.node.selected_wheel_joints_inv.clear()
            for cb in self.joint_checkboxes.values():
                cb.set_state(False)

        urwid.connect_signal(clear, "click", on_clear_wheels)
        self.body.append(urwid.AttrMap(clear, None, focus_map="reversed"))

        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: self.node.enter_main_menu())
        self.body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    # ─────────────── IK SELECT ──────────────────────────────────

    def rebuild_ik_select(self, legs: List[int]):
        self.body.clear()
        self.leg_checkboxes.clear()

        ik_by_leg = {ih.limb_number: ih for ih in self.node.ik_handlers}

        for leg in legs:
            ih = ik_by_leg.get(leg)
            if ih is None or not ih.ready.done():
                lbl = urwid.Text(f"Leg {leg} (No IK)")
                self.body.append(urwid.AttrMap(lbl, "disabled"))
            else:
                state = leg in self.node.selected_ik_legs
                cb = urwid.CheckBox(f"Leg {leg}", state=state)

                def on_ik_change(cb, new, leg=leg):
                    if new:
                        if leg not in self.node.selected_ik_legs:
                            self.node.selected_ik_legs.append(leg)
                    else:
                        if leg in self.node.selected_ik_legs:
                            self.node.selected_ik_legs.remove(leg)
                    self.node.add_log(
                        "I", f"IK legs: {sorted(self.node.selected_ik_legs)}"
                    )
                    self.loop.draw_screen()

                urwid.connect_signal(cb, "change", on_ik_change)
                self.leg_checkboxes[leg] = cb
                self.body.append(urwid.AttrMap(cb, None, focus_map="reversed"))

        self.body.append(urwid.Divider())
        clear_btn = urwid.Button("✖ Clear IK‐Only")

        def on_clear_ik(_):
            self.node.selected_ik_legs.clear()
            for c in self.leg_checkboxes.values():
                c.set_state(False)
            self.node.add_log("I", "Cleared IK leg selections")
            self.loop.draw_screen()

        urwid.connect_signal(clear_btn, "click", on_clear_ik)
        self.body.append(urwid.AttrMap(clear_btn, None, focus_map="reversed"))

        self.body.append(urwid.Divider())
        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: self.node.enter_main_menu())
        self.body.append(urwid.AttrMap(back, None, focus_map="reversed"))
