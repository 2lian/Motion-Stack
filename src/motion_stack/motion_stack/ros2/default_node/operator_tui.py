#!/usr/bin/env python3
from typing import Any, Dict, List, Set, Tuple

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
            new = "mixed" if self.state != "mixed" and current != True else False
            self.state = new
            self._invalidate()
            self._emit("change", self, self.state)
            return True
        return super().mouse_event(size, event, button, x, y, focus)


def urwid_main(node: OperatorNode):
    mode_header = urwid.Text("", align="left")
    legs_header = urwid.Text("", align="right")
    header = urwid.Columns(
        [
            ("weight", 1, mode_header),
            ("weight", 1, legs_header),
        ],
        dividechars=1,
    )
    body = urwid.SimpleFocusListWalker([])
    listbox = urwid.ListBox(body)
    log_widget = urwid.Text("", align="left")
    log_box = urwid.LineBox(log_widget, title=" Logs ")
    frame = urwid.Frame(header=header, body=listbox, footer=log_box)

    # keep track of what we've already drawn
    state = {
        "mode": None,  # last mode
        "leg_list": [],  # last sorted(node.current_legs)
        "selected_legs": [],  # last node.selected_legs
    }

    leg_checkboxes: Dict[int, urwid.CheckBox] = {}
    joint_checkboxes: Dict[Tuple[int, str], urwid.CheckBox] = {}

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
    palette = [
        ("disabled", "dark gray", ""),
        ("leg_selected", "dark green", ""),
        ("leg_unselected", "dark red", ""),
    ]
    for i, col in enumerate(LEG_COLORS):
        palette.append((f"leg{i}", col, ""))
        palette.append((f"leg{i}_focus", col, ""))

    def on_input(key):
        if key in ("q", "Q"):
            raise urwid.ExitMainLoop()

    def rebuild_main_menu():
        body.clear()
        for label, cb in [
            ("Leg Selection", lambda b: node.enter_leg_mode()),
            ("Joint Selection", lambda b: node.enter_joint_mode()),
            ("Wheel Selection", lambda b: node.enter_wheel_mode()),
            ("IK Selection", lambda b: node.enter_ik_mode()),
            ("Recover [NOT IMPLEMENTED]", lambda b: node.recover()),
            ("Halt [NOT IMPLEMENTED]", lambda b: node.halt()),
            ("Quit", lambda b: loop.stop()),
        ]:
            btn = urwid.Button(label)
            urwid.connect_signal(btn, "click", cb)
            body.append(urwid.AttrMap(btn, None, focus_map="reversed"))

    def rebuild_leg_menu(legs: List[int]):
        body.clear()
        leg_checkboxes.clear()

        # one checkbox per leg
        for leg in legs:
            # pre-check if this leg is already selected
            state = leg in node.selected_legs
            cb = urwid.CheckBox(f" leg {leg}", state=state)
            leg_checkboxes[leg] = cb
            body.append(urwid.AttrMap(cb, None, focus_map="reversed"))

        body.append(urwid.Divider())

        # Confirm button
        confirm = urwid.Button("✔ Confirm Selection")

        def on_confirm(btn):
            # gather all checked legs
            chosen = [l for l, cb in leg_checkboxes.items() if cb.get_state()]
            # if none chosen, use None => all
            node.select_leg(chosen or None)

        urwid.connect_signal(confirm, "click", on_confirm)
        body.append(urwid.AttrMap(confirm, None, focus_map="reversed"))

        # Clear all button
        clear_btn = urwid.Button("✖ Clear All")

        def on_clear(_):
            node.selected_legs = []
            for cb in leg_checkboxes.values():
                cb.set_state(False)

        urwid.connect_signal(clear_btn, "click", on_clear)
        body.append(urwid.AttrMap(clear_btn, None, focus_map="reversed"))

        # Back button
        body.append(urwid.Divider())
        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
        body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    def rebuild_joint_menu():
        body.clear()
        joint_checkboxes.clear()

        # ──────────────── Speed radio buttons ────────────────────
        speed_levels = [("Low", 0.05), ("Med", 0.2), ("High", 0.5)]
        radio_group = []
        buttons = []
        for label, val in speed_levels:
            rb = urwid.RadioButton(
                radio_group,
                label,
                state=(node.current_speed == val),
            )
            urwid.connect_signal(
                rb,
                "change",
                lambda btn, new, v=val: (
                    setattr(node, "current_speed", v) if new else None
                ),
            )
            buttons.append(rb)
        speed_grid = urwid.GridFlow(
            buttons,
            cell_width=max(len(lbl) for lbl, _ in speed_levels) + 4,
            h_sep=1,
            v_sep=0,
            align="center",
        )
        body.append(speed_grid)
        body.append(urwid.Divider())
        # ─────────────────────────────────────────────────────────────

        ready = [
            jh
            for jh in node.joint_handlers
            if jh.limb_number in node.selected_legs and jh.ready.done()
        ]
        if not ready:
            body.append(urwid.Text("No ready legs to show — wait for joint data."))
            back = urwid.Button("← Back to Main")
            urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
            body.append(urwid.AttrMap(back, None, focus_map="reversed"))
            return

        CHUNK_SIZE = 3
        for idx, jh in enumerate(sorted(ready, key=lambda j: j.limb_number)):
            leg = jh.limb_number
            base_attr = f"leg{idx % len(LEG_COLORS)}"
            focus_attr = f"{base_attr}_focus"

            joint_list = sorted(jh.tracked)
            n_cols = (len(joint_list) + CHUNK_SIZE - 1) // CHUNK_SIZE

            cols: List[Any] = []
            # fixed leg label
            txt = urwid.Text((base_attr, f"Leg {leg}"), wrap="clip")
            cols.append(
                ("fixed", 12, urwid.AttrMap(txt, base_attr, focus_map=focus_attr))
            )

            # one vertical Pile per chunk
            for c in range(n_cols):
                pile_items = []
                for r in range(CHUNK_SIZE):
                    i = c * CHUNK_SIZE + r
                    if i < len(joint_list):
                        jn = joint_list[i]
                        # if wheel owns it, gray out & suffix “(W)”
                        claimed = (
                            node.selected_wheel_joints | node.selected_wheel_joints_inv
                        )
                        if (leg, jn) in claimed:
                            lbl = urwid.Text(f"{jn} (W)")
                            pile_items.append(urwid.AttrMap(lbl, "disabled"))
                        else:
                            # set initial state
                            init = (
                                True
                                if (leg, jn) in node.selected_joints
                                else (
                                    "mixed"
                                    if (leg, jn) in node.selected_joints_inv
                                    else False
                                )
                            )
                            cb = TriStateCheckbox(jn, state=init)

                            # dynamic update callback
                            def on_state_change(cb, new, leg=leg, jn=jn):
                                if new is True:
                                    node.selected_joints.add((leg, jn))
                                    node.selected_joints_inv.discard((leg, jn))
                                elif new == "mixed":
                                    node.selected_joints_inv.add((leg, jn))
                                    node.selected_joints.discard((leg, jn))
                                else:
                                    node.selected_joints.discard((leg, jn))
                                    node.selected_joints_inv.discard((leg, jn))
                                node.add_log(
                                    "I",
                                    f"Joints: {sorted(node.selected_joints)} "
                                    f"Inv: {sorted(node.selected_joints_inv)}",
                                )

                            urwid.connect_signal(cb, "change", on_state_change)
                            joint_checkboxes[(leg, jn)] = cb
                            pile_items.append(
                                urwid.AttrMap(cb, base_attr, focus_map=focus_attr)
                            )
                    else:
                        pile_items.append(urwid.Text(""))

                pile = urwid.Pile(pile_items)
                cols.append(("weight", 1, pile))

            body.append(urwid.Columns(cols, dividechars=1))
            body.append(urwid.Divider())

        # ─── Clear All ───────────────────────────────────────────────
        clear = urwid.Button("✖ Clear All")

        def on_clear(_):
            node.selected_joints.clear()
            node.selected_joints_inv.clear()
            for cb in joint_checkboxes.values():
                cb.set_state(False)

        urwid.connect_signal(clear, "click", on_clear)
        body.append(urwid.AttrMap(clear, None, focus_map="reversed"))

        # ─── Back to main ────────────────────────────────────────────
        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
        body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    def rebuild_wheel_menu():
        body.clear()
        joint_checkboxes.clear()

        # ──────────────── Speed radio buttons ────────────────────
        speed_levels = [("Low", 0.05), ("Med", 0.2), ("High", 0.5)]
        radio_group = []
        buttons = []
        for label, val in speed_levels:
            rb = urwid.RadioButton(
                radio_group,
                label,
                state=(node.current_speed == val),
            )
            urwid.connect_signal(
                rb,
                "change",
                lambda btn, new, v=val: (
                    setattr(node, "current_speed", v) if new else None
                ),
            )
            buttons.append(rb)
        speed_grid = urwid.GridFlow(
            buttons,
            cell_width=max(len(lbl) for lbl, _ in speed_levels) + 4,
            h_sep=1,
            v_sep=0,
            align="center",
        )
        body.append(speed_grid)
        body.append(urwid.Divider())
        # ─────────────────────────────────────────────────────────────

        ready = [
            jh
            for jh in node.joint_handlers
            if jh.limb_number in node.selected_legs and jh.ready
        ]
        if not ready:
            body.append(urwid.Text("No ready legs to show — wait for joint data."))
            back = urwid.Button("← Back to Main")
            urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
            body.append(urwid.AttrMap(back, None, focus_map="reversed"))
            return

        CHUNK_SIZE = 3
        for idx, jh in enumerate(sorted(ready, key=lambda j: j.limb_number)):
            leg = jh.limb_number
            base_attr = f"leg{idx % len(LEG_COLORS)}"
            focus_attr = f"{base_attr}_focus"

            joint_list = sorted(jh.tracked)
            n_cols = (len(joint_list) + CHUNK_SIZE - 1) // CHUNK_SIZE

            cols: List[Any] = []
            # fixed Leg label
            txt = urwid.Text((base_attr, f"Leg {leg}"), wrap="clip")
            cols.append(
                ("fixed", 12, urwid.AttrMap(txt, base_attr, focus_map=focus_attr))
            )

            for c in range(n_cols):
                pile_items = []
                for r in range(CHUNK_SIZE):
                    i = c * CHUNK_SIZE + r
                    if i < len(joint_list):
                        jn = joint_list[i]
                        # if joint menu owns it, gray out & suffix “(J)”
                        claimed = node.selected_joints | node.selected_joints_inv
                        if (leg, jn) in claimed:
                            lbl = urwid.Text(f"{jn} (J)")
                            pile_items.append(urwid.AttrMap(lbl, "disabled"))
                        else:
                            init = (
                                True
                                if (leg, jn) in node.selected_wheel_joints
                                else (
                                    "mixed"
                                    if (leg, jn) in node.selected_wheel_joints_inv
                                    else False
                                )
                            )
                            cb = TriStateCheckbox(jn, state=init)

                            def on_state_change(cb, new, leg=leg, jn=jn):
                                if new is True:
                                    node.selected_wheel_joints.add((leg, jn))
                                    node.selected_wheel_joints_inv.discard((leg, jn))
                                elif new == "mixed":
                                    node.selected_wheel_joints_inv.add((leg, jn))
                                    node.selected_wheel_joints.discard((leg, jn))
                                else:
                                    node.selected_wheel_joints.discard((leg, jn))
                                    node.selected_wheel_joints_inv.discard((leg, jn))
                                node.add_log(
                                    "I",
                                    f"Wheel joints: {sorted(node.selected_wheel_joints)} "
                                    f"Inv: {sorted(node.selected_wheel_joints_inv)}",
                                )

                            urwid.connect_signal(cb, "change", on_state_change)
                            joint_checkboxes[(leg, jn)] = cb
                            pile_items.append(
                                urwid.AttrMap(cb, base_attr, focus_map=focus_attr)
                            )
                    else:
                        pile_items.append(urwid.Text(""))

                cols.append(("weight", 1, urwid.Pile(pile_items)))

            body.append(urwid.Columns(cols, dividechars=1))
            body.append(urwid.Divider())

        # ─── Clear All ─────────────────────────────────────────────
        clear = urwid.Button("✖ Clear All")

        def on_clear(_):
            node.selected_wheel_joints.clear()
            node.selected_wheel_joints_inv.clear()
            for cb in joint_checkboxes.values():
                cb.set_state(False)

        urwid.connect_signal(clear, "click", on_clear)
        body.append(urwid.AttrMap(clear, None, focus_map="reversed"))

        # ─── Back to Main ───────────────────────────────────────────
        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
        body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    def rebuild_ik_menu(legs: List[int]):
        body.clear()
        leg_checkboxes.clear()

        ik_by_leg = {ih.limb_number: ih for ih in node.ik_handlers}

        for leg in legs:
            ih = ik_by_leg.get(leg)
            if ih is None or not ih.ready.done():
                lbl = urwid.Text(f"Leg {leg} (No IK)")
                body.append(urwid.AttrMap(lbl, "disabled"))
            else:
                state = leg in node.selected_legs
                cb = urwid.CheckBox(f"Leg {leg}", state=state)
                leg_checkboxes[leg] = cb
                body.append(urwid.AttrMap(cb, None, focus_map="reversed"))

        body.append(urwid.Divider())

        # Confirm button
        confirm = urwid.Button("✔ Confirm Selection")

        def on_confirm(btn):
            # gather all checked legs
            chosen = [l for l, cb in leg_checkboxes.items() if cb.get_state()]
            # if none chosen, use None => all
            node.select_leg(chosen or None)

        urwid.connect_signal(confirm, "click", on_confirm)
        body.append(urwid.AttrMap(confirm, None, focus_map="reversed"))

        # Clear all button
        clear_btn = urwid.Button("✖ Clear All")

        def on_clear(_):
            node.selected_legs = []
            for cb in leg_checkboxes.values():
                cb.set_state(False)

        urwid.connect_signal(clear_btn, "click", on_clear)
        body.append(urwid.AttrMap(clear_btn, None, focus_map="reversed"))

        # Back button
        body.append(urwid.Divider())
        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
        body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    def refresh(loop, _):
        mode = node.current_mode
        # update header every time
        mode_header.set_text(f"Mode ▶ {mode}    (q to quit) 🦍")

        leg_marks = []
        for leg in sorted(node.current_legs):
            attr = "leg_selected" if leg in node.selected_legs else "leg_unselected"
            leg_marks.append((attr, str(leg)))
            leg_marks.append(("default", " "))

        legs_header.set_text(leg_marks)
        # if the mode changed, rebuild its screen
        if mode != state["mode"]:
            state["mode"] = mode
            state["leg_list"] = []  # reset cached legs
            if mode == "main":
                rebuild_main_menu()
            elif mode == "leg_select":
                rebuild_leg_menu(sorted(node.current_legs))
            elif mode == "joint_select":
                rebuild_joint_menu()
            elif mode == "wheel_select":
                rebuild_wheel_menu()
            elif mode == "ik_select":
                rebuild_ik_menu(sorted(node.current_legs))

        # if in leg_select, but the discovered legs changed, rebuild
        elif mode == "leg_select":
            legs = sorted(node.current_legs)
            if legs != state["leg_list"]:
                state["leg_list"] = legs
                rebuild_leg_menu(legs)

            else:
                sel = sorted(node.selected_legs)
                if sel != state["selected_legs"]:
                    state["selected_legs"] = sel
                    # update each checkbox in place
                    for leg, cb in leg_checkboxes.items():
                        cb.set_state(leg in sel)
                    loop.draw_screen()

        log_widget.set_text("\n".join(node.log_messages))
        loop.draw_screen()
        loop.set_alarm_in(0.5, refresh)

    loop = urwid.MainLoop(frame, palette, unhandled_input=on_input)
    loop.set_alarm_in(0, refresh)
    loop.run()
