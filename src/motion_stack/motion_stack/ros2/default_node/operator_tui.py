#!/usr/bin/env python3
import urwid
from typing import Any, List, Dict, Tuple, Set

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
        [("weight", 1, mode_header), ("weight", 1, legs_header)], dividechars=1
    )
    body = urwid.SimpleFocusListWalker([])
    listbox = urwid.ListBox(body)
    log_widget = urwid.Text("", align="left")
    log_box = urwid.LineBox(log_widget, title=" Logs ")
    frame = urwid.Frame(header=header, body=listbox, footer=log_box)

    # cache
    state = {"mode": None, "leg_list": [], "selected_legs": []}
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
        palette.extend([(f"leg{i}", col, ""), (f"leg{i}_focus", col, "")])

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
            ("Quit", lambda b: loop.stop()),
        ]:
            btn = urwid.Button(label)
            urwid.connect_signal(btn, "click", cb)
            body.append(urwid.AttrMap(btn, None, focus_map="reversed"))

    def rebuild_leg_menu(legs: List[int]):
        body.clear()
        leg_checkboxes.clear()
        for leg in legs:
            cb = urwid.CheckBox(f" leg {leg}", state=(leg in node.selected_legs))
            leg_checkboxes[leg] = cb
            body.append(urwid.AttrMap(cb, None, focus_map="reversed"))
        body.append(urwid.Divider())

        confirm = urwid.Button("✔ Confirm Selection")

        def on_confirm(btn):
            chosen = [l for l, cb in leg_checkboxes.items() if cb.get_state()]
            node.select_leg(chosen or None)

        urwid.connect_signal(confirm, "click", on_confirm)
        body.append(urwid.AttrMap(confirm, None, focus_map="reversed"))

        clear = urwid.Button("✖ Clear All")

        def on_clear(_):
            node.selected_legs.clear()
            for cb in leg_checkboxes.values():
                cb.set_state(False)

        urwid.connect_signal(clear, "click", on_clear)
        body.append(urwid.AttrMap(clear, None, focus_map="reversed"))

        body.append(urwid.Divider())
        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
        body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    def rebuild_joint_menu():
        body.clear()
        joint_checkboxes.clear()
        # ─── speed radios ─────────────────────────────────
        speed_levels = [("Low", 0.05), ("Med", 0.25), ("High", 0.5)]
        grp: List[urwid.RadioButton] = []
        for lbl, v in speed_levels:
            rb = urwid.RadioButton(grp, lbl, state=(node.current_speed == v))
            urwid.connect_signal(
                rb,
                "change",
                lambda b, new, v=v: setattr(node, "current_speed", v) if new else None,
            )
            body.append(rb)
        body.append(urwid.Divider())

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
            base = f"leg{idx % len(LEG_COLORS)}"
            focus = f"{base}_focus"
            joints = sorted(jh.tracked)
            n_cols = (len(joints) + CHUNK_SIZE - 1) // CHUNK_SIZE
            cols: List[Any] = []
            txt = urwid.Text((base, f"Leg {leg}"), wrap="clip")
            cols.append(("fixed", 12, urwid.AttrMap(txt, base, focus_map=focus)))
            for c in range(n_cols):
                pile = []
                for r in range(CHUNK_SIZE):
                    i = c * CHUNK_SIZE + r
                    if i < len(joints):
                        jn = joints[i]
                        claimed = (
                            node.selected_wheel_joints | node.selected_wheel_joints_inv
                        )
                        if (leg, jn) in claimed:
                            lbl = urwid.Text(f"{jn} (W)")
                            pile.append(urwid.AttrMap(lbl, "disabled"))
                        else:
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

                            def on_state(cb, new, leg=leg, jn=jn):
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
                                    f"Joints: {sorted(node.selected_joints)} Inv: {sorted(node.selected_joints_inv)}",
                                )

                            urwid.connect_signal(cb, "change", on_state)
                            joint_checkboxes[(leg, jn)] = cb
                            pile.append(urwid.AttrMap(cb, base, focus_map=focus))
                    else:
                        pile.append(urwid.Text(""))
                cols.append(("weight", 1, urwid.Pile(pile)))
            body.append(urwid.Columns(cols, dividechars=1))
            body.append(urwid.Divider())

        clear = urwid.Button("✖ Clear All")

        def on_clear(_):
            node.selected_joints.clear()
            node.selected_joints_inv.clear()
            for cb in joint_checkboxes.values():
                cb.set_state(False)

        urwid.connect_signal(clear, "click", on_clear)
        body.append(urwid.AttrMap(clear, None, focus_map="reversed"))

        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
        body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    def rebuild_wheel_menu():
        body.clear()
        joint_checkboxes.clear()
        # ─── speed radios ─────────────────────────────────
        speed_levels = [("Low", 0.05), ("Med", 0.25), ("High", 0.5)]
        grp: List[urwid.RadioButton] = []
        for lbl, v in speed_levels:
            rb = urwid.RadioButton(grp, lbl, state=(node.current_speed == v))
            urwid.connect_signal(
                rb,
                "change",
                lambda b, new, v=v: setattr(node, "current_speed", v) if new else None,
            )
            body.append(rb)
        body.append(urwid.Divider())

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
            base = f"leg{idx % len(LEG_COLORS)}"
            focus = f"{base}_focus"
            joints = sorted(jh.tracked)
            n_cols = (len(joints) + CHUNK_SIZE - 1) // CHUNK_SIZE
            cols: List[Any] = []
            txt = urwid.Text((base, f"Leg {leg}"), wrap="clip")
            cols.append(("fixed", 12, urwid.AttrMap(txt, base, focus_map=focus)))
            for c in range(n_cols):
                pile = []
                for r in range(CHUNK_SIZE):
                    i = c * CHUNK_SIZE + r
                    if i < len(joints):
                        jn = joints[i]
                        claimed = node.selected_joints | node.selected_joints_inv
                        if (leg, jn) in claimed:
                            lbl = urwid.Text(f"{jn} (J)")
                            pile.append(urwid.AttrMap(lbl, "disabled"))
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

                            def on_state(cb, new, leg=leg, jn=jn):
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
                                    f"Wheel joints: {sorted(node.selected_wheel_joints)} Inv: {sorted(node.selected_wheel_joints_inv)}",
                                )

                            urwid.connect_signal(cb, "change", on_state)
                            joint_checkboxes[(leg, jn)] = cb
                            pile.append(urwid.AttrMap(cb, base, focus_map=focus))
                    else:
                        pile.append(urwid.Text(""))
                cols.append(("weight", 1, urwid.Pile(pile)))
            body.append(urwid.Columns(cols, dividechars=1))
            body.append(urwid.Divider())

        clear = urwid.Button("✖ Clear All")

        def on_clear(_):
            node.selected_wheel_joints.clear()
            node.selected_wheel_joints_inv.clear()
            for cb in joint_checkboxes.values():
                cb.set_state(False)

        urwid.connect_signal(clear, "click", on_clear)
        body.append(urwid.AttrMap(clear, None, focus_map="reversed"))

        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
        body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    def rebuild_ik_menu(legs: List[int]):
        body.clear()
        # same as rebuild_leg_menu
        rebuild_leg_menu(legs)

    def refresh(loop, _):
        mode = node.current_mode
        mode_header.set_text(f"Mode ▶ {mode}    (q to quit) 🦍")
        marks: List[Tuple[str, str]] = []
        for leg in sorted(node.current_legs):
            attr = "leg_selected" if leg in node.selected_legs else "leg_unselected"
            marks.append((attr, str(leg)))
            marks.append(("default", " "))
        legs_header.set_text(marks)

        if mode != state["mode"]:
            state["mode"] = mode
            state["leg_list"] = []
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
        elif mode == "leg_select":
            legs = sorted(node.current_legs)
            if legs != state["leg_list"]:
                state["leg_list"] = legs
                rebuild_leg_menu(legs)
            else:
                sel = sorted(node.selected_legs)
                if sel != state["selected_legs"]:
                    state["selected_legs"] = sel
                    for leg, cb in leg_checkboxes.items():
                        cb.set_state(leg in sel)
                    loop.draw_screen()

        log_widget.set_text("\n".join(node.log_messages))
        loop.draw_screen()
        loop.set_alarm_in(0.5, refresh)

    loop = urwid.MainLoop(frame, palette, unhandled_input=on_input)
    loop.set_alarm_in(0, refresh)
    loop.run()
