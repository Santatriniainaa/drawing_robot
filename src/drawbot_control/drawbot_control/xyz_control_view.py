"""Tk view for the Drawbot XYZ controller."""

import tkinter as tk
from tkinter import ttk
from typing import Callable

from drawbot_control.robot_geometry import GEOMETRY


class XYZControlView:
    """Owns widgets and presentation, leaving ROS orchestration to the GUI node."""

    def __init__(
        self,
        root: tk.Tk,
        pen_contact: float,
        callbacks: dict[str, Callable[[], None]],
        on_close: Callable[[], None],
    ) -> None:
        self.root = root
        self.scales: dict[str, tk.Scale] = {}
        self.value_labels: dict[str, ttk.Label] = {}
        self.status = tk.StringVar(value="Attente des contrôleurs…")
        self.root.title("Drawbot — commande XYZ")
        self.root.geometry("700x380")
        self.root.minsize(620, 350)
        self.root.protocol("WM_DELETE_WINDOW", on_close)
        self.root.after(150, self._raise_window_on_start)

        panel = ttk.Frame(root, padding=16)
        panel.pack(fill="both", expand=True)
        ttk.Label(
            panel,
            text="Commande manuelle du robot",
            font=("TkDefaultFont", 14, "bold"),
        ).pack(anchor="w", pady=(0, 2))
        ttk.Label(
            panel,
            text=(
                f"X/Y : ±{max(abs(value) for value in GEOMETRY['limits']['x']):.2f} m · "
                "Z : levé à 0, "
                f"contact estimé à {pen_contact * 1000:.1f} mm"
            ),
        ).pack(anchor="w", pady=(0, 12))

        axes = ttk.Frame(panel)
        axes.pack(fill="x", expand=True)
        self._add_axis(axes, "X", *GEOMETRY["limits"]["x"], row=0)
        self._add_axis(axes, "Y", *GEOMETRY["limits"]["y"], row=1)
        self._add_axis(axes, "Z", *GEOMETRY["limits"]["z"], row=2)

        buttons = ttk.Frame(panel)
        buttons.pack(fill="x", pady=(8, 8))
        self.send_button = ttk.Button(
            buttons, text="Envoyer XYZ", command=callbacks["send"]
        )
        self.send_button.pack(side="left", padx=(0, 6))
        self.center_button = ttk.Button(
            buttons, text="Centre (stylo levé)", command=callbacks["center"]
        )
        self.center_button.pack(side="left", padx=6)
        self.random_button = ttk.Button(
            buttons, text="Aléatoire", command=callbacks["random"]
        )
        self.random_button.pack(side="left", padx=6)
        self.print_button = ttk.Button(
            buttons, text="Print", command=callbacks["print"]
        )
        self.print_button.pack(side="left", padx=6)
        self.cancel_button = ttk.Button(
            buttons, text="Annuler XY", command=callbacks["cancel"]
        )
        self.cancel_button.pack(side="right", padx=(6, 0))
        ttk.Label(panel, textvariable=self.status, wraplength=520).pack(
            anchor="w", pady=(4, 0)
        )

    def _raise_window_on_start(self) -> None:
        """Bring the manual-control window forward once it has been mapped."""
        try:
            self.root.deiconify()
            self.root.attributes("-topmost", True)
            self.root.lift()
            self.root.focus_force()
            self.root.after(
                1200,
                lambda: self.root.attributes("-topmost", False),
            )
        except tk.TclError:
            # The window may be closing while the delayed callback is pending.
            pass

    def _add_axis(
        self,
        parent: ttk.Frame,
        axis: str,
        minimum: float,
        maximum: float,
        row: int,
    ) -> None:
        ttk.Label(
            parent,
            text=axis,
            width=3,
            font=("TkDefaultFont", 11, "bold"),
        ).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=5)
        scale = tk.Scale(
            parent,
            from_=minimum,
            to=maximum,
            resolution=0.001,
            orient="horizontal",
            showvalue=False,
            length=390,
            command=lambda _value, name=axis: self.update_value_label(name),
        )
        scale.set(0.0)
        scale.grid(row=row, column=1, sticky="ew", pady=3)
        label = ttk.Label(parent, text="0.000 m", width=10, anchor="e")
        label.grid(row=row, column=2, sticky="e", padx=(8, 0))
        parent.columnconfigure(1, weight=1)
        self.scales[axis] = scale
        self.value_labels[axis] = label

    def update_value_label(self, axis: str) -> None:
        value = self.scales[axis].get()
        self.value_labels[axis].configure(text=f"{value:+.3f} m")

    def set_controls_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        for button in (
            self.send_button,
            self.center_button,
            self.random_button,
            self.print_button,
        ):
            button.configure(state=state)
        for scale in self.scales.values():
            scale.configure(state=state)
        self.cancel_button.configure(
            text="Annuler XY" if enabled else "Annuler impression"
        )
