"""Shared event format for the controller and read-only console monitors."""

from datetime import datetime

COMPONENTS = ("ARMS", "GRIP", "RECORD")


def event_line(component, level, state, event, detail=""):
    line = f"[{datetime.now():%H:%M:%S}] [{component}] [{level}] STATE={state} | EVENT={event}"
    return line + (f" | DETAIL={' '.join(str(detail).split())}" if detail else "")
