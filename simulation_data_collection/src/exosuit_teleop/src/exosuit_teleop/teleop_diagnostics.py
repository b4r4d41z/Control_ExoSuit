"""Structured state and failure logging for the arm controller."""

from datetime import datetime

def event_line(component, level, state, event, detail=""):
    line = f"[{datetime.now():%H:%M:%S}] [{component}] [{level}] STATE={state} | EVENT={event}"
    return line + (f" | DETAIL={' '.join(str(detail).split())}" if detail else "")
