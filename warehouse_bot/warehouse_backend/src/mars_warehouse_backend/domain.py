"""Small pure validation helpers shared by the HTTP layer and tests."""
from __future__ import annotations

import re
from typing import Any

LOCATION_KEYS = frozenset({"dock", "pick", "drop"})
ORDER_STATES = frozenset({"queued", "in_progress", "delivered", "cancelled", "assistance_required"})
TASK_TRANSITIONS = {
    "navigating_to_pick": {"at_pick", "assistance_required"},
    "at_pick": {"scanning", "assistance_required"},
    "scanning": {"assistance_required"},
    "waiting_for_operator": {"assistance_required"},
    "navigating_to_drop": {"at_drop", "assistance_required"},
    "at_drop": {"assistance_required"},
    "delivered": {"returning_to_dock"},
    "returning_to_dock": {"completed", "assistance_required"},
    "assistance_required": {"navigating_to_pick", "navigating_to_drop", "returning_to_dock"},
}


def positive_quantity(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("quantity must be a positive integer")
    return value


def validate_identifier(value: str) -> str:
    cleaned = value.strip()
    if not cleaned or len(cleaned) > 256:
        raise ValueError("identifier must contain 1 to 256 characters")
    return cleaned


def validate_location_key(value: str) -> str:
    if value not in LOCATION_KEYS:
        raise ValueError("destination must be one of dock, pick, drop")
    return value


def validate_idempotency_key(value: str | None) -> str:
    if value is None or not re.fullmatch(r"[A-Za-z0-9._:-]{8,128}", value):
        raise ValueError("Idempotency-Key header must be 8-128 safe characters")
    return value


def task_transition_allowed(current: str, target: str) -> bool:
    return target in TASK_TRANSITIONS.get(current, set())


def customer_product_payload(product: dict) -> dict:
    """Limit catalogue responses to fields needed by customers."""
    return {
        "id": str(product["_id"]),
        "sku": product["sku"],
        "name": product["name"],
        "description": product.get("description", ""),
        "available_stock": max(0, product["stock_on_hand"] - product["stock_reserved"]),
    }
