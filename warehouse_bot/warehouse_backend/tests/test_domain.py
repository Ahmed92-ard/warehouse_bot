import unittest
from mars_warehouse_backend.domain import (
    customer_product_payload, positive_quantity, task_transition_allowed, validate_identifier, validate_idempotency_key, validate_location_key,
)

class DomainValidationTests(unittest.TestCase):
    def test_positive_quantity(self):
        self.assertEqual(positive_quantity(3), 3)
        for value in (0, -1, True, 1.5, "2"):
            with self.assertRaises(ValueError):
                positive_quantity(value)

    def test_identifier_is_trimmed_and_bounded(self):
        self.assertEqual(validate_identifier("  SKU-1 "), "SKU-1")
        with self.assertRaises(ValueError):
            validate_identifier("  ")
        with self.assertRaises(ValueError):
            validate_identifier("x" * 257)

    def test_only_existing_location_keys_are_accepted(self):
        for key in ("dock", "pick", "drop"):
            self.assertEqual(validate_location_key(key), key)
        for key in ("aisle-1", "", "Dock"):
            with self.assertRaises(ValueError):
                validate_location_key(key)

    def test_task_transition_gates(self):
        self.assertTrue(task_transition_allowed("navigating_to_pick", "at_pick"))
        self.assertTrue(task_transition_allowed("waiting_for_operator", "assistance_required"))
        self.assertTrue(task_transition_allowed("delivered", "returning_to_dock"))
        self.assertFalse(task_transition_allowed("waiting_for_operator", "at_drop"))
        self.assertFalse(task_transition_allowed("queued", "completed"))

    def test_customer_catalogue_hides_internal_fields(self):
        payload = customer_product_payload({
            "_id": "abc", "sku": "A-1", "name": "Widget", "description": "Part",
            "stock_on_hand": 9, "stock_reserved": 4, "identifier": "PRIVATE-QR",
            "identifier_type": "qr", "version": 12,
        })
        self.assertEqual(payload["available_stock"], 5)
        self.assertNotIn("identifier", payload)
        self.assertNotIn("stock_reserved", payload)
        self.assertNotIn("version", payload)

    def test_idempotency_key_format(self):
        self.assertEqual(validate_idempotency_key("order:abc-123"), "order:abc-123")
        for key in (None, "short", "bad key value", "x" * 129):
            with self.assertRaises(ValueError):
                validate_idempotency_key(key)

if __name__ == "__main__":
    unittest.main()
