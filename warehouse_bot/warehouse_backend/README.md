# MARS Warehouse Backend API — Phase 2 foundation

This is a separate HTTP service for product/order workflow. The browser calls this API; it does not use ROS 2 DDS or connect directly to MongoDB. The ROS robot integration will be implemented separately and will use the robot-authenticated API routes.

## Current scope

The customer storefront is available at `http://127.0.0.1:8000/`, the separate cart page at `/cart/`, and tracking and inventory administration are available from the storefront header. Customers do not enter API credentials. Each new order receives one six-digit order ID, which is also used to look up order status. The admin view remains protected by the administrator access key. Picked and delivered confirmations are handled in the SO-101 Leader GUI. Product cards contain text only; QR graphics are generated for warehouse labels and are not product photos.

- MongoDB-backed product catalogue and product creation.
- Order acceptance with stock reservation and idempotency key.
- Per-order task queue record; the robot bridge recovers active tasks after restart and claims queued work without requiring the robot to be at a previous dock/drop waypoint. Nav2 uses the current localization to navigate to the approved Pick pose.
- The leader GUI has a persistent QR scan control that uses Arm camera 2. It detects QR symbols inside the on-screen aiming box, resolves the decoded identifier to the matching catalogue item name, and remains active until stopped. Scanning does not change stock.
- Operator `Picked` endpoint advances the active line without changing stock.
- Operator `Dropped` endpoint only succeeds after all lines were picked and the task reached `at_drop`; it deducts reserved stock and records delivery in one MongoDB transaction.
- Administrators can permanently delete catalogue products at any time. Past order snapshots and inventory transaction records remain available.

The service creates only the three location keys (`dock`, `pick`, `drop`) as `needs_owner_seed` records if absent. It never writes coordinates or overwrites an existing record. The ROS task bridge uses only the approved `warehouse_bot/maps/docking_poses.yaml` points; it does not write database poses.

## Local setup

Python 3.10+ and MongoDB 4.2+ configured as a replica set are required because stock reservation/confirmation spans multiple documents and uses transactions.

Run `python /home/ahmed/ros2_ws/src/warehouse_robot_project/warehouse_bot/warehouse_backend/run_server.py` with the general Python environment active. The script reads `.env`, starts MongoDB if needed, initializes/elects the local replica set, starts Uvicorn, waits for the health endpoint, and opens a browser tab. The active browser receives a new tab when one is already open. MongoDB data is stored persistently in `~/.local/share/mars-warehouse/mongodb`; `MARS_MONGODB_DATABASE` selects the `mars_warehouse` database.

Keep `.env` out of backups shared publicly and never commit real credentials. The API binds to loopback by default. For site deployment, use a private network and a TLS-terminating authenticated reverse proxy; do not expose the API, MongoDB, or ROS DDS directly to the public Internet. Production identity management, account lifecycle and audit policy remain deployment decisions.

## API outline

- `GET /health`
- `GET /api/v1/products` — customer-readable active catalogue.
- `POST /api/v1/products` — admin bearer role.
- `PATCH /api/v1/products/{product_id}` — admin item-name edit.
- `DELETE /api/v1/products/{product_id}` — admin-only permanent catalogue removal; historical order and inventory records remain.
- `POST /api/v1/products/{product_id}/stock-adjustments` — admin inventory adjustment with ledger/idempotency key.
- `POST /api/v1/orders` — customer bearer role, requires `Idempotency-Key`.
- `GET /api/v1/orders/{order_id}` — operator/admin role.
- `POST /store/orders` and `GET /store/orders/{order_number}` — storefront checkout and order status lookup using the six-digit order ID; no API credential is exposed in the browser.
- `POST /api/v1/robot/tasks/claim` — robot bearer role.
- `POST /api/v1/robot/tasks/{order_id}/scan` — legacy robot scan-verification endpoint; the leader GUI camera scan does not call it.
- `POST /api/v1/operator/tasks/{order_id}/picked` — operator/admin role; requires `X-Order-Line-Id` and the task to be at Pick. It advances the task without deducting inventory.
- `GET /api/v1/operator/tasks` — operator/admin role; tasks shown in the SO-101 Leader GUI.
- `POST /api/v1/robot/tasks/{order_id}/state` — robot role; state is constrained to known workflow states and destination to dock/pick/drop.
- `POST /api/v1/operator/tasks/{order_id}/dropped` — operator/admin role.

Role tokens are environment-based development credentials, not a complete user identity system. They must be replaced by managed operator/customer authentication before production. Do not pass an operator-supplied display name as authenticated identity; current inventory records the authorized role only.

## Workflow constraints and known follow-up

The HTTP API remains a development foundation. The six-digit order ID is the sole customer-facing status lookup value. A retry with the same idempotency key returns the existing order ID. Operators use the task panel in the SO-101 Leader GUI to scan item QR tags, confirm each line as picked, and confirm delivery at Drop. QR scanning is independent of the active order queue and does not change stock. Stock is deducted only by the explicit `Dropped` confirmation. Production identity management, map-version binding, event streaming, order cancellation/reservation release, and audit retention still need design.

## Phase 5: opt-in ROS/Nav2 bridge

Build the `warehouse_bot` ROS package, start the warehouse API and MongoDB, launch the rover and Nav2/localization, then run this bridge separately:

```bash
ros2 launch warehouse_bot warehouse_task_bridge.launch.py enable:=true
```

Set `MARS_ROBOT_API_TOKEN` and `MARS_WAREHOUSE_API_URL` in the shell before launch. The bridge reads the robot's current position from `map -> base_footprint` TF when available, but position is advisory and does not gate task claiming or Nav2 goals. It navigates only to the three coordinates in the installed `maps/docking_poses.yaml`. On restart it resumes persisted tasks. A Nav2 failure retries the same destination after a short delay. Collision Monitor and sensor freshness are handled by Nav2; they do not cancel or pause bridge goal retries. At Pick it waits for operator confirmation in the SO-101 Leader GUI (scan is skipped for this test); at Drop it waits for explicit operator confirmation there. The AMCL startup pose in `config/nav2.yaml` is set to the saved dock coordinate, so the robot must start at the dock or be given a corrected initial pose in RViz. The bridge is not included in the combined robot launch and its launch argument defaults to disabled.

This is a development integration: confirm the stored points match the active map and test with wheels lifted or a blocked motor output before enabling motion. Do not expose ROS 2 DDS or MongoDB directly to the Internet.

Before physical integration, add concurrency/idempotency integration tests against a replica-set MongoDB, configure managed identity and TLS, verify task transition authorization, reconcile map/pose identity, and review rate limiting/audit retention.
