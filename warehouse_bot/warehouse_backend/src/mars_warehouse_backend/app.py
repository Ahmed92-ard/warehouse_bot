from __future__ import annotations

import hmac
import os
import secrets
import uuid
from io import BytesIO
from pathlib import Path
from datetime import datetime, timezone
from typing import Annotated, Any

from fastapi import Depends, FastAPI, Header, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from pymongo import MongoClient, ReturnDocument
from pymongo.errors import DuplicateKeyError, PyMongoError

from .domain import customer_product_payload, positive_quantity, task_transition_allowed, validate_identifier, validate_idempotency_key, validate_location_key


def utcnow() -> datetime:
    return datetime.now(timezone.utc)


def new_order_number(db) -> str:
    for _ in range(100):
        number = str(secrets.randbelow(900_000) + 100_000)
        if not db.orders.find_one({"order_number": number}, {"_id": 1}):
            return number
    raise HTTPException(status_code=503, detail="Could not allocate an order number; try again")


def token_role(authorization: str | None) -> str:
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Bearer token required")
    supplied = authorization[7:]
    for role, env_name in (
        ("customer", "MARS_CUSTOMER_API_TOKEN"),
        ("operator", "MARS_OPERATOR_API_TOKEN"),
        ("admin", "MARS_ADMIN_API_TOKEN"),
        ("robot", "MARS_ROBOT_API_TOKEN"),
    ):
        expected = os.getenv(env_name, "")
        if expected and hmac.compare_digest(supplied, expected):
            return role
    raise HTTPException(status_code=401, detail="Invalid bearer token")


def require_roles(*roles: str):
    def dependency(authorization: Annotated[str | None, Header()] = None) -> str:
        role = token_role(authorization)
        if role not in roles:
            raise HTTPException(status_code=403, detail="Insufficient role")
        return role
    return dependency


class ProductInput(BaseModel):
    sku: str | None = Field(default=None, min_length=1, max_length=80)
    name: str = Field(min_length=1, max_length=200)
    description: str = Field(default="", max_length=2000)
    identifier: str = Field(min_length=1, max_length=256)
    identifier_type: str = Field(default="qr", pattern="^(qr|apriltag)$")
    initial_stock: int = Field(ge=0)


class ProductPatch(BaseModel):
    name: str | None = Field(default=None, min_length=1, max_length=200)


class StockAdjustment(BaseModel):
    delta: int
    reason: str = Field(min_length=1, max_length=300)


class OrderLineInput(BaseModel):
    product_id: str = Field(min_length=1, max_length=64)
    quantity: int = Field(gt=0)


class OrderInput(BaseModel):
    items: list[OrderLineInput] = Field(min_length=1, max_length=50)
    customer_reference: str | None = Field(default=None, max_length=120)


class ScanInput(BaseModel):
    line_id: str
    identifier: str
    confidence: float = Field(ge=0.0, le=1.0)


class QRResolveInput(BaseModel):
    identifier: str = Field(min_length=1, max_length=256)


class TaskStateInput(BaseModel):
    state: str = Field(pattern="^(navigating_to_pick|navigating_to_drop|at_pick|scanning|at_drop|assistance_required|returning_to_dock|completed)$")
    destination: str | None = None
    nav_status: str | None = Field(default=None, max_length=100)
    detail: str | None = Field(default=None, max_length=500)


class AppState:
    client: MongoClient | None = None
    db: Any = None


state = AppState()
app = FastAPI(title="MARS Warehouse API", version="0.1.0")
ADMIN_WEB_SESSION_ID = secrets.token_urlsafe(18)
origins = [x.strip() for x in os.getenv("MARS_ALLOWED_ORIGINS", "http://localhost:5173").split(",") if x.strip()]
app.add_middleware(CORSMiddleware, allow_origins=origins, allow_credentials=False,
                   allow_methods=["GET", "POST", "PATCH"], allow_headers=["Authorization", "Content-Type", "Idempotency-Key", "X-Robot-Location"])


@app.on_event("startup")
def startup() -> None:
    uri = os.getenv("MARS_MONGODB_URI")
    if not uri:
        return  # Health reports unavailable; this allows config/help inspection without a secret or DB.
    state.client = MongoClient(uri, serverSelectionTimeoutMS=2500, tz_aware=True)
    state.db = state.client[os.getenv("MARS_MONGODB_DATABASE", "mars_warehouse")]
    state.db.products.create_index("sku", unique=True)
    state.db.products.create_index("identifier", unique=True)
    state.db.orders.create_index("idempotency_key", unique=True)
    state.db.orders.create_index("order_number", unique=True, sparse=True)
    state.db.robot_tasks.create_index("order_id", unique=True)
    state.db.robot_tasks.create_index([("state", 1), ("created_at", 1)])
    state.db.inventory_transactions.create_index("idempotency_key", unique=True)
    state.db.inventory_transactions.create_index([("product_id", 1), ("created_at", -1)])
    state.db.delivery_confirmations.create_index("idempotency_key", unique=True)
    state.db.locations.create_index("key", unique=True)
    # Seed only the three owner-provided locations. Never overwrite existing coordinates.
    for key in ("dock", "pick", "drop"):
        state.db.locations.update_one({"key": key}, {"$setOnInsert": {"key": key, "frame_id": "map", "status": "needs_owner_seed"}}, upsert=True)


@app.on_event("shutdown")
def shutdown() -> None:
    if state.client:
        state.client.close()


def database():
    if state.db is None:
        raise HTTPException(status_code=503, detail="MongoDB is not configured or unavailable")
    try:
        state.db.command("ping")
    except PyMongoError:
        raise HTTPException(status_code=503, detail="MongoDB is unavailable")
    return state.db


def json_safe(value):
    try:
        from bson import ObjectId
        if isinstance(value, ObjectId):
            return str(value)
    except ImportError:
        pass
    if isinstance(value, datetime):
        return value.isoformat()
    if isinstance(value, dict):
        private_fields = {"_id", "tracking_token_hash", "tracking_number", "tracking_format", "idempotency_key"}
        return {str(k): json_safe(v) for k, v in value.items() if k not in private_fields}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def obj(value: dict) -> dict:
    result = json_safe(value)
    result["id"] = str(value.get("_id", ""))
    return result


def run_transaction(db, callback):
    if state.client is None:
        raise HTTPException(status_code=503, detail="MongoDB is not configured")
    try:
        with state.client.start_session() as session:
            return session.with_transaction(lambda s: callback(s))
    except DuplicateKeyError:
        raise
    except PyMongoError as exc:
        raise HTTPException(status_code=503, detail=f"Database transaction failed: {type(exc).__name__}")


@app.get("/health")
def health():
    if state.db is None:
        return {"status": "degraded", "database": "not_configured"}
    try:
        state.db.command("ping")
        return {"status": "ok", "database": "connected"}
    except PyMongoError:
        return {"status": "degraded", "database": "unavailable"}


@app.get("/api/v1/session")
def api_session():
    """Opaque process marker; the browser uses it to reset admin sign-in after relaunch."""
    return {"session_id": ADMIN_WEB_SESSION_ID}


@app.get("/api/v1/products")
def list_products(db=Depends(database)):
    return [customer_product_payload(product)
            for product in db.products.find({}).sort("name", 1)]


@app.get("/api/v1/admin/products")
def admin_products(role=Depends(require_roles("admin")), db=Depends(database)):
    return [obj(p) for p in db.products.find({}).sort("name", 1)]


@app.post("/api/v1/operator/qr/resolve")
def resolve_qr(body: QRResolveInput, role=Depends(require_roles("operator", "admin")),
                db=Depends(database)):
    """Resolve a decoded QR payload to its catalogue item name, without touching stock."""
    try:
        identifier = validate_identifier(body.identifier)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    product = db.products.find_one({"identifier": identifier}, {"_id": 0, "name": 1})
    if product:
        return {"found": True, "name": product["name"]}
    return {"found": False, "name": "Unknown item"}


@app.get("/api/v1/products/{product_id}/inventory-transactions")
def product_transactions(product_id: str, role=Depends(require_roles("admin", "operator")), db=Depends(database)):
    try:
        from bson import ObjectId
        pid = ObjectId(product_id)
    except Exception:
        raise HTTPException(status_code=422, detail="Invalid product id")
    if not db.products.find_one({"_id": pid}, {"_id": 1}):
        raise HTTPException(status_code=404, detail="Product not found")
    return [obj(x) for x in db.inventory_transactions.find({"product_id": pid}).sort("created_at", -1).limit(200)]


@app.get("/api/v1/products/{product_id}/qr.png")
def product_qr(product_id: str, role=Depends(require_roles("admin")), db=Depends(database)):
    try:
        from bson import ObjectId
        import qrcode
        pid = ObjectId(product_id)
    except Exception as exc:
        if "qrcode" in str(exc):
            raise HTTPException(status_code=503, detail="QR generation dependency is not installed")
        raise HTTPException(status_code=422, detail="Invalid product id")
    product = db.products.find_one({"_id": pid}, {"identifier": 1, "sku": 1})
    if not product:
        raise HTTPException(status_code=404, detail="Product not found")
    image = qrcode.make(product["identifier"])
    stream = BytesIO()
    image.save(stream, format="PNG")
    stream.seek(0)
    return StreamingResponse(stream, media_type="image/png", headers={"Content-Disposition": f'inline; filename="{product["sku"]}-qr.png"'})


@app.post("/api/v1/products", status_code=201)
def create_product(body: ProductInput, role=Depends(require_roles("admin")), db=Depends(database)):
    now = utcnow()
    sku = body.sku.strip() if body.sku else f"ITEM-{uuid.uuid4().hex[:10].upper()}"
    doc = {"sku": sku, "name": body.name.strip(), "description": body.description,
           "identifier": validate_identifier(body.identifier), "identifier_type": body.identifier_type,
           "stock_on_hand": body.initial_stock, "stock_reserved": 0, "status": "active",
           "created_at": now, "updated_at": now, "version": 1}
    def insert_product(session):
        result = db.products.insert_one(doc, session=session)
        if body.initial_stock:
            db.inventory_transactions.insert_one({"product_id": result.inserted_id, "type": "initial_stock",
                "quantity_delta": body.initial_stock, "new_stock": body.initial_stock,
                "idempotency_key": f"initial:{result.inserted_id}", "created_at": now}, session=session)
        return db.products.find_one({"_id": result.inserted_id}, session=session)
    try:
        created = run_transaction(db, insert_product)
    except DuplicateKeyError:
        raise HTTPException(status_code=409, detail="SKU or product identifier already exists")
    return obj(created)


@app.patch("/api/v1/products/{product_id}")
def patch_product(product_id: str, body: ProductPatch, role=Depends(require_roles("admin")), db=Depends(database)):
    try:
        from bson import ObjectId
        pid = ObjectId(product_id)
    except Exception:
        raise HTTPException(status_code=422, detail="Invalid product id")
    updates = {k: v for k, v in body.model_dump(exclude_unset=True).items() if v is not None}
    if not updates:
        raise HTTPException(status_code=422, detail="No product fields supplied")
    updates["updated_at"] = utcnow()
    result = db.products.update_one({"_id": pid}, {"$set": updates, "$inc": {"version": 1}})
    if not result.matched_count:
        raise HTTPException(status_code=404, detail="Product not found")
    return obj(db.products.find_one({"_id": pid}))


@app.delete("/api/v1/products/{product_id}")
def delete_product(product_id: str, role=Depends(require_roles("admin")), db=Depends(database)):
    """Permanently remove a product from the catalogue, regardless of stock or history."""
    try:
        from bson import ObjectId
        pid = ObjectId(product_id)
    except Exception:
        raise HTTPException(status_code=422, detail="Invalid product id")
    result = db.products.delete_one({"_id": pid})
    if not result.deleted_count:
        raise HTTPException(status_code=404, detail="Product not found")
    # Order snapshots and stock transactions remain visible as historical records.
    return {"deleted": True, "product_id": product_id}


@app.post("/api/v1/products/{product_id}/stock-adjustments")
def adjust_stock(product_id: str, body: StockAdjustment,
                 idempotency_key: Annotated[str | None, Header(alias="Idempotency-Key")] = None,
                 role=Depends(require_roles("admin")), db=Depends(database)):
    try:
        from bson import ObjectId
        pid = ObjectId(product_id)
        key = validate_idempotency_key(idempotency_key)
        if body.delta == 0:
            raise ValueError("stock adjustment cannot be zero")
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    except Exception:
        raise HTTPException(status_code=422, detail="Invalid product id")
    now = utcnow()
    def apply(session):
        previous = db.inventory_transactions.find_one({"idempotency_key": key}, session=session)
        if previous:
            return {"already_applied": True}
        product = db.products.find_one_and_update({"_id": pid, "$expr": {"$gte": [{"$add": ["$stock_on_hand", body.delta]}, "$stock_reserved"]}},
            {"$inc": {"stock_on_hand": body.delta, "version": 1}, "$set": {"updated_at": now}},
            return_document=ReturnDocument.BEFORE, session=session)
        if not product:
            raise HTTPException(status_code=409, detail="Product not found or adjustment would consume reserved stock")
        db.inventory_transactions.insert_one({"product_id": pid, "type": "adjustment", "quantity_delta": body.delta,
            "previous_stock": product["stock_on_hand"], "new_stock": product["stock_on_hand"] + body.delta,
            "reason": body.reason, "idempotency_key": key, "created_at": now}, session=session)
        return {"already_applied": False, "stock_on_hand": product["stock_on_hand"] + body.delta}
    try:
        return run_transaction(db, apply)
    except DuplicateKeyError:
        return {"already_applied": True}


@app.post("/api/v1/orders", status_code=201)
def create_order(body: OrderInput, idempotency_key: Annotated[str | None, Header(alias="Idempotency-Key")] = None,
                 role=Depends(require_roles("customer")), db=Depends(database)):
    try:
        key = validate_idempotency_key(idempotency_key)
        quantities: dict[str, int] = {}
        for item in body.items:
            positive_quantity(item.quantity)
            quantities[item.product_id] = quantities.get(item.product_id, 0) + item.quantity
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    existing = db.orders.find_one({"idempotency_key": key})
    if existing:
        replay = obj(existing)
        replay.setdefault("order_number", replay.get("id", "")[-6:])
        for line in replay.get("items", []):
            line.pop("identifier", None)
            line.pop("identifier_type", None)
        return replay
    order_id = uuid.uuid4().hex
    order_number = new_order_number(db)
    now = utcnow()
    lines = []
    def reserve(session):
        for product_id, quantity in quantities.items():
            try:
                from bson import ObjectId
                pid = ObjectId(product_id)
            except Exception:
                raise HTTPException(status_code=422, detail=f"Invalid product id: {product_id}")
            product = db.products.find_one_and_update(
                {"_id": pid, "status": "active", "$expr": {"$gte": [{"$subtract": ["$stock_on_hand", "$stock_reserved"]}, quantity]}},
                {"$inc": {"stock_reserved": quantity, "version": 1}, "$set": {"updated_at": now}},
                return_document=ReturnDocument.BEFORE, session=session)
            if product is None:
                raise HTTPException(status_code=409, detail=f"Product unavailable or insufficient stock: {product_id}")
            line_id = uuid.uuid4().hex
            lines.append({"line_id": line_id, "product_id": pid, "sku": product["sku"], "name": product["name"],
                          "identifier": product["identifier"], "identifier_type": product["identifier_type"],
                          "quantity": quantity, "picked": False, "verified": False})
            db.inventory_transactions.insert_one({"product_id": pid, "type": "reserve", "quantity_delta": 0,
                "reservation_delta": quantity, "order_id": order_id, "idempotency_key": f"reserve:{order_id}:{line_id}", "created_at": now}, session=session)
        order = {"_id": order_id, "order_number": order_number,
                 "idempotency_key": key,
                 "customer_reference": body.customer_reference, "items": lines, "status": "queued",
                 "created_at": now, "updated_at": now}
        db.orders.insert_one(order, session=session)
        db.robot_tasks.insert_one({"order_id": order_id, "state": "queued", "current_destination": "dock",
            "current_line_index": 0, "created_at": now, "updated_at": now, "version": 1}, session=session)
        return order
    try:
        order = run_transaction(db, reserve)
    except DuplicateKeyError:
        order = db.orders.find_one({"idempotency_key": key})
        if not order:
            raise HTTPException(status_code=409, detail="Duplicate order submission")
    response = obj(order)
    for line in response.get("items", []):
        line.pop("identifier", None)
        line.pop("identifier_type", None)
    return response


@app.post("/store/orders", status_code=201)
def storefront_create_order(body: OrderInput,
                            idempotency_key: Annotated[str | None, Header(alias="Idempotency-Key")] = None,
                            db=Depends(database)):
    """Public storefront checkout; the six-digit order ID is returned for status lookup."""
    result = create_order(body, idempotency_key, "customer", db)
    result["id"] = result.get("order_number", result.get("id", ""))
    return result


def order_status_payload(db, order: dict) -> dict:
    order_id = str(order["_id"])
    result = obj(order)
    for line in result.get("items", []):
        line.pop("identifier", None)
        line.pop("identifier_type", None)
    task = db.robot_tasks.find_one({"order_id": order_id})
    result["task"] = ({"state": task.get("state"), "current_destination": task.get("current_destination"),
                       "updated_at": task.get("updated_at").isoformat() if task.get("updated_at") else None}
                      if task else None)
    return result


@app.get("/api/v1/orders/{order_id}")
def get_order(order_id: str, role=Depends(require_roles("operator", "admin")), db=Depends(database)):
    order = db.orders.find_one({"_id": order_id})
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")
    return order_status_payload(db, order)


@app.get("/api/v1/operator/tasks")
def list_operator_tasks(role=Depends(require_roles("operator", "admin")), db=Depends(database)):
    """Return recent order tasks for the separate operator page."""
    tasks = db.robot_tasks.find({}).sort("updated_at", -1).limit(100)
    result = []
    for task in tasks:
        order = db.orders.find_one({"_id": task["order_id"]})
        if not order:
            continue
        result.append({
            "order_id": str(order["_id"]),
            "order_number": order.get("order_number", ""),
            "customer_reference": order.get("customer_reference"),
            "order_status": order.get("status"),
            "created_at": order.get("created_at").isoformat() if order.get("created_at") else None,
            "task": {
                "state": task.get("state"),
                "current_destination": task.get("current_destination"),
                "current_line_id": task.get("current_line_id"),
                "detail": task.get("detail"),
                "updated_at": task.get("updated_at").isoformat() if task.get("updated_at") else None,
            },
            "items": [{
                "line_id": line["line_id"],
                "name": line["name"],
                "quantity": line["quantity"],
                "picked": bool(line.get("picked")),
            } for line in order.get("items", [])],
        })
    return result


@app.get("/store/orders/{order_number}")
def storefront_get_order(order_number: str, db=Depends(database)):
    """Public tracking lookup by the six-digit order ID."""
    if len(order_number) != 6 or not order_number.isdigit():
        raise HTTPException(status_code=422, detail="Order ID must be six digits")
    order = db.orders.find_one({"order_number": order_number})
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")
    result = order_status_payload(db, order)
    result["id"] = order_number
    result["order_number"] = order_number
    return result


@app.post("/api/v1/robot/tasks/claim")
def claim_task(x_robot_location: Annotated[str | None, Header(alias="X-Robot-Location")] = None,
               role=Depends(require_roles("robot")), db=Depends(database)):
    """Return an active task or claim queued work regardless of prior waypoint."""
    active_states = ("navigating_to_pick", "at_pick", "scanning", "waiting_for_operator",
                     "navigating_to_drop", "at_drop", "returning_to_dock", "assistance_required")
    task = db.robot_tasks.find_one({"state": {"$in": list(active_states)}}, sort=[("updated_at", 1)])
    if task:
        return {"task": _robot_task_payload(db, task), "reason": "active_task_recovered"}

    queued = db.robot_tasks.find_one({"state": "queued"}, sort=[("created_at", 1)])
    if queued:
        task = db.robot_tasks.find_one_and_update(
            {"_id": queued["_id"], "state": "queued"},
            {"$set": {"state": "navigating_to_pick", "claimed_at": utcnow(), "updated_at": utcnow(), "current_destination": "pick"}, "$inc": {"version": 1}},
            return_document=ReturnDocument.AFTER)
        if not task:
            return {"task": None, "reason": "task_claim_race"}
    else:
        # Once all orders are delivered, the last delivery record doubles as the
        # durable return-to-dock work item. Repeated polls recover this state above.
        delivered = db.robot_tasks.find_one({"state": "delivered"}, sort=[("updated_at", -1)])
        if not delivered:
            return {"task": None, "reason": "queue_empty"}
        task = db.robot_tasks.find_one_and_update(
            {"_id": delivered["_id"], "state": "delivered"},
            {"$set": {"state": "returning_to_dock", "current_destination": "dock", "updated_at": utcnow()}, "$inc": {"version": 1}},
            return_document=ReturnDocument.AFTER)
        if not task:
            return {"task": None, "reason": "task_claim_race"}

    return {"task": _robot_task_payload(db, task), "reason": "task_claimed_or_recovered"}


def _robot_task_payload(db, task: dict) -> dict:
    """Build a robot-only payload with current order line metadata."""
    order = db.orders.find_one({"_id": task["order_id"]})
    if not order or not order.get("items"):
        db.robot_tasks.update_one({"_id": task["_id"]}, {"$set": {"state": "assistance_required", "detail": "Order record missing"}})
        task["state"] = "assistance_required"
        task["detail"] = "Order record missing"
        return obj(task)
    first_line = next((item for item in order["items"] if not item["picked"]), None)
    if first_line and not task.get("current_line_id"):
        db.robot_tasks.update_one({"_id": task["_id"]}, {"$set": {"current_line_id": first_line["line_id"]}})
        task["current_line_id"] = first_line["line_id"]
    task["order_number"] = order.get("order_number", task["order_id"])
    task["items"] = [{"line_id": item["line_id"], "sku": item["sku"], "name": item["name"],
                      "quantity": item["quantity"], "picked": item["picked"], "verified": item["verified"]}
                     for item in order["items"]]
    return obj(task)


@app.post("/api/v1/robot/tasks/{order_id}/scan")
def verify_scan(order_id: str, body: ScanInput, role=Depends(require_roles("robot")), db=Depends(database)):
    task = db.robot_tasks.find_one({"order_id": order_id})
    order = db.orders.find_one({"_id": order_id})
    if not task or not order:
        raise HTTPException(status_code=404, detail="Task not found")
    if task["state"] not in ("scanning", "at_pick") or task.get("current_line_id") != body.line_id:
        raise HTTPException(status_code=409, detail="Task is not at the scan stage for this order line")
    line = next((item for item in order["items"] if item["line_id"] == body.line_id), None)
    if line is None or line["picked"]:
        raise HTTPException(status_code=409, detail="Order line is not awaiting identification")
    try:
        scanned = validate_identifier(body.identifier)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    min_confidence = float(os.getenv("MARS_SCAN_MIN_CONFIDENCE", "0.80"))
    match = body.confidence >= min_confidence and hmac.compare_digest(scanned, line["identifier"])
    now = utcnow()
    db.scan_events.insert_one({"order_id": order_id, "line_id": body.line_id, "identifier": scanned,
        "confidence": body.confidence, "matched": match, "created_at": now})
    if not match:
        return {"verified": False, "state": "wrong_product"}
    db.orders.update_one({"_id": order_id, "items.line_id": body.line_id},
        {"$set": {"items.$.verified": True, "updated_at": now}})
    db.robot_tasks.update_one({"order_id": order_id}, {"$set": {"state": "waiting_for_operator", "current_line_id": body.line_id, "updated_at": now}, "$inc": {"version": 1}})
    return {"verified": True, "state": "waiting_for_operator", "line_id": body.line_id}


@app.post("/api/v1/operator/tasks/{order_id}/picked")
def confirm_picked(order_id: str, line_id: str = Header(alias="X-Order-Line-Id"),
                   role=Depends(require_roles("operator", "admin")), db=Depends(database)):
    now = utcnow()
    key = f"picked:{order_id}:{line_id}"
    def apply(session):
        previous = db.inventory_transactions.find_one({"idempotency_key": key}, session=session)
        if previous:
            return {"already_applied": True}
        task = db.robot_tasks.find_one({"order_id": order_id}, session=session)
        order = db.orders.find_one({"_id": order_id}, session=session)
        if (not task or not order
                or task["state"] not in ("at_pick", "scanning", "waiting_for_operator")
                or task.get("current_line_id") != line_id):
            raise HTTPException(status_code=409, detail="Picked is not valid for the current task state")
        line = next((x for x in order["items"] if x["line_id"] == line_id), None)
        if not line or line["picked"]:
            raise HTTPException(status_code=409, detail="Product line is missing or already picked")
        # Picking is an operator workflow milestone, not the inventory
        # deduction point. Stock remains reserved until the operator confirms
        # the order was dropped.
        product = db.products.find_one({"_id": line["product_id"]}, session=session)
        db.inventory_transactions.insert_one({"product_id": line["product_id"], "type": "pick_confirmed",
            "quantity_delta": 0,
            "previous_stock": product["stock_on_hand"] if product else None,
            "new_stock": product["stock_on_hand"] if product else None,
            "order_id": order_id,
            "operator_role": role, "idempotency_key": key, "created_at": now}, session=session)
        db.orders.update_one({"_id": order_id, "items.line_id": line_id}, {"$set": {"items.$.picked": True, "updated_at": now}}, session=session)
        remaining = [x for x in order["items"] if x["line_id"] != line_id and not x["picked"]]
        next_state = "at_pick" if remaining else "navigating_to_drop"
        db.robot_tasks.update_one({"order_id": order_id}, {"$set": {"state": next_state,
            "current_line_id": remaining[0]["line_id"] if remaining else None,
            "current_destination": "pick" if remaining else "drop", "updated_at": now}, "$inc": {"version": 1}}, session=session)
        db.orders.update_one({"_id": order_id}, {"$set": {"status": "in_progress", "updated_at": now}}, session=session)
        return {"already_applied": False, "state": next_state}
    try:
        return run_transaction(db, apply)
    except DuplicateKeyError:
        return {"already_applied": True, "state": "already_advanced"}


@app.post("/api/v1/robot/tasks/{order_id}/state")
def update_task_state(order_id: str, body: TaskStateInput, role=Depends(require_roles("robot")), db=Depends(database)):
    if body.destination:
        try:
            validate_location_key(body.destination)
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc))
    now = utcnow()
    task = db.robot_tasks.find_one({"order_id": order_id})
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    if task["state"] == body.state:
        if body.destination and task.get("current_destination") != body.destination:
            raise HTTPException(status_code=409, detail="Task is already in this state at a different destination")
        return obj(task)
    if not task_transition_allowed(task["state"], body.state):
        raise HTTPException(status_code=409, detail=f"Invalid task transition: {task['state']} -> {body.state}")
    if body.state == "at_drop":
        order = db.orders.find_one({"_id": order_id})
        if not order or any(not line["picked"] for line in order["items"]):
            raise HTTPException(status_code=409, detail="Cannot report arrival at Drop before all lines are picked")
    location_for_state = {"at_pick": "pick", "scanning": "pick", "at_drop": "drop",
                          "returning_to_dock": "dock", "completed": "dock"}
    current_location = body.destination or location_for_state.get(body.state, task.get("current_destination", "dock"))
    try:
        validate_location_key(current_location)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    updated = db.robot_tasks.find_one_and_update({"order_id": order_id, "state": task["state"]},
        {"$set": {"state": body.state, "current_destination": current_location,
          "nav_status": body.nav_status, "detail": body.detail, "updated_at": now}, "$inc": {"version": 1}},
        return_document=ReturnDocument.AFTER)
    if not updated:
        raise HTTPException(status_code=409, detail="Task changed concurrently; refresh task state")
    if body.state == "assistance_required":
        db.orders.update_one({"_id": order_id}, {"$set": {"status": "assistance_required", "updated_at": now}})
    elif body.state in {"navigating_to_pick", "navigating_to_drop", "returning_to_dock"}:
        db.orders.update_one({"_id": order_id}, {"$set": {"status": "in_progress", "updated_at": now}})
    return obj(updated)


@app.post("/api/v1/operator/tasks/{order_id}/dropped")
def confirm_dropped(order_id: str, role=Depends(require_roles("operator", "admin")), db=Depends(database)):
    now = utcnow()
    key = f"dropped:{order_id}"
    existing = db.delivery_confirmations.find_one({"idempotency_key": key})
    if existing:
        return {"already_applied": True, "status": "delivered"}
    task = db.robot_tasks.find_one({"order_id": order_id})
    order = db.orders.find_one({"_id": order_id})
    if not task or not order:
        raise HTTPException(status_code=404, detail="Task not found")
    if task["state"] != "at_drop" or any(not x["picked"] for x in order["items"]):
        raise HTTPException(status_code=409, detail="Dropped is valid only at Drop after all lines are picked")
    def apply(session):
        previous = db.delivery_confirmations.find_one({"idempotency_key": key}, session=session)
        if previous:
            return {"already_applied": True, "status": "delivered"}
        current_order = db.orders.find_one({"_id": order_id}, session=session)
        current_task = db.robot_tasks.find_one({"order_id": order_id}, session=session)
        if (not current_order or not current_task or current_task["state"] != "at_drop"
                or any(not line["picked"] for line in current_order["items"])):
            raise HTTPException(status_code=409, detail="Dropped is valid only at Drop after all lines are picked")
        for line in current_order["items"]:
            product = db.products.find_one_and_update(
                {"_id": line["product_id"], "stock_on_hand": {"$gte": line["quantity"]},
                 "stock_reserved": {"$gte": line["quantity"]}},
                {"$inc": {"stock_on_hand": -line["quantity"],
                          "stock_reserved": -line["quantity"], "version": 1},
                 "$set": {"updated_at": now}},
                return_document=ReturnDocument.BEFORE, session=session)
            if not product:
                raise HTTPException(status_code=409,
                                    detail=f"Reserved stock invariant failed for {line['name']}; delivery was not recorded")
            db.inventory_transactions.insert_one({
                "product_id": line["product_id"], "type": "dropped",
                "quantity_delta": -line["quantity"],
                "previous_stock": product["stock_on_hand"],
                "new_stock": product["stock_on_hand"] - line["quantity"],
                "order_id": order_id, "line_id": line["line_id"],
                "operator_role": role,
                "idempotency_key": f"dropped:{order_id}:{line['line_id']}",
                "created_at": now}, session=session)
        db.delivery_confirmations.insert_one({"order_id": order_id, "operator_role": role, "idempotency_key": key, "created_at": now}, session=session)
        db.orders.update_one({"_id": order_id}, {"$set": {"status": "delivered", "completed_at": now, "updated_at": now}}, session=session)
        db.robot_tasks.update_one({"order_id": order_id, "state": "at_drop"}, {"$set": {"state": "delivered", "updated_at": now}, "$inc": {"version": 1}}, session=session)
        return {"already_applied": False, "status": "delivered"}
    try:
        return run_transaction(db, apply)
    except DuplicateKeyError:
        return {"already_applied": True, "status": "delivered"}


# Serve the customer/admin interface from the same origin as the API.
_web_dir = Path(__file__).resolve().parent / "web"
if _web_dir.is_dir():
    _admin_dir = _web_dir / "admin"
    if _admin_dir.is_dir():
        app.mount("/admin", StaticFiles(directory=str(_admin_dir), html=True), name="warehouse_admin")
    _cart_dir = _web_dir / "cart"
    if _cart_dir.is_dir():
        app.mount("/cart", StaticFiles(directory=str(_cart_dir), html=True), name="warehouse_cart")
    app.mount("/", StaticFiles(directory=str(_web_dir), html=True), name="warehouse_web")
