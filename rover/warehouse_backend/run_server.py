#!/usr/bin/env python3
"""Start the local MARS MongoDB replica set and warehouse web service."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import webbrowser
from pathlib import Path

from pymongo import MongoClient
from pymongo.errors import OperationFailure, PyMongoError


BACKEND_DIR = Path(__file__).resolve().parent
ENV_FILE = BACKEND_DIR / ".env"


def load_environment(path: Path = ENV_FILE) -> None:
    """Read simple KEY=VALUE entries without replacing shell-provided values."""
    if not path.is_file():
        raise RuntimeError(f"Missing {path}; create it from .env.example first")
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key, value = key.strip(), value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        if key and key not in os.environ:
            os.environ[key] = value


def mongo_uris(uri: str) -> tuple[str, str, str, int, str]:
    parts = urllib.parse.urlsplit(uri)
    host = parts.hostname or "127.0.0.1"
    port = parts.port or 27018
    query = dict(urllib.parse.parse_qsl(parts.query, keep_blank_values=True))
    replica_set = query.pop("replicaSet", "rs0")
    direct_query = {**query, "directConnection": "true"}
    direct_uri = urllib.parse.urlunsplit((parts.scheme, parts.netloc, parts.path,
                                         urllib.parse.urlencode(direct_query), parts.fragment))
    replica_uri = urllib.parse.urlunsplit((parts.scheme, parts.netloc, parts.path,
                                           urllib.parse.urlencode({**query, "replicaSet": replica_set}),
                                           parts.fragment))
    return direct_uri, replica_uri, host, port, replica_set


def direct_client(uri: str) -> MongoClient:
    return MongoClient(uri, serverSelectionTimeoutMS=1000, connectTimeoutMS=1000)


def ensure_mongodb() -> None:
    uri = os.getenv("MARS_MONGODB_URI", "mongodb://127.0.0.1:27018/?replicaSet=rs0")
    direct_uri, replica_uri, host, port, replica_set = mongo_uris(uri)
    dbpath = Path(os.getenv(
        "MARS_MONGODB_DBPATH", "~/.local/share/mars-warehouse/mongodb")).expanduser()
    dbpath.mkdir(parents=True, exist_ok=True)
    log_path = Path(os.getenv("MARS_MONGODB_LOG", str(dbpath / "mongod.log"))).expanduser()
    log_path.parent.mkdir(parents=True, exist_ok=True)

    direct = direct_client(direct_uri)
    try:
        direct.admin.command("ping")
    except PyMongoError:
        mongod = shutil.which("mongod")
        if not mongod:
            raise RuntimeError("mongod is not installed or is not on PATH")
        command = [mongod, "--dbpath", str(dbpath), "--port", str(port),
                   "--bind_ip", host, "--replSet", replica_set,
                   "--fork", "--logpath", str(log_path)]
        try:
            subprocess.run(command, check=True, capture_output=True, text=True, timeout=20)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
            # Another process may have won the startup race; confirm MongoDB
            # answers before treating mongod's nonzero exit as a real failure.
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                try:
                    direct.admin.command("ping")
                    break
                except PyMongoError:
                    time.sleep(0.25)
            else:
                detail = getattr(error, "stderr", None) or str(error)
                raise RuntimeError(f"Could not start mongod: {detail}") from error

    try:
        direct.admin.command("replSetGetStatus")
    except OperationFailure as error:
        if error.code != 94:  # NotYetInitialized
            raise
        member = os.getenv("MARS_MONGODB_RS_HOST", f"{host}:{port}")
        direct.admin.command("replSetInitiate", {
            "_id": replica_set,
            "members": [{"_id": 0, "host": member}],
        })
    finally:
        direct.close()

    client = MongoClient(replica_uri, serverSelectionTimeoutMS=1500, connectTimeoutMS=1000)
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            try:
                hello = client.admin.command("hello")
                if hello.get("isWritablePrimary"):
                    print(f"MongoDB ready: {host}:{port}/{replica_set}")
                    return
            except PyMongoError:
                pass
            time.sleep(0.5)
        raise RuntimeError("MongoDB started but the replica set did not elect a primary")
    finally:
        client.close()


def health_ok(url: str) -> bool:
    try:
        with urllib.request.urlopen(url.rstrip("/") + "/health", timeout=1.0) as response:
            return response.status == 200
    except (OSError, urllib.error.URLError):
        return False


def main() -> int:
    load_environment()
    ensure_mongodb()

    host = os.getenv("MARS_WAREHOUSE_HOST", "127.0.0.1")
    port = int(os.getenv("MARS_WAREHOUSE_PORT", "8000"))
    url = os.getenv("MARS_WAREHOUSE_URL", f"http://{host}:{port}/")
    if health_ok(url):
        print("Warehouse API is already running; opening another browser tab.")
        webbrowser.open_new_tab(url)
        return 0

    try:
        import uvicorn
    except ImportError as error:
        raise RuntimeError("Install uvicorn in the general Python environment") from error

    source_dir = str(BACKEND_DIR / "src")
    if source_dir not in sys.path:
        sys.path.insert(0, source_dir)

    server = uvicorn.Server(uvicorn.Config(
        "mars_warehouse_backend.app:app",
        host=host,
        port=port,
        log_level="info",
    ))
    thread = threading.Thread(target=server.run, name="warehouse-uvicorn", daemon=True)
    thread.start()
    health_url = url.rstrip("/") + "/health"
    deadline = time.monotonic() + 25.0
    while time.monotonic() < deadline and thread.is_alive():
        if server.started and health_ok(url):
            webbrowser.open_new_tab(url)
            print(f"Warehouse ready at {url} (opened a browser tab). Press Ctrl+C to stop the API.")
            break
        time.sleep(0.25)
    else:
        server.should_exit = True
        thread.join(timeout=3.0)
        raise RuntimeError(f"Warehouse API did not become ready at {health_url}")

    try:
        while thread.is_alive():
            thread.join(timeout=0.5)
    except KeyboardInterrupt:
        server.should_exit = True
        thread.join(timeout=5.0)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"Warehouse startup failed: {error}", file=sys.stderr)
        raise SystemExit(1)
