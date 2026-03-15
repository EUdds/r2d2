#!/usr/bin/env python3
import os
import uuid
import json
import hashlib
import asyncio
import datetime as dt
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fastapi import FastAPI, UploadFile, File, HTTPException, Header
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

# ---------- Config ----------
FIRMWARE_DIR = Path(os.environ.get("OTA_FIRMWARE_DIR", "/opt/ota/firmware"))
INDEX_PATH   = FIRMWARE_DIR / "index.json"
API_TOKEN    = os.environ.get("OTA_API_TOKEN")  # optional simple auth
HOST         = os.environ.get("OTA_HOST", "0.0.0.0")
PORT         = int(os.environ.get("OTA_PORT", "8080"))

FIRMWARE_DIR.mkdir(parents=True, exist_ok=True)
if not INDEX_PATH.exists():
    INDEX_PATH.write_text(json.dumps({"artifacts": {}}, indent=2))

# ---------- Helpers ----------
def load_index():
    try:
        return json.loads(INDEX_PATH.read_text())
    except Exception:
        return {"artifacts": {}}

def save_index(idx):
    tmp = INDEX_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(idx, indent=2))
    tmp.replace(INDEX_PATH)

def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

def now_iso() -> str:
    return dt.datetime.utcnow().replace(tzinfo=dt.timezone.utc).isoformat()

# ---------- FastAPI models ----------
class DeployRequest(BaseModel):
    firmware_id: str
    targets: List[str]
    job_id: Optional[str] = None  # optional; auto-gen if omitted

# ---------- ROS2 + Web ----------
class OtaServerNode(Node):
    def __init__(self):
        super().__init__("ota_http_server")
        self.publisher = self.create_publisher(String, "ota_jobs", 10)
        self.app = self._build_app()

    def _auth_guard(self, auth_header: Optional[str]):
        if API_TOKEN is None:
            return  # no auth enforced
        if not auth_header or not auth_header.startswith("Bearer "):
            raise HTTPException(status_code=401, detail="Missing or invalid Authorization header")
        token = auth_header.split(" ", 1)[1]
        if token != API_TOKEN:
            raise HTTPException(status_code=403, detail="Forbidden")

    def _build_app(self) -> FastAPI:
        app = FastAPI(title="OTA Server", version="0.1.0")

        # Allow local tooling / dashboards
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.get("/health")
        async def health():
            return {"ok": True, "time": now_iso()}

        @app.post("/ota/upload")
        async def upload_firmware(
            file: UploadFile = File(...),
            authorization: Optional[str] = Header(default=None)
        ):
            self._auth_guard(authorization)

            # Save firmware
            firmware_id = uuid.uuid4().hex
            dest = FIRMWARE_DIR / f"{firmware_id}.bin"
            try:
                # Stream to disk
                with dest.open("wb") as out:
                    while True:
                        chunk = await file.read(1024 * 1024)
                        if not chunk:
                            break
                        out.write(chunk)
            finally:
                await file.close()

            # Hash + index
            digest = sha256_file(dest)
            idx = load_index()
            idx["artifacts"][firmware_id] = {
                "filename": file.filename,
                "path": str(dest),
                "sha256": digest,
                "size": dest.stat().st_size,
                "created_at": now_iso(),
            }
            save_index(idx)

            return {
                "firmware_id": firmware_id,
                "sha256": digest,
                "size": dest.stat().st_size,
                "stored_as": str(dest),
            }

        @app.post("/ota/deploy")
        async def deploy(req: DeployRequest, authorization: Optional[str] = Header(default=None)):
            self._auth_guard(authorization)

            idx = load_index()
            art = idx["artifacts"].get(req.firmware_id)
            if not art:
                raise HTTPException(status_code=404, detail="firmware_id not found")

            job_id = req.job_id or uuid.uuid4().hex
            payload = {
                "job_id": job_id,
                "firmware_id": req.firmware_id,
                "targets": req.targets,
                "created_at": now_iso(),
                "sha256": art["sha256"],
                "path": art["path"],
            }
            msg = String()
            msg.data = json.dumps(payload)
            self.publisher.publish(msg)

            # Optional: log for observability
            self.get_logger().info(f"Published OTA job {job_id} -> {req.targets}")

            return {"accepted": True, "job_id": job_id, "published_on": "ota_jobs"}

        @app.get("/ota/index")
        async def list_index():
            return load_index()

        return app

    async def run_http(self):
        """Run uvicorn inside the current event loop."""
        config = uvicorn.Config(self.app, host=HOST, port=PORT, log_level="info")
        server = uvicorn.Server(config)
        await server.serve()

def main():
    rclpy.init()
    node = OtaServerNode()

    # Run FastAPI + ROS in the same asyncio loop
    loop = asyncio.get_event_loop()

    async def main_async():
        # Start HTTP server task
        http_task = asyncio.create_task(node.run_http())
        try:
            # Spin ROS periodically so timers/callbacks work if you add them later
            while not http_task.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                await asyncio.sleep(0.01)
        finally:
            node.get_logger().info("Shutting down OTA server…")
            http_task.cancel()

    try:
        loop.run_until_complete(main_async())
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
