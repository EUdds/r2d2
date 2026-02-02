const { useEffect, useMemo, useRef, useState } = React;

const SAMPLE_PAYLOADS = {
  PsiColorRequest: { red: 255, green: 120, blue: 20, brightness: 128 },
  HoloColorRequest: { red: 0, green: 180, blue: 255, brightness: 180 },
  ServoMoveCommand: { servo: 1, position_deg: 45, duration_ms: 0 },
  ServoHomeCommand: { servo: 1, all: false },
  HoloPositionRequest: { azimuth_deg: 30, elevation_deg: 15 },
  PsiMoodRequest: { mood: 1 },
  Ping: { payload: [112, 105, 110, 103] },
  Pong: { payload: [112, 111, 110, 103] },
  Empty: {},
};

const HEARTBEAT_MESSAGE = "Heartbeat";
const HEARTBEAT_OK_MS = 3000;
const HEARTBEAT_WARN_MS = 10000;

function buildDefaultUrl() {
  // Try to load from localStorage first
  const saved = localStorage.getItem("r2bus_ws_url");
  if (saved) {
    return saved;
  }
  // Fall back to hostname-based default
  const host = window.location.hostname || "localhost";
  return `ws://${host}:8090/ws`;
}

function parseDestId(value) {
  const trimmed = value.trim();
  if (!trimmed) return null;
  const parsed = trimmed.startsWith("0x") ? parseInt(trimmed, 16) : parseInt(trimmed, 10);
  if (Number.isNaN(parsed) || parsed < 0 || parsed > 255) return null;
  return parsed;
}

function formatTime(ts) {
  const date = new Date(ts * 1000);
  return date.toLocaleTimeString();
}

function formatAge(msAgo) {
  if (msAgo == null) return "no data";
  if (msAgo < 1000) return "just now";
  if (msAgo < 60000) return `${Math.round(msAgo / 1000)}s ago`;
  if (msAgo < 3600000) return `${Math.round(msAgo / 60000)}m ago`;
  return `${Math.round(msAgo / 3600000)}h ago`;
}

function heartbeatState(msAgo) {
  if (msAgo == null) {
    return { state: "bad"};
  }
  if (msAgo < HEARTBEAT_OK_MS) {
    return { state: "good"};
  }
  if (msAgo < HEARTBEAT_WARN_MS) {
    return { state: "warn"};
  }
  return { state: "bad"};
}

function App() {
  const [bridgeUrl, setBridgeUrl] = useState(buildDefaultUrl());
  const [status, setStatus] = useState("disconnected");
  const [messages, setMessages] = useState([]);
  const [messageNames, setMessageNames] = useState([]);
  const [serviceNames, setServiceNames] = useState([]);
  const [nodeMap, setNodeMap] = useState({});
  const [nodeHeartbeat, setNodeHeartbeat] = useState({});
  const [availableFirmware, setAvailableFirmware] = useState([]);
  const [piLastSeen, setPiLastSeen] = useState(null);
  const [piHeartbeat, setPiHeartbeat] = useState(null);
  const [activePage, setActivePage] = useState(
    window.location.hash === "#services" ? "services" : 
    window.location.hash === "#status" ? "status" :
    window.location.hash === "#control" ? "control" : "console"
  );
  const [tick, setTick] = useState(0);
  const [selectedMessage, setSelectedMessage] = useState("");
  const [destIdInput, setDestIdInput] = useState("0x10");
  const [payloadInput, setPayloadInput] = useState("{}");
  const [filterText, setFilterText] = useState("");
  const [events, setEvents] = useState([]);
  const [lastModified, setLastModified] = useState(null);
  const [firmwareNodeId, setFirmwareNodeId] = useState("0x42");
  const [firmwarePath, setFirmwarePath] = useState("");
  const [skipReset, setSkipReset] = useState(false);
  const [skipBoot, setSkipBoot] = useState(false);
  const [updateInProgress, setUpdateInProgress] = useState(false);
  const [firmwareProgress, setFirmwareProgress] = useState(null);
  const [indicators, setIndicators] = useState([]);
  const [actuators, setActuators] = useState([]);
  const [newIndicatorName, setNewIndicatorName] = useState("");
  const [newActuatorName, setNewActuatorName] = useState("");
  const [topArmAngle, setTopArmAngle] = useState(85);
  const [bottomArmAngle, setBottomArmAngle] = useState(85);
  const wsRef = useRef(null);

  const filteredMessages = useMemo(() => {
    if (!filterText) return messages;
    const query = filterText.toLowerCase();
    return messages.filter((item) => {
      return (
        item.topic.toLowerCase().includes(query) ||
        item.message.toLowerCase().includes(query)
      );
    });
  }, [messages, filterText]);

  const ecuNodes = useMemo(() => {
    const now = Date.now();
    return Object.entries(nodeMap || {})
      .map(([id, name]) => {
        const parsedId = Number.parseInt(id, 10);
        const normalizedId = Number.isNaN(parsedId) ? null : parsedId;
        const key = normalizedId === null ? id : String(normalizedId);
        const lastSeen = nodeHeartbeat[key] || null;
        const age = lastSeen ? now - lastSeen : null;
        const statusInfo = heartbeatState(age);
        const idHex = normalizedId === null
          ? id
          : `0x${normalizedId.toString(16).padStart(2, "0")}`;
        return {
          id: normalizedId,
          idHex,
          name: name || `node_${id}`,
          lastSeen,
          age,
          status: statusInfo,
        };
      })
      .sort((a, b) => (a.id ?? 0) - (b.id ?? 0));
  }, [nodeMap, nodeHeartbeat, tick]);

  useEffect(() => {
    connectWebSocket();
    return () => disconnectWebSocket();
  }, []);

  useEffect(() => {
    const controller = new AbortController();
    fetch("app.js", { method: "HEAD", cache: "no-store", signal: controller.signal })
      .then((response) => response.headers.get("last-modified"))
      .then((value) => {
        if (value) {
          setLastModified(new Date(value));
        }
      })
      .catch(() => {});
    return () => controller.abort();
  }, []);

  useEffect(() => {
    const onHashChange = () => {
      if (window.location.hash === "#services") {
        setActivePage("services");
      } else if (window.location.hash === "#status") {
        setActivePage("status");
      } else {
        setActivePage("console");
      }
    };
    window.addEventListener("hashchange", onHashChange);
    return () => window.removeEventListener("hashchange", onHashChange);
  }, []);

  function addIndicator() {
    if (!newIndicatorName.trim()) return;
    setIndicators([...indicators, { id: Date.now(), name: newIndicatorName, status: "off" }]);
    setNewIndicatorName("");
  }

  function addActuator() {
    if (!newActuatorName.trim()) return;
    setActuators([...actuators, { id: Date.now(), name: newActuatorName, value: 0 }]);
    setNewActuatorName("");
  }

  function removeIndicator(id) {
    setIndicators(indicators.filter(i => i.id !== id));
  }

  function removeActuator(id) {
    setActuators(actuators.filter(a => a.id !== id));
  }

  function toggleIndicator(id) {
    setIndicators(indicators.map(i => 
      i.id === id ? { ...i, status: i.status === "off" ? "on" : "off" } : i
    ));
  }

  function updateActuatorValue(id, value) {
    setActuators(actuators.map(a =>
      a.id === id ? { ...a, value: parseInt(value) } : a
    ));
  }

  function sendArmControl(arm, open, positionOverride = false, angle = 0) {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      logEvent("WebSocket not connected", "error");
      return;
    }

    const requestId = Math.random().toString(16).slice(2, 10);
    const armLabel = arm === "top_arm" ? "Top Arm (Ch15)" : "Bottom Arm (Ch14)";

    ws.send(
      JSON.stringify({
        type: "arm_control",
        request_id: requestId,
        arm: arm,
        payload: {
          open: open,
          position_override: positionOverride,
          angle: angle
        },
      })
    );

    if (positionOverride) {
      logEvent(`${armLabel} -> ${angle}°`, "info");
    } else {
      logEvent(`${armLabel} -> ${open ? "open" : "close"}`, "info");
    }
  }

  function moveTopArm(angle) {
    sendArmControl("top_arm", false, true, angle);
  }

  function moveBottomArm(angle) {
    sendArmControl("bottom_arm", false, true, angle);
  }

  function openTopArm() {
    setTopArmAngle(0);
    sendArmControl("top_arm", false, true, 0);
  }

  function closeTopArm() {
    setTopArmAngle(170);
    sendArmControl("top_arm", false, true, 170);
  }

  function openBottomArm() {
    setBottomArmAngle(0);
    sendArmControl("bottom_arm", false, true, 0);
  }

  function closeBottomArm() {
    setBottomArmAngle(170);
    sendArmControl("bottom_arm", false, true, 170);
  }

  function openBothArms() {
    openTopArm();
    openBottomArm();
  }

  function closeBothArms() {
    closeTopArm();
    closeBottomArm();
  }

  useEffect(() => {
    const timer = setInterval(() => setTick((prev) => prev + 1), 1000);
    return () => clearInterval(timer);
  }, []);

  function logEvent(text, tone = "info") {
    setEvents((prev) => {
      const next = [{ text, tone, time: Date.now() }, ...prev];
      return next.slice(0, 20);
    });
  }

  function connectWebSocket() {
    disconnectWebSocket();
    if (!bridgeUrl) return;
    
    // Save URL to localStorage for next time
    localStorage.setItem("r2bus_ws_url", bridgeUrl);
    
    const ws = new WebSocket(bridgeUrl);
    wsRef.current = ws;
    setStatus("connecting");

    ws.onopen = () => {
      setStatus("connected");
      setPiLastSeen(Date.now());
      logEvent(`Connected to ${bridgeUrl}`, "success");
    };

    ws.onclose = () => {
      setStatus("disconnected");
      setPiLastSeen(null);
      logEvent("Disconnected", "error");
    };

    ws.onerror = () => {
      setStatus("disconnected");
      logEvent("WebSocket error", "error");
    };

    ws.onmessage = (event) => {
      let payload;
      try {
        payload = JSON.parse(event.data);
      } catch (err) {
        logEvent("Invalid message from bridge", "error");
        return;
      }

      if (payload.type === "hello") {
        setMessageNames(payload.messages || []);
        setServiceNames(payload.services || []);
        if (payload.nodes) {
          setNodeMap(payload.nodes);
        }
        if (payload.available_firmware) {
          setAvailableFirmware(payload.available_firmware);
        }
        setPiLastSeen(Date.now());
        if (!selectedMessage && payload.services && payload.services.length) {
          setSelectedMessage(payload.services[0]);
          setPayloadInput(JSON.stringify(SAMPLE_PAYLOADS[payload.services[0]] || {}, null, 2));
        }
        return;
      }

      if (payload.type === "firmware_progress") {
        setFirmwareProgress(payload);
        const statusMsg = `[0x${payload.node_id.toString(16).padStart(2, "0").toUpperCase()}] ${payload.step}: ${payload.message} (${payload.percent}%)`;
        logEvent(statusMsg, "info");
        return;
      }

      if (payload.type === "telemetry") {
        const entry = {
          topic: payload.topic || "",
          message: payload.message || "",
          timestamp: payload.timestamp || Date.now() / 1000,
          data: payload.data || {},
          nodeId: payload.node_id,
        };
        setMessages((prev) => [entry, ...prev].slice(0, 200));
        if (entry.message === HEARTBEAT_MESSAGE) {
          if (payload.node_id !== undefined && payload.node_id !== null) {
            const key = String(payload.node_id);
            setNodeHeartbeat((prev) => ({ ...prev, [key]: entry.timestamp * 1000 }));
          } else {
            setPiHeartbeat(entry.timestamp * 1000);
          }
        }
        setPiLastSeen(Date.now());
        return;
      }

      if (payload.type === "ack") {
        const statusName = payload.ack && payload.ack.status_name ? payload.ack.status_name : "ACK";
        logEvent(`${payload.message} -> ${statusName}`, "success");
        return;
      }

      if (payload.type === "error") {
        logEvent(payload.error || "Bridge error", "error");
      }

      if (payload.type === "service_response") {
        const success = payload.result?.success;
        const message = payload.result?.message || "Unknown result";
        const tone = success ? "success" : "error";
        logEvent(`Service result: ${message}`, tone);
        setUpdateInProgress(false);
        // Clear progress when update completes (whether success or failure)
        setTimeout(() => setFirmwareProgress(null), 2000);
        return;
      }
    };
  }

  function disconnectWebSocket() {
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
  }

  function handleMessageChange(value) {
    setSelectedMessage(value);
    const sample = SAMPLE_PAYLOADS[value] || {};
    setPayloadInput(JSON.stringify(sample, null, 2));
  }

  function handleSend() {
    const destId = parseDestId(destIdInput);
    if (destId === null) {
      logEvent("Invalid dest_id (use 0xNN or decimal)", "error");
      return;
    }
    if (!selectedMessage) {
      logEvent("Select a message", "error");
      return;
    }

    let payload;
    try {
      payload = payloadInput.trim() ? JSON.parse(payloadInput) : {};
    } catch (err) {
      logEvent("Payload JSON is invalid", "error");
      return;
    }

    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      logEvent("WebSocket not connected", "error");
      return;
    }

    const requestId = Math.random().toString(16).slice(2, 10);
    ws.send(
      JSON.stringify({
        type: "send",
        request_id: requestId,
        message: selectedMessage,
        dest_id: destId,
        payload,
      })
    );
    logEvent(`Sent ${selectedMessage} -> ${destIdInput}`, "info");
  }

  function handleResetNode(node) {
    const ws = wsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      logEvent("WebSocket not connected", "error");
      return;
    }
    if (!serviceNames.includes("Empty")) {
      logEvent("Reset message not available (missing 'Empty' proto)", "error");
      return;
    }
    if (node.id == null || Number.isNaN(Number(node.id))) {
      logEvent("Cannot reset node (invalid id)", "error");
      return;
    }
    const requestId = Math.random().toString(16).slice(2, 10);
    ws.send(
      JSON.stringify({
        type: "send",
        request_id: requestId,
        message: "Empty",
        dest_id: Number(node.id),
        payload: {},
      })
    );
    logEvent(`Sent ECU reset -> ${node.name} (${node.idHex})`, "info");
  }

  function setPage(nextPage) {
    const hash = nextPage === "status" ? "status" : nextPage === "services" ? "services" : nextPage === "control" ? "control" : "";
    window.location.hash = hash;
    setActivePage(nextPage);
  }

  function handleFirmwareUpdate() {
    const nodeId = parseDestId(firmwareNodeId);
    if (nodeId === null) {
      logEvent("Invalid node ID (use 0xNN or decimal)", "error");
      return;
    }
    
    // If no dropdown available, firmware path is required
    if (availableFirmware.length === 0 && !firmwarePath.trim()) {
      logEvent("Firmware path is required", "error");
      return;
    }
    
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      logEvent("WebSocket not connected", "error");
      return;
    }

    setUpdateInProgress(true);
    const requestId = Math.random().toString(16).slice(2, 10);
    
    const request = {
      type: "service_call",
      request_id: requestId,
      service: "r2bus/firmware_update",
      node_id: nodeId,
      firmware_path: firmwarePath,  // Empty string if using dropdown (gateway will look up from CSV)
      skip_reset: skipReset,
      skip_boot: skipBoot,
    };

    wsRef.current.send(JSON.stringify(request));
    const pathDesc = availableFirmware.length > 0 ? "(from CSV)" : `from ${firmwarePath}`;
    logEvent(`Started firmware update for 0x${nodeId.toString(16).padStart(2, "0").toUpperCase()} ${pathDesc}`, "info");
  }

  const piHeartbeatAge = piHeartbeat
    ? Date.now() - piHeartbeat
    : piLastSeen
      ? Date.now() - piLastSeen
      : null;
  const piStatus = heartbeatState(piHeartbeatAge);

  return (
    <div className="app">
      <header>
        <div className="header-stack">
          <h1 className="title">R2BUS Web Console</h1>
          <div className="subtitle">Live telemetry, command dispatch, and ACK tracking.</div>
          <div className="nav">
            <button
              className={`nav-button ${activePage === "console" ? "active" : ""}`}
              onClick={() => setPage("console")}
            >
              Console
            </button>
            <button
              className={`nav-button ${activePage === "control" ? "active" : ""}`}
              onClick={() => setPage("control")}
            >
              Control
            </button>
            <button
              className={`nav-button ${activePage === "status" ? "active" : ""}`}
              onClick={() => setPage("status")}
            >
              ECU Status
            </button>
            <button
              className={`nav-button ${activePage === "services" ? "active" : ""}`}
              onClick={() => setPage("services")}
            >
              Services
            </button>
          </div>
        </div>
        <div className="header-controls">
          <input
            type="text"
            value={bridgeUrl}
            onChange={(e) => setBridgeUrl(e.target.value)}
            placeholder="ws://host:8090/ws"
            className="url-input"
          />
          <button className="button secondary reconnect-button" onClick={connectWebSocket}>
            Reconnect
          </button>
          <div className="status">
            <span className={`status-dot ${status === "connected" ? "connected" : ""}`}></span>
            {status}
          </div>
        </div>
      </header>

      {activePage === "services" ? (
        <section className="card">
          <h2>ROS 2 Services</h2>
          <div className="services-container">
            <div className="service-card">
              <h3>Firmware Update</h3>
              <p className="service-description">Update firmware on an embedded R2 bus node</p>
              <div className="control-row">
                <label className="form-label">Target Node</label>
                {availableFirmware.length > 0 ? (
                  <select 
                    value={firmwareNodeId}
                    onChange={(e) => setFirmwareNodeId(e.target.value)}
                  >
                    <option value="">Select a node...</option>
                    {availableFirmware.map((fw) => (
                      <option key={fw.node_id} value={`0x${fw.node_id.toString(16).padStart(2, "0")}`}>
                        {fw.node_name} ({`0x${fw.node_id.toString(16).padStart(2, "0").toUpperCase()}`})
                      </option>
                    ))}
                  </select>
                ) : (
                  <input
                    type="text"
                    placeholder="Node ID (0x42)"
                    value={firmwareNodeId}
                    onChange={(e) => setFirmwareNodeId(e.target.value)}
                  />
                )}
              </div>
              {availableFirmware.length > 0 ? (
                <div className="firmware-info">
                  {(() => {
                    const nodeId = parseInt(firmwareNodeId, 16) || parseInt(firmwareNodeId, 10);
                    const fw = availableFirmware.find(f => f.node_id === nodeId);
                    if (fw) {
                      return (
                        <>
                          <div className="info-line">
                            <strong>Firmware:</strong> {fw.artifact.split('/').pop()}
                          </div>
                          {fw.pcb_rev && <div className="info-line"><strong>PCB Rev:</strong> {fw.pcb_rev}</div>}
                          {fw.sha256 && <div className="info-line"><strong>SHA256:</strong> {fw.sha256.substring(0, 16)}...</div>}
                        </>
                      );
                    }
                    return <div className="info-line error">No firmware available for this node</div>;
                  })()}
                </div>
              ) : (
                <div className="control-row">
                  <input
                    type="text"
                    placeholder="Firmware path (e.g., /opt/deploy/firmware.bin)"
                    value={firmwarePath}
                    onChange={(e) => setFirmwarePath(e.target.value)}
                  />
                </div>
              )}
              <div className="control-row checkbox-group">
                <label className="checkbox-label">
                  <input
                    type="checkbox"
                    checked={skipReset}
                    onChange={(e) => setSkipReset(e.target.checked)}
                  />
                  Skip ECU Reset
                </label>
                <label className="checkbox-label">
                  <input
                    type="checkbox"
                    checked={skipBoot}
                    onChange={(e) => setSkipBoot(e.target.checked)}
                  />
                  Skip Boot
                </label>
              </div>
              <button
                className="button"
                onClick={handleFirmwareUpdate}
                disabled={!status === "connected" || updateInProgress}
              >
                {updateInProgress ? "Updating..." : "Start Update"}
              </button>
            </div>
          </div>

          {firmwareProgress && (
            <div className="progress-container">
              <h2>Firmware Update Progress</h2>
              <div className="progress-info">
                <div className="info-line">
                  <strong>Node:</strong> 0x{firmwareProgress.node_id.toString(16).padStart(2, "0").toUpperCase()}
                </div>
                <div className="info-line">
                  <strong>Step:</strong> {firmwareProgress.step}
                </div>
                <div className="info-line">
                  <strong>Message:</strong> {firmwareProgress.message}
                </div>
                {firmwareProgress.bytes_total > 0 && (
                  <>
                    <div className="info-line">
                      <strong>Progress:</strong> {firmwareProgress.bytes_processed} / {firmwareProgress.bytes_total} bytes
                    </div>
                    <div className="progress-bar">
                      <div className="progress-fill" style={{ width: `${firmwareProgress.percent}%` }}></div>
                    </div>
                    <div className="info-line">
                      <strong>Percent:</strong> {firmwareProgress.percent}%
                    </div>
                  </>
                )}
              </div>
            </div>
          )}

          <h2>Service Status</h2>
          <div className="event-log">
            {events.length === 0 ? (
              <div className="event-item">No service calls yet.</div>
            ) : (
              events.slice(0, 10).map((event, idx) => (
                <div className="event-item" key={`${event.time}-${idx}`}>
                  <span className={`event-tone ${event.tone}`}>{event.tone}</span>
                  {new Date(event.time).toLocaleTimeString()} - {event.text}
                </div>
              ))
            )}
          </div>
        </section>
      ) : activePage === "control" ? (
        <section className="control-page">
          <h2>R2D2 Control Panel</h2>
          
          <div className="r2d2-layout">
            <div className="r2d2-canvas">
              <svg viewBox="0 0 180 320" className="r2d2-svg">
                {/* Barrel body - main cylinder */}
                <ellipse cx="90" cy="160" rx="45" ry="60" className="r2d2-barrel" fill="none" stroke="currentColor" strokeWidth="2.5"/>
                
                {/* Dome - hemisphere on top */}
                <path d="M 55 100 Q 55 50 90 40 Q 125 50 125 100" className="r2d2-dome" fill="none" stroke="currentColor" strokeWidth="2.5"/>
                
                {/* Dome shine/highlight */}
                <ellipse cx="90" cy="65" rx="18" ry="12" className="r2d2-shine" fill="none" stroke="currentColor" strokeWidth="1.2" opacity="0.6"/>
                
                {/* Main photoreceptor (large blue lens) */}
                <circle cx="90" cy="75" r="7" className="r2d2-main-eye" fill="none" stroke="currentColor" strokeWidth="1.8"/>
                <circle cx="90" cy="75" r="4" className="r2d2-main-eye-inner" fill="none" stroke="currentColor" strokeWidth="0.8" opacity="0.7"/>
                
                {/* Side photoreceptors */}
                <circle cx="70" cy="85" r="3.5" className="r2d2-side-eye" fill="none" stroke="currentColor" strokeWidth="1.2"/>
                <circle cx="110" cy="85" r="3.5" className="r2d2-side-eye" fill="none" stroke="currentColor" strokeWidth="1.2"/>
                
                {/* Dome ridge line */}
                <line x1="90" y1="40" x2="90" y2="100" stroke="currentColor" strokeWidth="1.2" opacity="0.4"/>
                
                {/* Upper body panel */}
                <rect x="65" y="110" width="50" height="35" rx="4" className="r2d2-panel-upper" fill="none" stroke="currentColor" strokeWidth="1.5"/>
                
                {/* Panel details - grooves and features */}
                <line x1="75" y1="115" x2="105" y2="115" stroke="currentColor" strokeWidth="0.8" opacity="0.5"/>
                <line x1="75" y1="125" x2="105" y2="125" stroke="currentColor" strokeWidth="0.8" opacity="0.5"/>
                <line x1="75" y1="135" x2="105" y2="135" stroke="currentColor" strokeWidth="0.8" opacity="0.5"/>
                
                {/* Circular port on chest */}
                <circle cx="90" cy="122" r="4" className="r2d2-port" fill="none" stroke="currentColor" strokeWidth="1"/>
                
                {/* Middle band detail */}
                <line x1="60" y1="150" x2="120" y2="150" stroke="currentColor" strokeWidth="1.2" opacity="0.5"/>
                
                {/* Lower body panel with horizontal lines */}
                <rect x="65" y="155" width="50" height="45" rx="4" className="r2d2-panel-lower" fill="none" stroke="currentColor" strokeWidth="1.5"/>
                <line x1="70" y1="165" x2="110" y2="165" stroke="currentColor" strokeWidth="0.8" opacity="0.4"/>
                <line x1="70" y1="175" x2="110" y2="175" stroke="currentColor" strokeWidth="0.8" opacity="0.4"/>
                <line x1="70" y1="185" x2="110" y2="185" stroke="currentColor" strokeWidth="0.8" opacity="0.4"/>
                
                {/* Foot/base section */}
                <ellipse cx="90" cy="215" rx="42" ry="15" className="r2d2-foot" fill="none" stroke="currentColor" strokeWidth="2"/>
                
                {/* Left wheel */}
                <circle cx="65" cy="220" r="12" className="r2d2-wheel" fill="none" stroke="currentColor" strokeWidth="2.2"/>
                <circle cx="65" cy="220" r="7" className="r2d2-wheel-hub" fill="none" stroke="currentColor" strokeWidth="1" opacity="0.6"/>
                <line x1="58" y1="220" x2="72" y2="220" stroke="currentColor" strokeWidth="1" opacity="0.5"/>
                <line x1="65" y1="213" x2="65" y2="227" stroke="currentColor" strokeWidth="1" opacity="0.5"/>
                
                {/* Right wheel */}
                <circle cx="115" cy="220" r="12" className="r2d2-wheel" fill="none" stroke="currentColor" strokeWidth="2.2"/>
                <circle cx="115" cy="220" r="7" className="r2d2-wheel-hub" fill="none" stroke="currentColor" strokeWidth="1" opacity="0.6"/>
                <line x1="108" y1="220" x2="122" y2="220" stroke="currentColor" strokeWidth="1" opacity="0.5"/>
                <line x1="115" y1="213" x2="115" y2="227" stroke="currentColor" strokeWidth="1" opacity="0.5"/>
                
                {/* Center caster wheel */}
                <circle cx="90" cy="235" r="8" className="r2d2-caster" fill="none" stroke="currentColor" strokeWidth="1.8" opacity="0.7"/>
                <circle cx="90" cy="235" r="4" className="r2d2-caster-hub" fill="none" stroke="currentColor" strokeWidth="0.8" opacity="0.5"/>
                
                {/* Indicators - positioned around R2D2 */}
                {indicators.map((indicator, idx) => {
                  const angle = (idx * 360 / Math.max(indicators.length, 1)) * Math.PI / 180;
                  const radius = 65;
                  const x = 90 + radius * Math.cos(angle);
                  const y = 160 + radius * Math.sin(angle);
                  return (
                    <g key={indicator.id}>
                      <circle cx={x} cy={y} r="6" className={`r2d2-indicator ${indicator.status}`} fill="none" stroke="currentColor" strokeWidth="1.5"/>
                      <circle cx={x} cy={y} r="3" className={`r2d2-indicator-dot ${indicator.status}`} fill={indicator.status === "on" ? "currentColor" : "none"}/>
                    </g>
                  );
                })}
              </svg>
            </div>
            
            <div className="control-panels">
              <div className="indicator-panel">
                <h3>Indicators</h3>
                <div className="add-control">
                  <input
                    type="text"
                    placeholder="Indicator name (e.g., Motion, Power)"
                    value={newIndicatorName}
                    onChange={(e) => setNewIndicatorName(e.target.value)}
                    onKeyPress={(e) => e.key === "Enter" && addIndicator()}
                  />
                  <button className="button" onClick={addIndicator}>Add</button>
                </div>
                <div className="control-list">
                  {indicators.length === 0 ? (
                    <div className="empty-state">No indicators added yet</div>
                  ) : (
                    indicators.map((indicator) => (
                      <div key={indicator.id} className="control-item">
                        <div className="control-info">
                          <span className="control-name">{indicator.name}</span>
                          <span className={`control-status ${indicator.status}`}>{indicator.status}</span>
                        </div>
                        <div className="control-actions">
                          <button className="action-button" onClick={() => toggleIndicator(indicator.id)}>
                            Toggle
                          </button>
                          <button className="action-button danger" onClick={() => removeIndicator(indicator.id)}>
                            Remove
                          </button>
                        </div>
                      </div>
                    ))
                  )}
                </div>
              </div>
              
              <div className="actuator-panel">
                <h3>Actuators</h3>
                <div className="add-control">
                  <input
                    type="text"
                    placeholder="Actuator name (e.g., Motor, Servo)"
                    value={newActuatorName}
                    onChange={(e) => setNewActuatorName(e.target.value)}
                    onKeyPress={(e) => e.key === "Enter" && addActuator()}
                  />
                  <button className="button" onClick={addActuator}>Add</button>
                </div>
                <div className="control-list">
                  {actuators.length === 0 ? (
                    <div className="empty-state">No actuators added yet</div>
                  ) : (
                    actuators.map((actuator) => (
                      <div key={actuator.id} className="control-item">
                        <div className="control-info">
                          <span className="control-name">{actuator.name}</span>
                        </div>
                        <div className="control-slider">
                          <input
                            type="range"
                            min="0"
                            max="100"
                            value={actuator.value}
                            onChange={(e) => updateActuatorValue(actuator.id, e.target.value)}
                            className="slider"
                          />
                          <span className="slider-value">{actuator.value}%</span>
                        </div>
                        <button className="action-button danger" onClick={() => removeActuator(actuator.id)}>
                          Remove
                        </button>
                      </div>
                    ))
                  )}
                </div>
              </div>

              <div className="arm-control-panel">
                <h3>Utility Arms</h3>
                <div className="arm-controls-grid">
                  <div className="arm-row">
                    <span className="arm-label">Top</span>
                    <input
                      type="number"
                      min="0"
                      max="170"
                      value={topArmAngle}
                      onChange={(e) => setTopArmAngle(parseInt(e.target.value) || 0)}
                      className="arm-angle-input"
                    />
                    <button className="arm-btn" onClick={() => moveTopArm(topArmAngle)} disabled={status !== "connected"}>Go</button>
                    <button className="arm-btn" onClick={openTopArm} disabled={status !== "connected"}>Open</button>
                    <button className="arm-btn" onClick={closeTopArm} disabled={status !== "connected"}>Close</button>
                  </div>
                  <div className="arm-row">
                    <span className="arm-label">Bottom</span>
                    <input
                      type="number"
                      min="0"
                      max="170"
                      value={bottomArmAngle}
                      onChange={(e) => setBottomArmAngle(parseInt(e.target.value) || 0)}
                      className="arm-angle-input"
                    />
                    <button className="arm-btn" onClick={() => moveBottomArm(bottomArmAngle)} disabled={status !== "connected"}>Go</button>
                    <button className="arm-btn" onClick={openBottomArm} disabled={status !== "connected"}>Open</button>
                    <button className="arm-btn" onClick={closeBottomArm} disabled={status !== "connected"}>Close</button>
                  </div>
                  <div className="arm-row">
                    <span className="arm-label">Both</span>
                    <button className="arm-btn wide" onClick={openBothArms} disabled={status !== "connected"}>Open Both</button>
                    <button className="arm-btn wide" onClick={closeBothArms} disabled={status !== "connected"}>Close Both</button>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
      ) : activePage === "status" ? (
        <section className="status-board">
          <div className="board-header">
            <h2>ECU Heartbeat Map</h2>
            <div className="legend">
              <div className="legend-item">
                <span className="heartbeat-indicator good" />
                <span>{"<3s"}</span>
              </div>
              <div className="legend-item">
                <span className="heartbeat-indicator warn" />
                <span>{"3s+"}</span>
              </div>
              <div className="legend-item">
                <span className="heartbeat-indicator bad" />
                <span>{"10s+"}</span>
              </div>
            </div>
          </div>
          <div className="diagram">
            <div className="pi-row">
              <div className={`ecu-box ${piStatus.state}`}>
                <div className="ecu-box-header">
                  <div>
                    <div className="ecu-title">Raspberry Pi</div>
                    <div className="ecu-subtitle">Bridge heartbeat</div>
                  </div>
                  <span className={`heartbeat-indicator ${piStatus.state}`} />
                </div>
                <div className="ecu-meta-line">
                  {formatAge(piHeartbeatAge)} · {piStatus.label}
                </div>
              </div>
            </div>
            <div className="bus-line">
              <span>R2BUS</span>
            </div>
            <div className="ecu-row">
              {ecuNodes.length === 0 ? (
                <div className="ecu-box empty">
                  <div className="ecu-box-header">
                    <div>
                      <div className="ecu-title">No ECUs configured</div>
                      <div className="ecu-subtitle">Add entries to nodes.json</div>
                    </div>
                    <span className="heartbeat-indicator bad" />
                  </div>
                </div>
              ) : (
                ecuNodes.map((node) => (
                  <div key={node.idHex} className={`ecu-box ${node.status.state}`}>
                    <div className="ecu-box-header">
                      <div>
                        <div className="ecu-title">{node.name}</div>
                        <div className="ecu-subtitle">{node.idHex}</div>
                      </div>
                      <span className={`heartbeat-indicator ${node.status.state}`} />
                    </div>
                    <div className="ecu-meta-line">
                      {formatAge(node.age)}
                    </div>
                    <div className="ecu-actions">
                      <button
                        className="action-button"
                        onClick={() => handleResetNode(node)}
                        disabled={
                          status !== "connected" ||
                          node.id == null ||
                          Number.isNaN(Number(node.id)) ||
                          !serviceNames.includes("Empty")
                        }
                        title={
                          status !== "connected"
                            ? "Connect to the bridge to send commands"
                            : !serviceNames.includes("Empty")
                              ? "Reset message not available"
                              : undefined
                        }
                      >
                        Reset node
                      </button>
                    </div>
                  </div>
                ))
              )}
            </div>
          </div>
          <div className="ecu-footnote">
            Heartbeat thresholds: green under 3s, yellow 3-10s, red after 10s.
          </div>
        </section>
      ) : (
        <div className="grid">
        <section className="card">
          <h2>Telemetry Stream</h2>
          <div className="control-row">
            <input
              type="text"
              placeholder="Filter by topic or message"
              value={filterText}
              onChange={(e) => setFilterText(e.target.value)}
            />
          </div>
          <div className="telemetry">
            {filteredMessages.length === 0 ? (
              <div className="telemetry-item">No telemetry yet.</div>
            ) : (
              filteredMessages.map((item, index) => (
                <div className="telemetry-item" key={`${item.timestamp}-${index}`}>
                  <div className="meta">
                    <span>{formatTime(item.timestamp)}</span>
                    <span>{item.topic || item.message}</span>
                  </div>
                  <pre>{JSON.stringify(item.data, null, 2)}</pre>
                </div>
              ))
            )}
          </div>
        </section>

        <section className="card">
          <h2>Command Dispatch</h2>
          <div className="control-row">
            <select value={selectedMessage} onChange={(e) => handleMessageChange(e.target.value)}>
              <option value="">Select message</option>
              {serviceNames.map((name) => (
                <option key={name} value={name}>{name}</option>
              ))}
            </select>
            <input
              type="text"
              value={destIdInput}
              onChange={(e) => setDestIdInput(e.target.value)}
              placeholder="dest_id (0x10)"
            />
            <button className="button" onClick={handleSend}>Send</button>
          </div>
          <div className="control-row">
            <textarea
              value={payloadInput}
              onChange={(e) => setPayloadInput(e.target.value)}
              spellCheck="false"
            />
          </div>

          <h2>Event Log</h2>
          <div className="event-log">
            {events.length === 0 ? (
              <div className="event-item">No events yet.</div>
            ) : (
              events.map((event, idx) => (
                <div className="event-item" key={`${event.time}-${idx}`}>
                  {new Date(event.time).toLocaleTimeString()} - {event.text}
                </div>
              ))
            )}
          </div>
        </section>
      </div>
      )}

      <footer>
        <div>Messages: {messageNames.length} | Services: {serviceNames.length}</div>
        <div>
          UI updated:{" "}
          {lastModified ? lastModified.toLocaleString() : "unknown"}
        </div>
        <div>Tip: Use 0x10/0x11 for nodes defined in r2bus_codegen/nodes.json</div>
      </footer>
    </div>
  );
}

const root = ReactDOM.createRoot(document.getElementById("root"));
root.render(<App />);
