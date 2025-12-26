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
    return { state: "bad", label: "no heartbeat" };
  }
  if (msAgo < HEARTBEAT_OK_MS) {
    return { state: "good", label: "<3s" };
  }
  if (msAgo < HEARTBEAT_WARN_MS) {
    return { state: "warn", label: "3s+" };
  }
  return { state: "bad", label: "10s+" };
}

function App() {
  const [bridgeUrl, setBridgeUrl] = useState(buildDefaultUrl());
  const [status, setStatus] = useState("disconnected");
  const [messages, setMessages] = useState([]);
  const [messageNames, setMessageNames] = useState([]);
  const [serviceNames, setServiceNames] = useState([]);
  const [nodeMap, setNodeMap] = useState({});
  const [nodeHeartbeat, setNodeHeartbeat] = useState({});
  const [piLastSeen, setPiLastSeen] = useState(null);
  const [piHeartbeat, setPiHeartbeat] = useState(null);
  const [activePage, setActivePage] = useState(
    window.location.hash === "#status" ? "status" : "console"
  );
  const [tick, setTick] = useState(0);
  const [selectedMessage, setSelectedMessage] = useState("");
  const [destIdInput, setDestIdInput] = useState("0x10");
  const [payloadInput, setPayloadInput] = useState("{}");
  const [filterText, setFilterText] = useState("");
  const [events, setEvents] = useState([]);
  const [lastModified, setLastModified] = useState(null);
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
      setActivePage(window.location.hash === "#status" ? "status" : "console");
    };
    window.addEventListener("hashchange", onHashChange);
    return () => window.removeEventListener("hashchange", onHashChange);
  }, []);

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
        setPiLastSeen(Date.now());
        if (!selectedMessage && payload.services && payload.services.length) {
          setSelectedMessage(payload.services[0]);
          setPayloadInput(JSON.stringify(SAMPLE_PAYLOADS[payload.services[0]] || {}, null, 2));
        }
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

  function setPage(nextPage) {
    const hash = nextPage === "status" ? "status" : "";
    window.location.hash = hash;
    setActivePage(nextPage);
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
              className={`nav-button ${activePage === "status" ? "active" : ""}`}
              onClick={() => setPage("status")}
            >
              ECU Status
            </button>
          </div>
        </div>
        <div className="status">
          <span className={`status-dot ${status === "connected" ? "connected" : ""}`}></span>
          {status}
        </div>
      </header>

      {activePage === "status" ? (
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
                      {formatAge(node.age)} · {node.status.label}
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
            <input
              type="text"
              value={bridgeUrl}
              onChange={(e) => setBridgeUrl(e.target.value)}
              placeholder="ws://host:8090/ws"
            />
            <button className="button secondary" onClick={connectWebSocket}>Reconnect</button>
          </div>
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
