/* Wheeltec dashboard — roslibjs front-end */
(function () {
  'use strict';

  const MAX_POINTS = 120; // ~2 min @ 1 Hz, less at higher rates
  const $ = (id) => document.getElementById(id);

  // ---------- Connection ----------
  const defaultUrl = `ws://${location.hostname || 'localhost'}:9090`;
  const urlInput = $('ws-url');
  const statusEl = $('ws-status');
  const connectBtn = $('ws-connect');
  urlInput.value = defaultUrl;

  let ros = null;
  const subs = [];   // active subscriptions
  let cmdVelPub = null;
  let vlaInstrPub = null;  // /vla/instruction publisher
  let ttsPub = null;       // /tts_text publisher

  function setStatus(state, text) {
    statusEl.className = 'status status-' + state;
    statusEl.textContent = text;
  }

  // Track whether the current `ros` is the user's "intended" connection,
  // so that an explicit Disconnect doesn't auto-reconnect on 'close'.
  let userWantsConnected = false;

  function setButtonForState(connected) {
    connectBtn.textContent = connected ? '断开' : '连接';
  }

  function connect() {
    if (ros) {
      try { ros.close(); } catch (_) { /* ignore */ }
    }
    userWantsConnected = true;
    setButtonForState(true);
    setStatus('conn', '连接中…');
    const r = new ROSLIB.Ros({ url: urlInput.value });
    ros = r;

    r.on('connection', () => {
      if (ros !== r) return;
      setStatus('on', '已连接');
      setupTopics();
    });
    r.on('close', () => {
      // Stale close from a previously discarded ros instance — ignore.
      if (ros !== r) return;
      setStatus('off', '已断开');
      teardownTopics();
      setButtonForState(false);
      // Keep intent in sync with the visible label: after an unexpected
      // close, the button now says 连接, so the next click must take the
      // connect path — otherwise users have to click twice to reconnect.
      userWantsConnected = false;
      ros = null;
    });
    r.on('error', (err) => {
      if (ros !== r) return;
      console.error('rosbridge error', err);
      setStatus('off', '连接错误');
      // WebSocket usually fires 'close' right after 'error', but not
      // always (e.g. immediate handshake failure on some browsers).
      // Reset the same state here so the button label and intent stay
      // consistent regardless. The 'ros !== r' guard above plus setting
      // ros = null below makes a follow-up 'close' a no-op.
      teardownTopics();
      setButtonForState(false);
      userWantsConnected = false;
      ros = null;
    });
  }

  function disconnect() {
    userWantsConnected = false;
    if (ros) {
      try { ros.close(); } catch (_) { /* ignore */ }
    }
    teardownTopics();
    setStatus('off', '已断开');
    setButtonForState(false);
    ros = null;
  }

  connectBtn.addEventListener('click', () => {
    if (userWantsConnected) disconnect();
    else connect();
  });

  // ---------- Topic wiring ----------
  function sub(name, type, cb, opts) {
    const t = new ROSLIB.Topic(Object.assign({ ros, name, messageType: type }, opts || {}));
    t.subscribe(cb);
    subs.push(t);
    return t;
  }

  function teardownTopics() {
    subs.forEach((t) => { try { t.unsubscribe(); } catch (_) { /* ignore */ } });
    subs.length = 0;
    if (cmdVelPub) {
      try { cmdVelPub.unadvertise(); } catch (_) { /* ignore */ }
      cmdVelPub = null;
    }
    if (vlaInstrPub) {
      try { vlaInstrPub.unadvertise(); } catch (_) { /* ignore */ }
      vlaInstrPub = null;
    }
    if (ttsPub) {
      try { ttsPub.unadvertise(); } catch (_) { /* ignore */ }
      ttsPub = null;
    }
  }

  function setupTopics() {
    teardownTopics();

    sub('/PowerVoltage', 'std_msgs/msg/Float32', (msg) => {
      const v = msg.data;
      const el = $('m-voltage');
      el.textContent = v.toFixed(2) + ' V';
      el.classList.remove('ok', 'warn', 'err');
      // S300 uses ~24V battery pack. Tweak thresholds to your battery.
      if (v < 22) el.classList.add('err');
      else if (v < 23) el.classList.add('warn');
      else el.classList.add('ok');
      pushChart(chartVoltage, v);
    });

    sub('/robot_charging_flag', 'std_msgs/msg/Bool', (msg) => {
      const el = $('m-charging');
      el.textContent = msg.data ? '是' : '否';
      el.classList.toggle('ok', !!msg.data);
    });
    sub('/robot_charging_current', 'std_msgs/msg/Float32', (msg) => {
      $('m-charge-current').textContent = msg.data.toFixed(2) + ' A';
    });
    sub('/robot_red_flag', 'std_msgs/msg/Bool', (msg) => {
      const el = $('m-red');
      el.textContent = msg.data ? '触发' : '正常';
      el.classList.remove('ok', 'err');
      el.classList.add(msg.data ? 'err' : 'ok');
    });
    sub('/self_check_data', 'std_msgs/msg/UInt32', (msg) => {
      $('m-selfcheck').textContent = '0x' + msg.data.toString(16).toUpperCase();
    });

    sub('/odom', 'nav_msgs/msg/Odometry', (msg) => {
      const p = msg.pose.pose.position;
      const q = msg.pose.pose.orientation;
      $('m-odom-xy').textContent = `${p.x.toFixed(2)}, ${p.y.toFixed(2)} m`;
      $('m-odom-yaw').textContent = (yawFromQuat(q) * 180 / Math.PI).toFixed(1) + '°';
    }, { throttle_rate: 100 });

    sub('/imu/data_raw', 'sensor_msgs/msg/Imu', (msg) => {
      const q = msg.orientation;
      const r = rpyFromQuat(q);
      $('m-imu-rpy').textContent =
        `${(r.roll * 180 / Math.PI).toFixed(1)}, ${(r.pitch * 180 / Math.PI).toFixed(1)}, ${(r.yaw * 180 / Math.PI).toFixed(1)}°`;
    }, { throttle_rate: 100 });

    // Only A–F carry real ultrasonic data; bytes for G/H are reused by the
    // self-check payload in the firmware, so they would always read 0.
    sub('/Distance', 'robot_interfaces/msg/Supersonic', (msg) => {
      const keys = ['a', 'b', 'c', 'd', 'e', 'f'];
      keys.forEach((k) => {
        const v = msg['distance_' + k];
        const el = $('s-' + k);
        if (!el) return;
        el.textContent = (typeof v === 'number') ? v.toFixed(2) : '—';
        el.classList.remove('near', 'mid');
        if (typeof v === 'number') {
          if (v < 0.3) el.classList.add('near');
          else if (v < 0.6) el.classList.add('mid');
        }
      });
    }, { throttle_rate: 100 });

    cmdVelPub = new ROSLIB.Topic({
      ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist',
    });
    cmdVelPub.advertise();

    // VLA: publish instructions / TTS text, watch recognition + status.
    vlaInstrPub = new ROSLIB.Topic({
      ros, name: '/vla/instruction', messageType: 'std_msgs/msg/String',
    });
    vlaInstrPub.advertise();
    ttsPub = new ROSLIB.Topic({
      ros, name: '/tts_text', messageType: 'std_msgs/msg/String',
    });
    ttsPub.advertise();

    sub('/voice_words', 'std_msgs/msg/String', (msg) => {
      const el = $('vla-heard');
      if (el) el.textContent = msg.data || '—';
    });
    sub('/tts_text', 'std_msgs/msg/String', (msg) => {
      const el = $('vla-say');
      if (el) el.textContent = msg.data || '—';
    });
    sub('/vla/status', 'std_msgs/msg/String', (msg) => {
      addVlaStatus(msg.data || '');
    });

    // Build viewer + log subscription as part of the connection lifecycle.
    rebuildViewer();
    subscribeRosout();
    refreshNodeList();
  }

  // ---------- Math helpers ----------
  function yawFromQuat(q) {
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return Math.atan2(siny_cosp, cosy_cosp);
  }
  function rpyFromQuat(q) {
    const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    let sinp = 2 * (q.w * q.y - q.z * q.x);
    sinp = Math.max(-1, Math.min(1, sinp));
    const pitch = Math.asin(sinp);
    return { roll, pitch, yaw: yawFromQuat(q) };
  }

  // ---------- Teleop ----------
  const linMax = $('lin-max');
  const angMax = $('ang-max');
  const linVal = $('lin-val');
  const angVal = $('ang-val');
  const holdMode = $('hold-mode');
  const cmdReadout = $('cmd-current');

  linMax.addEventListener('input', () => { linVal.textContent = (+linMax.value).toFixed(2); });
  angMax.addEventListener('input', () => { angVal.textContent = (+angMax.value).toFixed(2); });

  let curVx = 0, curWz = 0;
  let activeBtn = null;
  let repeatTimer = null;

  function publishCmd(vx, wz) {
    curVx = vx; curWz = wz;
    cmdReadout.textContent = `vx=${vx.toFixed(2)}, wz=${wz.toFixed(2)}`;
    if (!cmdVelPub) return;
    cmdVelPub.publish(new ROSLIB.Message({
      linear: { x: vx, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: wz },
    }));
  }

  function computeCmd(btn) {
    if (btn.dataset.stop) return { vx: 0, wz: 0 };
    const vx = (+btn.dataset.vx) * (+linMax.value);
    const wz = (+btn.dataset.wz) * (+angMax.value);
    return { vx, wz };
  }

  function startBtn(btn) {
    activeBtn = btn;
    btn.classList.add('active');
    const { vx, wz } = computeCmd(btn);
    publishCmd(vx, wz);
    if (holdMode.checked) {
      clearInterval(repeatTimer);
      repeatTimer = setInterval(() => {
        const c = computeCmd(btn);
        publishCmd(c.vx, c.wz);
      }, 100);
    }
  }
  function stopBtn() {
    if (activeBtn) {
      activeBtn.classList.remove('active');
      activeBtn = null;
    }
    clearInterval(repeatTimer);
    repeatTimer = null;
    if (holdMode.checked) publishCmd(0, 0);
  }

  document.querySelectorAll('.teleop-grid .tk').forEach((btn) => {
    btn.addEventListener('mousedown', () => startBtn(btn));
    btn.addEventListener('touchstart', (e) => { e.preventDefault(); startBtn(btn); }, { passive: false });
    btn.addEventListener('mouseup', stopBtn);
    btn.addEventListener('mouseleave', () => { if (activeBtn === btn) stopBtn(); });
    btn.addEventListener('touchend', stopBtn);
  });

  // Keyboard: WASD + space (stop). Held keys publish at 10 Hz so a single
  // dropped message can't strand the chassis. Ignore key events that come
  // from form inputs so typing in the URL / topic boxes never moves the
  // robot.
  function isTypingTarget(t) {
    if (!t || !t.tagName) return false;
    const tag = t.tagName;
    return tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || t.isContentEditable;
  }

  const heldKeys = new Set();
  let keyRepeatTimer = null;

  function computeKeyboardCmd() {
    let vxs = 0, wzs = 0;
    if (heldKeys.has('w')) vxs += 1;
    if (heldKeys.has('s')) vxs -= 1;
    if (heldKeys.has('a')) wzs += 1;
    if (heldKeys.has('d')) wzs -= 1;
    return { vx: vxs * (+linMax.value), wz: wzs * (+angMax.value) };
  }

  function stopKeyboardLoop(publishZero) {
    if (keyRepeatTimer) { clearInterval(keyRepeatTimer); keyRepeatTimer = null; }
    if (publishZero) publishCmd(0, 0);
  }

  document.addEventListener('keydown', (e) => {
    if (isTypingTarget(e.target)) return;
    const k = e.key.toLowerCase();
    if (k === ' ') {
      e.preventDefault();
      heldKeys.clear();
      stopKeyboardLoop(true);
      return;
    }
    if (!'wasd'.includes(k)) return;
    if (e.repeat) return;
    heldKeys.add(k);
    // Re-arm the repeat immediately with the new key combination.
    if (keyRepeatTimer) { clearInterval(keyRepeatTimer); keyRepeatTimer = null; }
    const c = computeKeyboardCmd();
    publishCmd(c.vx, c.wz);
    keyRepeatTimer = setInterval(() => {
      const cc = computeKeyboardCmd();
      publishCmd(cc.vx, cc.wz);
    }, 100);
  });
  // Important: keyup must NOT skip on isTypingTarget. If a user holds W,
  // then clicks into a text input and releases, the keyup fires with the
  // input as target — skipping it would leave 'w' stuck in heldKeys and
  // the 10 Hz loop driving the robot forever. Releasing a key that was
  // never added (e.g. typed inside an input) is a harmless no-op.
  document.addEventListener('keyup', (e) => {
    const k = e.key.toLowerCase();
    if (!'wasd'.includes(k)) return;
    if (!heldKeys.has(k)) return;
    heldKeys.delete(k);
    if (heldKeys.size === 0) {
      stopKeyboardLoop(true);
    }
  });
  // Browser/tab lost focus — release any held keys so the robot doesn't
  // keep going on a key we'll never see released.
  window.addEventListener('blur', () => {
    if (heldKeys.size > 0 || keyRepeatTimer) {
      heldKeys.clear();
      stopKeyboardLoop(true);
    }
  });

  // Emergency stop button: always publish (0,0), regardless of held inputs.
  const eStopBtn = $('e-stop');
  if (eStopBtn) {
    eStopBtn.addEventListener('click', () => {
      heldKeys.clear();
      stopKeyboardLoop(false);
      // Send the stop a few times in case rosbridge / WiFi drops one.
      publishCmd(0, 0);
      setTimeout(() => publishCmd(0, 0), 50);
      setTimeout(() => publishCmd(0, 0), 150);
    });
  }

  // ---------- Parameters ----------
  // rcl_interfaces/msg/ParameterType constants.
  const PT_BOOL = 1, PT_INTEGER = 2, PT_DOUBLE = 3, PT_STRING = 4;
  const TYPE_NAME = { 1: 'bool', 2: 'int', 3: 'double', 4: 'string' };
  const NAME_TYPE = { bool: PT_BOOL, int: PT_INTEGER, double: PT_DOUBLE, string: PT_STRING };

  function paramService(nodeName, kind) {
    return new ROSLIB.Service({
      ros,
      name: `${nodeName}/${kind}`,
      serviceType: kind === 'get_parameters'
        ? 'rcl_interfaces/srv/GetParameters'
        : 'rcl_interfaces/srv/SetParameters',
    });
  }

  // Build a fully-populated rcl_interfaces/ParameterValue for one of the
  // four scalar types we support, leaving the unused variants at their
  // zero values so rosbridge accepts the message.
  function buildParameterValue(typeName, rawString) {
    const t = NAME_TYPE[typeName] || PT_DOUBLE;
    const base = {
      type: t,
      bool_value: false,
      integer_value: 0,
      double_value: 0,
      string_value: '',
      byte_array_value: [],
      bool_array_value: [],
      integer_array_value: [],
      double_array_value: [],
      string_array_value: [],
    };
    switch (t) {
      case PT_BOOL: {
        const s = String(rawString).trim().toLowerCase();
        base.bool_value = (s === 'true' || s === '1' || s === 'yes' || s === 'on');
        return base;
      }
      case PT_INTEGER: {
        const n = parseInt(rawString, 10);
        if (Number.isNaN(n)) return null;
        base.integer_value = n;
        return base;
      }
      case PT_DOUBLE: {
        const n = parseFloat(rawString);
        if (Number.isNaN(n)) return null;
        base.double_value = n;
        return base;
      }
      case PT_STRING:
        base.string_value = String(rawString);
        return base;
    }
    return null;
  }

  function refreshParams() {
    if (!ros) return;
    const node = $('param-node').value.trim();
    const names = Array.from(document.querySelectorAll('#param-list .param-row'))
      .map((r) => r.dataset.param);
    const req = new ROSLIB.ServiceRequest({ names });
    paramService(node, 'get_parameters').callService(req, (res) => {
      res.values.forEach((val, i) => {
        const row = document.querySelector(`#param-list .param-row[data-param="${names[i]}"]`);
        if (!row) return;
        let v = null;
        switch (val.type) {
          case PT_INTEGER: v = val.integer_value; break;
          case PT_DOUBLE:  v = val.double_value; break;
          case PT_BOOL:    v = val.bool_value; break;
          case PT_STRING:  v = val.string_value; break;
          default: v = null;
        }
        // Auto-update data-type from the server's real type, so subsequent
        // apply uses the right ParameterValue variant even if the HTML
        // declared something else.
        if (val.type && TYPE_NAME[val.type]) {
          row.dataset.type = TYPE_NAME[val.type];
        }
        row.querySelector('.param-current').textContent =
          v === null ? '(未设置)' : `${String(v)}  [${TYPE_NAME[val.type] || '?'}]`;
        if (v !== null) row.querySelector('input').value = v;
      });
    }, (err) => {
      console.error('get_parameters failed', err);
      alert('读取参数失败：' + err);
    });
  }

  $('param-refresh').addEventListener('click', refreshParams);

  document.querySelectorAll('#param-list .param-apply').forEach((btn) => {
    btn.addEventListener('click', () => {
      if (!ros) { alert('未连接 rosbridge'); return; }
      const row = btn.closest('.param-row');
      const name = row.dataset.param;
      const typeName = row.dataset.type || 'double';
      const raw = row.querySelector('input').value;
      const value = buildParameterValue(typeName, raw);
      if (!value) { alert(`无效 ${typeName} 值: "${raw}"`); return; }
      const node = $('param-node').value.trim();
      const req = new ROSLIB.ServiceRequest({ parameters: [{ name, value }] });
      paramService(node, 'set_parameters').callService(req, (res) => {
        const ok = res.results && res.results[0] && res.results[0].successful;
        if (ok) {
          row.querySelector('.param-current').textContent = `${String(raw)}  [${typeName}]`;
        } else {
          alert('设置失败：' + (res.results && res.results[0] && res.results[0].reason || '未知'));
        }
      }, (err) => alert('设置失败：' + err));
    });
  });

  // Populate the node datalist from ros.getNodes() each time we connect.
  function refreshNodeList() {
    if (!ros || !ros.getNodes) return;
    const dl = $('node-list');
    if (!dl) return;
    ros.getNodes((nodes) => {
      dl.innerHTML = '';
      (nodes || []).slice().sort().forEach((n) => {
        const opt = document.createElement('option');
        opt.value = n;
        dl.appendChild(opt);
      });
    }, (err) => console.warn('getNodes failed', err));
  }

  // ---------- Charts ----------
  function makeChart(canvasId, label, color) {
    const ctx = $(canvasId).getContext('2d');
    return new Chart(ctx, {
      type: 'line',
      data: { labels: [], datasets: [{ label, data: [], borderColor: color, backgroundColor: color + '33', tension: 0.2, pointRadius: 0 }] },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: { ticks: { color: '#8b98a5', maxTicksLimit: 6 }, grid: { color: '#2d3845' } },
          y: { ticks: { color: '#8b98a5' }, grid: { color: '#2d3845' } },
        },
        plugins: { legend: { labels: { color: '#e6edf3' } } },
      },
    });
  }
  const chartVoltage = makeChart('chart-voltage', 'PowerVoltage (V)', '#58a6ff');
  const chartCmdvel = (function () {
    const ctx = $('chart-cmdvel').getContext('2d');
    return new Chart(ctx, {
      type: 'line',
      data: { labels: [], datasets: [
        { label: 'vx (m/s)', data: [], borderColor: '#3fb950', backgroundColor: '#3fb95033', tension: 0.2, pointRadius: 0 },
        { label: 'wz (rad/s)', data: [], borderColor: '#d29922', backgroundColor: '#d2992233', tension: 0.2, pointRadius: 0 },
      ] },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: { ticks: { color: '#8b98a5', maxTicksLimit: 6 }, grid: { color: '#2d3845' } },
          y: { ticks: { color: '#8b98a5' }, grid: { color: '#2d3845' } },
        },
        plugins: { legend: { labels: { color: '#e6edf3' } } },
      },
    });
  })();

  function pushChart(chart, value) {
    const t = new Date().toLocaleTimeString();
    const d = chart.data;
    d.labels.push(t);
    d.datasets[0].data.push(value);
    while (d.labels.length > MAX_POINTS) {
      d.labels.shift();
      d.datasets.forEach((ds) => ds.data.shift());
    }
    chart.update('none');
  }

  // Sample cmd_vel into the chart at 2 Hz.
  setInterval(() => {
    const t = new Date().toLocaleTimeString();
    const d = chartCmdvel.data;
    d.labels.push(t);
    d.datasets[0].data.push(curVx);
    d.datasets[1].data.push(curWz);
    while (d.labels.length > MAX_POINTS) {
      d.labels.shift();
      d.datasets.forEach((ds) => ds.data.shift());
    }
    chartCmdvel.update('none');
  }, 500);

  // ---------- 3D Viewer (plain three.js; ros3djs + tf2_web_republisher dropped) ----------
  // ros3djs 1.1.0's LaserScan shader crashes against the loaded three.js
  // ("Cannot read properties of undefined (reading 'getUniforms')"), and its
  // TFClient can't talk to the ROS 2 tf2_web_republisher. So we build the
  // scene with plain three.js and do TF composition client-side from /tf.
  let viewer = null;          // { scene, camera, renderer, controls, host }
  let tfClient = null;        // makeTfClient(...) instance
  const viewerLayers = [];    // disposable layer objects per rebuild
  let odomPath = null;
  let odomPathSub = null;
  const odomPoints = [];
  const MAX_PATH_POINTS = 2000;
  let viewerRaf = null;
  // Module-scoped so we never accumulate observers / listeners across
  // repeated buildViewer() calls, even if someone later removes the
  // `if (viewer) return;` short-circuit.
  let viewerResizeObserver = null;
  let viewerResizeFallback = null;

  // --- minimal quaternion / transform math for client-side TF ---
  const TF_IDENTITY = { translation: { x: 0, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } };
  function quatMul(a, b) {
    return {
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
  }
  function quatConj(q) { return { x: -q.x, y: -q.y, z: -q.z, w: q.w }; }
  function quatRotateVec(q, v) {
    const t = quatMul(quatMul(q, { x: v.x, y: v.y, z: v.z, w: 0 }), quatConj(q));
    return { x: t.x, y: t.y, z: t.z };
  }
  // compose: A = pose of X in Y, B = pose of Z in X  =>  pose of Z in Y
  function tfCompose(A, B) {
    const r = quatRotateVec(A.rotation, B.translation);
    return {
      translation: { x: r.x + A.translation.x, y: r.y + A.translation.y, z: r.z + A.translation.z },
      rotation: quatMul(A.rotation, B.rotation),
    };
  }
  function tfInverse(T) {
    const qi = quatConj(T.rotation);
    const ti = quatRotateVec(qi, { x: -T.translation.x, y: -T.translation.y, z: -T.translation.z });
    return { translation: ti, rotation: qi };
  }

  // Subscribe /tf + /tf_static via rosbridge and compose transforms ourselves,
  // replacing ROSLIB.TFClient (which needs tf2_web_republisher).
  function makeTfClient(rosConn, fixedFrameRaw) {
    const fixedFrame = (fixedFrameRaw || '').replace(/^\//, '');
    const edges = {};   // child -> { parent, translation, rotation }
    const subsList = [];
    const norm = (f) => (f || '').replace(/^\//, '');

    function ingest(msg) {
      if (!msg || !msg.transforms) return;
      msg.transforms.forEach((tr) => {
        edges[norm(tr.child_frame_id)] = {
          parent: norm(tr.header.frame_id),
          translation: tr.transform.translation,
          rotation: tr.transform.rotation,
        };
      });
    }
    function toRoot(frame) {
      let pose = TF_IDENTITY;
      let cur = frame;
      let guard = 0;
      while (edges[cur] && guard++ < 200) {
        const e = edges[cur];
        pose = tfCompose({ translation: e.translation, rotation: e.rotation }, pose);
        cur = e.parent;
      }
      return { root: cur, pose };
    }

    const tfDyn = new ROSLIB.Topic({ ros: rosConn, name: '/tf', messageType: 'tf2_msgs/msg/TFMessage', throttle_rate: 50 });
    const tfStat = new ROSLIB.Topic({ ros: rosConn, name: '/tf_static', messageType: 'tf2_msgs/msg/TFMessage' });
    tfDyn.subscribe(ingest);
    tfStat.subscribe(ingest);
    subsList.push(tfDyn, tfStat);

    return {
      fixedFrame,
      knows(frame) {
        const f = norm(frame);
        if (edges[f]) return true;
        for (const k in edges) { if (edges[k].parent === f) return true; }
        return false;
      },
      // transform of `frame` expressed in fixedFrame, or null if disconnected
      lookup(frame) {
        const f = norm(frame);
        if (f === fixedFrame) return TF_IDENTITY;
        const a = toRoot(f);
        const b = toRoot(fixedFrame);
        if (a.root !== b.root) return null;
        return tfCompose(tfInverse(b.pose), a.pose);
      },
      dispose() {
        subsList.forEach((t) => { try { t.unsubscribe(); } catch (_) { /* ignore */ } });
        subsList.length = 0;
      },
    };
  }

  // Render a LaserScan as plain THREE.Points, transformed into the fixed
  // frame via makeTfClient.
  function makeScanLayer(scanTopic) {
    const MAX_SCAN_POINTS = 6000;
    const positions = new Float32Array(MAX_SCAN_POINTS * 3);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geom.setDrawRange(0, 0);
    const mat = new THREE.PointsMaterial({ color: 0xff5555, size: 0.05 });
    const points = new THREE.Points(geom, mat);
    points.frustumCulled = false;
    if (viewer) viewer.scene.add(points);

    const sub = new ROSLIB.Topic({
      ros, name: scanTopic, messageType: 'sensor_msgs/msg/LaserScan', throttle_rate: 100,
    });
    sub.subscribe((msg) => {
      const frame = (msg.header && msg.header.frame_id) || '';
      const tf = tfClient ? tfClient.lookup(frame) : null;
      const ranges = msg.ranges || [];
      let n = 0;
      for (let i = 0; i < ranges.length && n < MAX_SCAN_POINTS; i++) {
        const r = ranges[i];
        if (!isFinite(r) || r < msg.range_min || r > msg.range_max) continue;
        const a = msg.angle_min + msg.angle_increment * i;
        let x = r * Math.cos(a), y = r * Math.sin(a), z = 0;
        if (tf) {
          const p = quatRotateVec(tf.rotation, { x, y, z });
          x = p.x + tf.translation.x; y = p.y + tf.translation.y; z = p.z + tf.translation.z;
        }
        positions[n * 3] = x; positions[n * 3 + 1] = y; positions[n * 3 + 2] = z;
        n++;
      }
      geom.attributes.position.needsUpdate = true;
      geom.setDrawRange(0, n);
      geom.computeBoundingSphere();
    });

    return {
      unsubscribe() { try { sub.unsubscribe(); } catch (_) { /* ignore */ } },
      dispose() {
        try { if (viewer) viewer.scene.remove(points); } catch (_) { /* ignore */ }
        try { geom.dispose(); mat.dispose(); } catch (_) { /* ignore */ }
      },
    };
  }

  function buildViewer() {
    if (viewer) return;
    const host = $('viewer');
    const w = host.clientWidth || 640;
    const h = host.clientHeight || 480;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0d1117);

    const camera = new THREE.PerspectiveCamera(50, w / h, 0.05, 1000);
    camera.up.set(0, 0, 1);              // ROS is Z-up
    camera.position.set(3, -3, 3);
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    renderer.setSize(w, h);
    host.innerHTML = '';
    host.appendChild(renderer.domElement);

    let controls = null;
    if (THREE.OrbitControls) {
      controls = new THREE.OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.dampingFactor = 0.1;
    }

    // Ground grid in the XY plane (GridHelper is XZ by default) + axes.
    const grid = new THREE.GridHelper(20, 40, 0x2d3845, 0x2d3845);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    scene.add(new THREE.AxesHelper(0.5));

    viewer = { scene, camera, renderer, controls, host };

    const animate = () => {
      viewerRaf = requestAnimationFrame(animate);
      if (controls) controls.update();
      renderer.render(scene, camera);
    };
    animate();

    // The host element resizes whenever cards above it expand/collapse,
    // even when the window itself didn't change — ResizeObserver catches
    // those, the old window.resize listener didn't.
    const resize = () => {
      if (!viewer) return;
      const ww = host.clientWidth || w, hh = host.clientHeight || h;
      camera.aspect = ww / hh;
      camera.updateProjectionMatrix();
      renderer.setSize(ww, hh);
    };
    if (window.ResizeObserver) {
      if (viewerResizeObserver) {
        try { viewerResizeObserver.disconnect(); } catch (_) { /* ignore */ }
      }
      viewerResizeObserver = new ResizeObserver(resize);
      viewerResizeObserver.observe(host);
    } else if (!viewerResizeFallback) {
      viewerResizeFallback = resize;
      window.addEventListener('resize', viewerResizeFallback);
    }
  }

  function setViewerStatus(text) {
    const el = $('viewer-status');
    if (el) el.textContent = text || '';
  }

  function disposeLayers() {
    viewerLayers.forEach((layer) => {
      try {
        if (layer.unsubscribe) layer.unsubscribe();
        if (layer.sn && viewer && viewer.scene) viewer.scene.remove(layer.sn);
        if (layer.dispose) layer.dispose();
      } catch (_) { /* ignore */ }
    });
    viewerLayers.length = 0;
    if (odomPathSub) { try { odomPathSub.unsubscribe(); } catch (_) {} odomPathSub = null; }
    if (odomPath && viewer) { try { viewer.scene.remove(odomPath); } catch (_) {} }
    odomPath = null;
    odomPoints.length = 0;
  }

  function rebuildViewer() {
    if (!ros) return;
    buildViewer();
    disposeLayers();

    const fixedFrame = $('vw-fixed').value.trim() || 'odom_combined';
    const scanTopic = $('vw-scan').value.trim();
    const odomTopic = $('vw-odom').value.trim();

    if (tfClient && tfClient.dispose) { try { tfClient.dispose(); } catch (_) { /* ignore */ } }
    tfClient = makeTfClient(ros, fixedFrame);

    // Warn if the chosen fixed frame never shows up in /tf — without it the
    // scan / odom layers can't be placed and the view stays empty.
    setViewerStatus(`等待 frame "${fixedFrame}" …`);
    let waited = 0;
    const statusTimer = setInterval(() => {
      waited += 500;
      if (tfClient && tfClient.knows(fixedFrame)) {
        setViewerStatus('');
        clearInterval(statusTimer);
      } else if (waited >= 5000) {
        setViewerStatus(`未收到 frame "${fixedFrame}" 的 TF — 检查 robot_state_publisher / EKF 是否启动`);
        clearInterval(statusTimer);
      }
    }, 500);
    viewerLayers.push({ dispose() { clearInterval(statusTimer); } });

    if (scanTopic) {
      viewerLayers.push(makeScanLayer(scanTopic));
    }

    // NOTE: the /map (OccupancyGrid) and URDF layers used ros3djs, which is
    // incompatible with the three.js version loaded here, so they are not
    // rendered. Scan + odom trajectory cover the common case. The /map and
    // URDF input fields are currently inert.

    if (odomTopic) {
      const lineMat = new THREE.LineBasicMaterial({ color: 0x58a6ff });
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
      odomPath = new THREE.Line(geom, lineMat);
      viewer.scene.add(odomPath);

      odomPathSub = new ROSLIB.Topic({
        ros, name: odomTopic, messageType: 'nav_msgs/msg/Odometry',
        throttle_rate: 50,
      });
      odomPathSub.subscribe((msg) => {
        const p = msg.pose.pose.position;
        odomPoints.push(p.x, p.y, p.z);
        if (odomPoints.length / 3 > MAX_PATH_POINTS) {
          odomPoints.splice(0, 3);
        }
        const arr = new Float32Array(odomPoints);
        odomPath.geometry.setAttribute('position', new THREE.BufferAttribute(arr, 3));
        odomPath.geometry.attributes.position.needsUpdate = true;
        odomPath.geometry.setDrawRange(0, odomPoints.length / 3);
        odomPath.geometry.computeBoundingSphere();
      });
    }
  }

  $('vw-apply').addEventListener('click', rebuildViewer);
  $('vw-clear-path').addEventListener('click', () => {
    odomPoints.length = 0;
    if (odomPath) {
      odomPath.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
      odomPath.geometry.setDrawRange(0, 0);
    }
  });

  // ---------- Cameras (web_video_server) ----------
  const camPort = $('cam-port');
  const camQuality = $('cam-quality');
  const camBase = $('cam-base');
  const camReload = $('cam-reload');

  function videoHost() {
    return location.hostname || 'localhost';
  }

  function videoBase() {
    // Allow the user to override the entire base URL — handy when the
    // dashboard is served behind an HTTPS reverse proxy and the bare
    // http://host:8081 stream would be blocked as mixed content.
    const override = (camBase && camBase.value || '').trim();
    if (override) return override.replace(/\/+$/, '');
    const port = (camPort && camPort.value) || '8081';
    return `${location.protocol}//${videoHost()}:${port}`;
  }

  function buildStreamUrl(topic) {
    if (!topic) return '';
    const q = Math.max(1, Math.min(100, parseInt(camQuality.value, 10) || 60));
    const params = new URLSearchParams({
      topic,
      type: 'mjpeg',
      quality: String(q),
    });
    // Bust cache so reload actually re-fetches the stream.
    params.set('_', String(Date.now()));
    return `${videoBase()}/stream?${params.toString()}`;
  }

  const CAM_PLACEHOLDER = 'placeholder.svg';

  function showPlaceholder(slot, message) {
    const img = document.querySelector(`.cam-img[data-slot="${slot}"]`);
    const errEl = document.querySelector(`.cam-err[data-slot="${slot}"]`);
    img.onerror = null;
    img.src = CAM_PLACEHOLDER;
    if (message) {
      errEl.hidden = false;
      errEl.textContent = message;
    } else {
      errEl.hidden = true;
    }
  }

  function applyCam(slot) {
    const img = document.querySelector(`.cam-img[data-slot="${slot}"]`);
    const topicInput = document.querySelector(`.cam-topic[data-slot="${slot}"]`);
    const enable = document.querySelector(`.cam-enable[data-slot="${slot}"]`);
    const errEl = document.querySelector(`.cam-err[data-slot="${slot}"]`);
    if (!img || !topicInput || !enable) return;
    errEl.hidden = true;
    if (!enable.checked) {
      showPlaceholder(slot, '已禁用');
      return;
    }
    const topic = topicInput.value.trim();
    if (!topic) {
      showPlaceholder(slot, '未设置 topic');
      return;
    }
    img.onerror = () => {
      // Swap to landscape placeholder so the tile stays presentable.
      showPlaceholder(slot, '无法加载流，检查 web_video_server 与相机话题');
    };
    img.onload = () => { errEl.hidden = true; };
    img.src = buildStreamUrl(topic);
  }

  function applyAllCams() {
    applyCam('car');
    applyCam('arm');
  }

  camReload.addEventListener('click', applyAllCams);
  document.querySelectorAll('.cam-topic').forEach((el) => {
    el.addEventListener('change', () => applyCam(el.dataset.slot));
  });
  document.querySelectorAll('.cam-enable').forEach((el) => {
    el.addEventListener('change', () => applyCam(el.dataset.slot));
  });
  camPort.addEventListener('change', applyAllCams);
  camQuality.addEventListener('change', applyAllCams);
  if (camBase) camBase.addEventListener('change', applyAllCams);

  // Start streams once on load (they're independent of rosbridge).
  applyAllCams();

  // ---------- VLA voice navigation ----------
  const VLA_MAX_LINES = 200;

  function addVlaStatus(text) {
    const view = $('vla-timeline');
    if (!view) return;
    const row = document.createElement('div');
    row.className = 'vla-line';
    const t = document.createElement('span');
    t.className = 'vt';
    t.textContent = new Date().toLocaleTimeString();
    const m = document.createElement('span');
    m.className = 'vm';
    m.textContent = text;          // textContent: never inject markup from ROS
    row.appendChild(t);
    row.appendChild(m);
    view.appendChild(row);
    while (view.childElementCount > VLA_MAX_LINES) view.removeChild(view.firstChild);
    view.scrollTop = view.scrollHeight;
  }

  function sendInstruction(text) {
    const t = (text || '').trim();
    if (!t) return;
    if (!vlaInstrPub) { addVlaStatus('未连接 rosbridge，无法发送指令'); return; }
    vlaInstrPub.publish(new ROSLIB.Message({ data: t }));
    addVlaStatus('> 发送指令: ' + t);
  }

  function sendTts(text) {
    const t = (text || '').trim();
    if (!t) return;
    if (!ttsPub) { addVlaStatus('未连接 rosbridge，无法播报'); return; }
    ttsPub.publish(new ROSLIB.Message({ data: t }));
  }

  const vlaInstrInput = $('vla-instr');
  const vlaSendBtn = $('vla-send');
  if (vlaSendBtn) vlaSendBtn.addEventListener('click', () => sendInstruction(vlaInstrInput.value));
  if (vlaInstrInput) vlaInstrInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') { sendInstruction(vlaInstrInput.value); vlaInstrInput.value = ''; }
  });
  document.querySelectorAll('.vla-q').forEach((b) => {
    b.addEventListener('click', () => sendInstruction(b.dataset.instr));
  });

  const ttsInput = $('tts-input');
  const ttsSendBtn = $('tts-send');
  if (ttsSendBtn) ttsSendBtn.addEventListener('click', () => sendTts(ttsInput.value));
  if (ttsInput) ttsInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') { sendTts(ttsInput.value); ttsInput.value = ''; }
  });

  const vlaClearBtn = $('vla-clear');
  if (vlaClearBtn) vlaClearBtn.addEventListener('click', () => {
    const v = $('vla-timeline');
    if (v) v.innerHTML = '';
  });

  // ---------- Logs (/rosout) ----------
  const logView = $('log-view');
  const logLevel = $('log-level');
  const logFilter = $('log-filter');
  const logBuffer = $('log-buffer');
  const logAutoscroll = $('log-autoscroll');
  const logPauseBtn = $('log-pause');
  const logClearBtn = $('log-clear');
  const logCountEl = $('log-count');
  const logDroppedEl = $('log-dropped');

  const LEVEL_NAME = { 10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL' };
  const LEVEL_CLASS = { 10: 'log-debug', 20: 'log-info', 30: 'log-warn', 40: 'log-error', 50: 'log-fatal' };

  let logSub = null;
  let logPaused = false;
  let logDropped = 0;
  const logEntries = []; // ring buffer of {level,name,msg,timeStr}

  function fmtTime(stamp) {
    // stamp: {sec, nanosec}
    if (!stamp) return '';
    const ms = stamp.sec * 1000 + Math.floor(stamp.nanosec / 1e6);
    const d = new Date(ms);
    const pad = (n, w = 2) => String(n).padStart(w, '0');
    return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`;
  }

  function escapeHTML(s) {
    return String(s).replace(/[&<>"']/g, (c) => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
    }[c]));
  }

  function renderRow(e) {
    const cls = LEVEL_CLASS[e.level] || 'log-info';
    const lv = LEVEL_NAME[e.level] || String(e.level);
    return `<div class="log-row ${cls}"><span class="lt">${e.timeStr}</span>` +
      `<span class="lv">${lv}</span>` +
      `<span class="ln" title="${escapeHTML(e.name)}">${escapeHTML(e.name)}</span>` +
      `<span class="lm">${escapeHTML(e.msg)}</span></div>`;
  }

  function matchesFilters(e) {
    if (e.level < +logLevel.value) return false;
    const f = logFilter.value.trim().toLowerCase();
    if (f && !e.name.toLowerCase().includes(f)) return false;
    return true;
  }

  function rerenderAll() {
    logView.innerHTML = logEntries.filter(matchesFilters).map(renderRow).join('');
    logCountEl.textContent = String(logEntries.length);
    if (logAutoscroll.checked) logView.scrollTop = logView.scrollHeight;
  }

  function pushEntry(e) {
    const cap = Math.max(50, Math.min(5000, parseInt(logBuffer.value, 10) || 500));
    logEntries.push(e);
    if (logEntries.length > cap) {
      logEntries.splice(0, logEntries.length - cap);
    }
    logCountEl.textContent = String(logEntries.length);
    if (!matchesFilters(e)) return;
    // Cheap append: also trim rendered children to ~cap.
    logView.insertAdjacentHTML('beforeend', renderRow(e));
    while (logView.childElementCount > cap) logView.removeChild(logView.firstChild);
    if (logAutoscroll.checked) logView.scrollTop = logView.scrollHeight;
  }

  function subscribeRosout() {
    if (logSub) { try { logSub.unsubscribe(); } catch (_) {} }
    logSub = new ROSLIB.Topic({
      ros,
      name: '/rosout',
      messageType: 'rcl_interfaces/msg/Log',
      throttle_rate: 0,
      queue_length: 0,
    });
    logSub.subscribe((msg) => {
      if (logPaused) { logDropped++; logDroppedEl.textContent = String(logDropped); return; }
      pushEntry({
        level: msg.level,
        name: msg.name || '',
        msg: msg.msg || '',
        timeStr: fmtTime(msg.stamp),
      });
    });
  }

  logLevel.addEventListener('change', rerenderAll);
  logFilter.addEventListener('input', rerenderAll);
  logBuffer.addEventListener('change', () => {
    const cap = Math.max(50, Math.min(5000, parseInt(logBuffer.value, 10) || 500));
    if (logEntries.length > cap) logEntries.splice(0, logEntries.length - cap);
    rerenderAll();
  });
  logPauseBtn.addEventListener('click', () => {
    logPaused = !logPaused;
    logPauseBtn.textContent = logPaused ? '继续' : '暂停';
    logPauseBtn.classList.toggle('paused', logPaused);
    if (!logPaused) { logDropped = 0; logDroppedEl.textContent = '0'; }
  });
  logClearBtn.addEventListener('click', () => {
    logEntries.length = 0;
    logDropped = 0;
    logDroppedEl.textContent = '0';
    logCountEl.textContent = '0';
    logView.innerHTML = '';
  });

  // Auto-connect on load.
  connect();
})();
