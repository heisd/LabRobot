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
      ros = null;
    });
    r.on('error', (err) => {
      if (ros !== r) return;
      console.error('rosbridge error', err);
      setStatus('off', '连接错误');
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

    // Build viewer + log subscription as part of the connection lifecycle.
    rebuildViewer();
    subscribeRosout();
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
  document.addEventListener('keyup', (e) => {
    if (isTypingTarget(e.target)) return;
    const k = e.key.toLowerCase();
    if (!'wasd'.includes(k)) return;
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
  function paramService(nodeName, kind) {
    return new ROSLIB.Service({
      ros,
      name: `${nodeName}/${kind}`,
      serviceType: kind === 'get_parameters'
        ? 'rcl_interfaces/srv/GetParameters'
        : 'rcl_interfaces/srv/SetParameters',
    });
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
          case 2: v = val.integer_value; break;
          case 3: v = val.double_value; break;
          case 1: v = val.bool_value; break;
          case 4: v = val.string_value; break;
          default: v = null;
        }
        row.querySelector('.param-current').textContent = v === null ? '(未设置)' : String(v);
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
      const value = parseFloat(row.querySelector('input').value);
      if (Number.isNaN(value)) { alert('无效数字'); return; }
      const node = $('param-node').value.trim();
      const req = new ROSLIB.ServiceRequest({
        parameters: [{
          name,
          value: { type: 3, double_value: value, bool_value: false, integer_value: 0, string_value: '', byte_array_value: [], bool_array_value: [], integer_array_value: [], double_array_value: [], string_array_value: [] },
        }],
      });
      paramService(node, 'set_parameters').callService(req, (res) => {
        const ok = res.results && res.results[0] && res.results[0].successful;
        if (ok) {
          row.querySelector('.param-current').textContent = String(value);
        } else {
          alert('设置失败：' + (res.results && res.results[0] && res.results[0].reason || '未知'));
        }
      }, (err) => alert('设置失败：' + err));
    });
  });

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

  // ---------- 3D Viewer (ros3djs) ----------
  let viewer = null;
  let tfClient = null;
  const viewerLayers = []; // disposable client objects per rebuild
  let odomPath = null;
  let odomPathSub = null;
  const odomPoints = [];
  const MAX_PATH_POINTS = 2000;

  function buildViewer() {
    if (viewer) return;
    const host = $('viewer');
    viewer = new ROS3D.Viewer({
      divID: 'viewer',
      width: host.clientWidth,
      height: host.clientHeight,
      antialias: true,
      background: '#0d1117',
      cameraPose: { x: 3, y: 3, z: 3 },
    });
    viewer.addObject(new ROS3D.Grid({ color: 0x2d3845, cellSize: 0.5, num_cells: 20 }));

    // The host element resizes whenever cards above it expand/collapse,
    // even when the window itself didn't change — ResizeObserver catches
    // those, the old window.resize listener didn't.
    const resize = () => { if (viewer) viewer.resize(host.clientWidth, host.clientHeight); };
    if (window.ResizeObserver) {
      new ResizeObserver(resize).observe(host);
    } else {
      window.addEventListener('resize', resize);
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
    const mapTopic = $('vw-map').value.trim();
    const odomTopic = $('vw-odom').value.trim();
    const urdfSpec = $('vw-urdf-param').value.trim();

    tfClient = new ROSLIB.TFClient({
      ros,
      fixedFrame,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
    });

    // Warn the user if the chosen fixed frame never shows up — without
    // it, scans / maps / odom layers stay invisible with no feedback.
    setViewerStatus(`等待 frame "${fixedFrame}" …`);
    let frameSeen = false;
    const tfProbe = new ROSLIB.Topic({
      ros, name: '/tf', messageType: 'tf2_msgs/msg/TFMessage', throttle_rate: 200,
    });
    const tfProbeStaticSub = new ROSLIB.Topic({
      ros, name: '/tf_static', messageType: 'tf2_msgs/msg/TFMessage',
    });
    const onTf = (msg) => {
      if (frameSeen || !msg || !msg.transforms) return;
      for (const tr of msg.transforms) {
        if (tr.header && (tr.header.frame_id === fixedFrame || tr.child_frame_id === fixedFrame)) {
          frameSeen = true;
          setViewerStatus('');
          try { tfProbe.unsubscribe(); } catch (_) {}
          try { tfProbeStaticSub.unsubscribe(); } catch (_) {}
          break;
        }
      }
    };
    tfProbe.subscribe(onTf);
    tfProbeStaticSub.subscribe(onTf);
    viewerLayers.push(tfProbe, tfProbeStaticSub);
    setTimeout(() => {
      if (!frameSeen) {
        setViewerStatus(`未收到 frame "${fixedFrame}" 的 TF — 检查 robot_state_publisher / EKF 是否启动`);
      }
    }, 4000);

    if (scanTopic) {
      const scan = new ROS3D.LaserScan({
        ros, tfClient,
        topic: scanTopic,
        rootObject: viewer.scene,
        material: { size: 0.05, color: 0xff5555 },
      });
      viewerLayers.push(scan);
    }

    if (mapTopic) {
      const map = new ROS3D.OccupancyGridClient({
        ros, tfClient,
        topic: mapTopic,
        rootObject: viewer.scene,
        continuous: true,
      });
      viewerLayers.push(map);
    }

    if (urdfSpec) {
      // ros3djs UrdfClient pulls from a parameter on a node.
      // Format accepts "node:param" or just a topic name -> fallback to subscribe.
      try {
        let nodeName = '/robot_state_publisher';
        let paramName = 'robot_description';
        if (urdfSpec.includes(':')) {
          const parts = urdfSpec.split(':');
          nodeName = parts[0];
          paramName = parts[1];
        }
        const urdfClient = new ROS3D.UrdfClient({
          ros, tfClient,
          path: 'https://cdn.jsdelivr.net/gh/ros/urdf_tutorial@master/',
          rootObject: viewer.scene,
          parameter: paramName,
          parameterNode: nodeName,
          loader: ROS3D.COLLADA_LOADER,
        });
        viewerLayers.push(urdfClient);
      } catch (e) {
        console.warn('UrdfClient failed:', e);
      }
    }

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
  const camReload = $('cam-reload');

  function videoHost() {
    return location.hostname || 'localhost';
  }

  function buildStreamUrl(topic) {
    if (!topic) return '';
    const port = camPort.value || '8081';
    const q = Math.max(1, Math.min(100, parseInt(camQuality.value, 10) || 60));
    const params = new URLSearchParams({
      topic,
      type: 'mjpeg',
      quality: String(q),
    });
    // Bust cache so reload actually re-fetches the stream.
    params.set('_', String(Date.now()));
    return `http://${videoHost()}:${port}/stream?${params.toString()}`;
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

  // Start streams once on load (they're independent of rosbridge).
  applyAllCams();

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
