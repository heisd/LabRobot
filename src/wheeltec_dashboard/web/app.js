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

  function connect() {
    if (ros) {
      try { ros.close(); } catch (_) { /* ignore */ }
    }
    setStatus('conn', '连接中…');
    ros = new ROSLIB.Ros({ url: urlInput.value });

    ros.on('connection', () => {
      setStatus('on', '已连接');
      setupTopics();
    });
    ros.on('close', () => {
      setStatus('off', '已断开');
      teardownTopics();
    });
    ros.on('error', (err) => {
      console.error('rosbridge error', err);
      setStatus('off', '连接错误');
    });
  }

  connectBtn.addEventListener('click', connect);

  // ---------- Topic wiring ----------
  function sub(name, type, cb) {
    const t = new ROSLIB.Topic({ ros, name, messageType: type });
    t.subscribe(cb);
    subs.push(t);
    return t;
  }

  function teardownTopics() {
    subs.forEach((t) => { try { t.unsubscribe(); } catch (_) { /* ignore */ } });
    subs.length = 0;
    cmdVelPub = null;
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
    });

    sub('/imu/data_raw', 'sensor_msgs/msg/Imu', (msg) => {
      const q = msg.orientation;
      const r = rpyFromQuat(q);
      $('m-imu-rpy').textContent =
        `${(r.roll * 180 / Math.PI).toFixed(1)}, ${(r.pitch * 180 / Math.PI).toFixed(1)}, ${(r.yaw * 180 / Math.PI).toFixed(1)}°`;
    });

    sub('/Distance', 'robot_interfaces/msg/Supersonic', (msg) => {
      const keys = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'];
      keys.forEach((k) => {
        const v = msg['distance_' + k];
        const el = $('s-' + k);
        el.textContent = (typeof v === 'number') ? v.toFixed(2) : '—';
        el.classList.remove('near', 'mid');
        if (v < 0.3) el.classList.add('near');
        else if (v < 0.6) el.classList.add('mid');
      });
    });

    cmdVelPub = new ROSLIB.Topic({
      ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist',
    });
    cmdVelPub.advertise();
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

  // Keyboard: WASD + space (stop)
  document.addEventListener('keydown', (e) => {
    if (e.repeat) return;
    const map = { w: [1, 0], s: [-1, 0], a: [0, 1], d: [0, -1] };
    const k = e.key.toLowerCase();
    if (map[k]) {
      const [vxs, wzs] = map[k];
      publishCmd(vxs * (+linMax.value), wzs * (+angMax.value));
    } else if (k === ' ') {
      publishCmd(0, 0);
    }
  });
  document.addEventListener('keyup', (e) => {
    const k = e.key.toLowerCase();
    if ('wasd'.includes(k)) publishCmd(0, 0);
  });

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

    window.addEventListener('resize', () => {
      if (!viewer) return;
      viewer.resize(host.clientWidth, host.clientHeight);
    });
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

  // Build the viewer right after the connection is up.
  const _origSetup = setupTopics;
  // eslint-disable-next-line no-func-assign
  setupTopics = function () { _origSetup(); rebuildViewer(); };

  // Auto-connect on load.
  connect();
})();
