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
  let armVlmInstrPub = null;   // /vlm/instruction publisher (机械臂 VLM 抓取)
  let armVlmConfirmPub = null; // /vlm/confirm publisher (确认/取消抓取)
  let lidarSubs = [];      // per-source LaserScan subs (managed separately so
                           // the lidar card can re-subscribe on topic change)
  const lidarLast = {};    // key -> { time, points, min }
  let yoloDetSub = null;   // optional yolo_msgs/DetectionArray sub (yolo_ros)

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
    lidarSubs.forEach((t) => { try { t.unsubscribe(); } catch (_) { /* ignore */ } });
    lidarSubs = [];
    if (yoloDetSub) {
      try { yoloDetSub.unsubscribe(); } catch (_) { /* ignore */ }
      yoloDetSub = null;
    }
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
    if (armVlmInstrPub) {
      try { armVlmInstrPub.unadvertise(); } catch (_) { /* ignore */ }
      armVlmInstrPub = null;
    }
    if (armVlmConfirmPub) {
      try { armVlmConfirmPub.unadvertise(); } catch (_) { /* ignore */ }
      armVlmConfirmPub = null;
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
      const txt = msg.data || '—';
      const a = $('vla-heard'); if (a) a.textContent = txt;
      const b = $('voice-words'); if (b) b.textContent = txt;
    });
    sub('/tts_text', 'std_msgs/msg/String', (msg) => {
      const el = $('vla-say');
      if (el) el.textContent = msg.data || '—';
    });
    sub('/vla/status', 'std_msgs/msg/String', (msg) => {
      addVlaStatus(msg.data || '');
    });
    // cmd_arbiter 当前控制源(键盘/巡线/KCF/YOLO/停车)
    sub('/cmd_arbiter/status', 'std_msgs/msg/String', (msg) => updateCtrlSource(msg.data));

    // Voice subsystem status (wheeltec_mic + tts_make).
    sub('/voice_flag', 'std_msgs/msg/Int8', (msg) => {
      const el = $('voice-mic'); if (!el) return;
      const ok = msg.data === 1 || msg.data === true;
      el.textContent = ok ? '已初始化' : '未就绪';
      el.classList.remove('ok', 'err');
      el.classList.add(ok ? 'ok' : 'err');
    });
    sub('/awake_flag', 'std_msgs/msg/Int8', (msg) => {
      const el = $('voice-awake'); if (!el) return;
      const awake = msg.data === 1 || msg.data === true;
      el.textContent = awake ? '已唤醒' : '休眠';
      el.classList.remove('ok', 'warn');
      el.classList.add(awake ? 'ok' : 'warn');
    });
    sub('/awake_angle', 'std_msgs/msg/UInt32', (msg) => {
      const el = $('voice-angle'); if (el) el.textContent = msg.data + ' °';
    });

    // cmd_vel monitor → line-follow / KCF readouts (whatever drives the bus).
    sub('/cmd_vel', 'geometry_msgs/msg/Twist', (msg) => {
      const vx = (msg.linear && msg.linear.x) || 0;
      const wz = (msg.angular && msg.angular.z) || 0;
      const txt = `vx=${(+vx).toFixed(2)}, wz=${(+wz).toFixed(2)}`;
      const a = $('line-cmd'); if (a) a.textContent = txt;
      const b = $('kcf-cmd'); if (b) b.textContent = txt;
    }, { throttle_rate: 100 });

    // QR line-following (line_follow_qr_fixed: qr_detector + cmd_arbiter).
    sub('/qr_code/detected', 'std_msgs/msg/Bool', (msg) => {
      const el = $('line-qr-detected'); if (!el) return;
      const yes = msg.data === true || msg.data === 1;
      el.textContent = yes ? '检测到' : '未检测';
      el.classList.remove('ok', 'warn');
      el.classList.add(yes ? 'ok' : 'warn');
    });
    sub('/qr_code/data', 'std_msgs/msg/String', (msg) => {
      const el = $('line-qr-data'); if (el) el.textContent = msg.data || '—';
    });
    sub('/qr_code/area_ratio', 'std_msgs/msg/Float32', (msg) => {
      const el = $('line-qr-area');
      if (el) el.textContent = (typeof msg.data === 'number') ? (msg.data * 100).toFixed(1) + ' %' : '—';
    });

    // 机械臂 (lebai_driver + grab_demo): 状态订阅 + VLM 抓取指令发布。
    armVlmInstrPub = new ROSLIB.Topic({
      ros, name: '/vlm/instruction', messageType: 'std_msgs/msg/String',
    });
    armVlmInstrPub.advertise();
    armVlmConfirmPub = new ROSLIB.Topic({
      ros, name: '/vlm/confirm', messageType: 'std_msgs/msg/Bool',
    });
    armVlmConfirmPub.advertise();

    sub('/robot_status', 'lebai_interfaces/msg/RobotStatus', renderArmStatus, { throttle_rate: 200 });
    sub('/gripper_status', 'lebai_interfaces/msg/GripperStatus', (msg) => {
      setArmMetric('arm-grip-pos', (+msg.position).toFixed(0));
      setArmMetric('arm-grip-force', (+msg.force).toFixed(0));
    }, { throttle_rate: 200 });
    sub('/joint_states', 'sensor_msgs/msg/JointState', ingestArmJointState, { throttle_rate: 100 });
    sub('/grab_target/distance', 'std_msgs/msg/Float32', (msg) => {
      armDistTime = Date.now();
      setArmDist((+msg.data).toFixed(3) + ' m');
    }, { throttle_rate: 200 });
    sub('/arm_arbiter/state', 'std_msgs/msg/String', (msg) => {
      const s = (msg.data || '').trim();
      // 状态卡片与"抓取与仲裁"卡片各有一份显示。
      ['arm-arb-state', 'arm-arb-state2'].forEach((id) => {
        if (s === 'manual') setArmMetric(id, '手动接管', 'warn');
        else if (s === 'idle') setArmMetric(id, '自动/空闲', 'ok');
        else setArmMetric(id, s || '—');
      });
    });
    sub('/vlm/result', 'std_msgs/msg/String', (msg) => {
      const el = $('arm-vlm-result');
      if (el) el.textContent = msg.data || '—';
      if (msg.data) addArmLog('VLM: ' + msg.data);
    });

    // Lidar health (fused + per-sensor) and optional YOLO detections.
    subscribeLidar();
    subscribeYolo();

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

  // Generic parameter editor. Each `.param-group` carries its own node input
  // (`.pg-node`), refresh button (`.pg-refresh`) and a set of
  // `.param-row[data-param]` rows, so the same logic drives both the robot
  // params (/wheeltec_robot) and the KCF PID params (/image_converter).
  function initParamGroup(root) {
    const nodeInput = root.querySelector('.pg-node');
    if (!nodeInput) return;
    const refreshBtn = root.querySelector('.pg-refresh');
    const rows = () => Array.from(root.querySelectorAll('.param-row[data-param]'));

    function refresh() {
      if (!ros) { alert('未连接 rosbridge'); return; }
      const node = nodeInput.value.trim();
      const rs = rows();
      const names = rs.map((r) => r.dataset.param);
      const req = new ROSLIB.ServiceRequest({ names });
      paramService(node, 'get_parameters').callService(req, (res) => {
        res.values.forEach((val, i) => {
          const row = rs[i];
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
          if (val.type && TYPE_NAME[val.type]) row.dataset.type = TYPE_NAME[val.type];
          row.querySelector('.param-current').textContent =
            v === null ? '(未设置)' : `${String(v)}  [${TYPE_NAME[val.type] || '?'}]`;
          if (v !== null) row.querySelector('input').value = v;
        });
      }, (err) => {
        console.error('get_parameters failed', err);
        alert('读取参数失败：' + err);
      });
    }

    if (refreshBtn) refreshBtn.addEventListener('click', refresh);

    root.querySelectorAll('.param-apply').forEach((btn) => {
      btn.addEventListener('click', () => {
        if (!ros) { alert('未连接 rosbridge'); return; }
        const row = btn.closest('.param-row');
        const name = row.dataset.param;
        const typeName = row.dataset.type || 'double';
        const raw = row.querySelector('input').value;
        const value = buildParameterValue(typeName, raw);
        if (!value) { alert(`无效 ${typeName} 值: "${raw}"`); return; }
        const node = nodeInput.value.trim();
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
  }

  document.querySelectorAll('.param-group').forEach(initParamGroup);

  // Live "distance" slider: drag -> set_parameters(double) on a node, no restart.
  // Drives YOLO follow (/yolo_follow desired_distance) and KCF (/image_converter
  // targetDist_). Reuses paramService + buildParameterValue above.
  function initDistanceSlider(root) {
    const param = root.dataset.param;
    const range = root.querySelector('.ds-range');
    const valEl = root.querySelector('.ds-val');
    const getBtn = root.querySelector('.ds-get');
    if (!param || !range) return;
    const node = () => (root.dataset.node || '').trim();
    const show = (v) => { if (valEl) valEl.textContent = Number(v).toFixed(2) + ' m'; };
    const sendDist = (v) => {
      if (!ros) return;
      const value = buildParameterValue('double', String(v));
      if (!value) return;
      const req = new ROSLIB.ServiceRequest({ parameters: [{ name: param, value }] });
      paramService(node(), 'set_parameters').callService(req, () => {},
        (err) => console.error(`set ${param} failed`, err));
    };
    let timer = null;
    range.addEventListener('input', () => {
      show(range.value);
      if (timer) clearTimeout(timer);     // debounce while dragging
      timer = setTimeout(() => { timer = null; sendDist(range.value); }, 120);
    });
    range.addEventListener('change', () => {
      if (timer) { clearTimeout(timer); timer = null; }   // release: send once, drop pending
      sendDist(range.value);
    });
    if (getBtn) getBtn.addEventListener('click', () => {
      if (!ros) { alert('未连接 rosbridge'); return; }
      const req = new ROSLIB.ServiceRequest({ names: [param] });
      paramService(node(), 'get_parameters').callService(req, (res) => {
        const val = res.values && res.values[0];
        if (val && (val.type === PT_DOUBLE || val.type === PT_INTEGER)) {
          const v = val.type === PT_DOUBLE ? val.double_value : val.integer_value;
          range.value = v; show(v);
        }
      }, (err) => alert('读取失败：' + err));
    });
    show(range.value);
  }
  document.querySelectorAll('.dist-slider').forEach(initDistanceSlider);

  // cmd_arbiter control-source badge. The arbiter owns the semantics; we just
  // render its string and color it (keyboard=alert, idle/stop=muted, else active).
  let ctrlSrcTime = 0;
  function updateCtrlSource(text) {
    const el = $('ctrl-source');
    if (!el) return;
    ctrlSrcTime = Date.now();
    const t = String(text || '').trim();
    el.textContent = '控制源: ' + (t || '—');
    el.classList.remove('ctrl-src-idle', 'ctrl-src-active', 'ctrl-src-kbd');
    if (/键盘/.test(t)) el.classList.add('ctrl-src-kbd');
    else if (!t || /停车|无控制源|idle/i.test(t)) el.classList.add('ctrl-src-idle');
    else el.classList.add('ctrl-src-active');
  }
  // Arbiter heartbeats ~1Hz; if it goes quiet (not running/stopped), reset to "—".
  setInterval(() => {
    if (ctrlSrcTime && Date.now() - ctrlSrcTime > 2500) {
      ctrlSrcTime = 0;
      const el = $('ctrl-source');
      if (el) {
        el.textContent = '控制源: —';
        el.classList.remove('ctrl-src-active', 'ctrl-src-kbd');
        el.classList.add('ctrl-src-idle');
      }
    }
  }, 1000);

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

  // ---------- Function-page MJPEG streams (line / KCF / YOLO) ----------
  // These reuse the camera toolbar's port/quality/base settings through
  // buildStreamUrl(), but each function page points its own <img> at a topic.
  function applyFnStream(img) {
    if (!img) return;
    const topic = (img.dataset.topic || '').trim();
    const frame = img.closest('.cam-frame');
    const errEl = frame ? frame.querySelector('.cam-err') : null;
    if (!topic) {
      img.onerror = null;
      img.src = CAM_PLACEHOLDER;
      if (errEl) { errEl.hidden = false; errEl.textContent = '未设置 topic'; }
      return;
    }
    img.onerror = () => {
      img.onerror = null;
      img.src = CAM_PLACEHOLDER;
      if (errEl) { errEl.hidden = false; errEl.textContent = '无法加载流，检查 web_video_server 与话题'; }
    };
    img.onload = () => { if (errEl) errEl.hidden = true; };
    img.src = buildStreamUrl(topic);
  }
  function applyAllFnStreams() {
    document.querySelectorAll('.fn-img').forEach(applyFnStream);
  }
  document.querySelectorAll('.fn-topic').forEach((inp) => {
    inp.addEventListener('change', () => {
      const img = $(inp.dataset.img);
      if (img) { img.dataset.topic = inp.value.trim(); applyFnStream(img); }
    });
  });
  document.querySelectorAll('.fn-reload').forEach((btn) => {
    btn.addEventListener('click', () => {
      const img = $(btn.dataset.img);
      if (img) applyFnStream(img);
    });
  });
  // Keep function streams in sync when the shared camera port/quality changes.
  camReload.addEventListener('click', applyAllFnStreams);
  camPort.addEventListener('change', applyAllFnStreams);
  camQuality.addEventListener('change', applyAllFnStreams);
  if (camBase) camBase.addEventListener('change', applyAllFnStreams);
  // Kick the default-visible sub-page's stream(s) now; others start on switch.
  document.querySelectorAll('#subpanel-line .fn-img').forEach(applyFnStream);

  // ---------- Lidar status (double_lidar_fusion) ----------
  const LIDAR_SRC = [
    { key: 'fused', input: 'lidar-fused', def: '/scan' },
    { key: 's1', input: 'lidar-scan1', def: '/scan1' },
    { key: 's2', input: 'lidar-scan2', def: '/scan2' },
  ];

  function subscribeLidar() {
    lidarSubs.forEach((t) => { try { t.unsubscribe(); } catch (_) { /* ignore */ } });
    lidarSubs = [];
    if (!ros) return;
    LIDAR_SRC.forEach((src) => {
      const el = $(src.input);
      const topic = ((el && el.value) || src.def).trim();
      if (!topic) return;
      const t = new ROSLIB.Topic({
        ros, name: topic, messageType: 'sensor_msgs/msg/LaserScan',
        throttle_rate: 200, queue_length: 1,
      });
      t.subscribe((msg) => {
        const ranges = msg.ranges || [];
        let n = 0, min = Infinity;
        for (let i = 0; i < ranges.length; i++) {
          const r = ranges[i];
          if (isFinite(r) && r >= msg.range_min && r <= msg.range_max) {
            n++;
            if (r < min) min = r;
          }
        }
        lidarLast[src.key] = { time: Date.now(), points: n, min: isFinite(min) ? min : null };
      });
      lidarSubs.push(t);
    });
  }

  function updateLidarUI() {
    LIDAR_SRC.forEach((src) => {
      const el = $('lidar-' + (src.key === 'fused' ? 'fused-val' : (src.key === 's1' ? 's1-val' : 's2-val')));
      if (!el) return;
      const last = lidarLast[src.key];
      const online = last && (Date.now() - last.time < 1500);
      el.classList.remove('ok', 'err');
      if (online) { el.textContent = `在线 · ${last.points} 点`; el.classList.add('ok'); }
      else { el.textContent = '离线'; el.classList.add('err'); }
    });
    const minEl = $('lidar-fused-min');
    if (minEl) {
      const last = lidarLast.fused;
      const online = last && (Date.now() - last.time < 1500);
      minEl.classList.remove('warn', 'err');
      if (online && last.min != null) {
        minEl.textContent = last.min.toFixed(2) + ' m';
        if (last.min < 0.3) minEl.classList.add('err');
        else if (last.min < 0.6) minEl.classList.add('warn');
      } else {
        minEl.textContent = '— m';
      }
    }
  }
  setInterval(updateLidarUI, 500);
  const lidarApplyBtn = $('lidar-apply');
  if (lidarApplyBtn) lidarApplyBtn.addEventListener('click', subscribeLidar);

  // ---------- YOLO detections (optional external node) ----------
  function renderYolo(msg) {
    const view = $('yolo-detections');
    if (!view) return;
    const dets = (msg && msg.detections) || [];
    const countEl = $('yolo-det-count');
    if (countEl) countEl.textContent = dets.length + ' 个目标';
    view.innerHTML = dets.map((d) => {
      let label = '?', score = 0, track = '';
      if (d && (d.class_name != null || d.class_id != null)) {
        // yolo_ros (yolo_msgs/Detection): class_name/score, optional tracking id.
        label = (d.class_name != null && d.class_name !== '') ? d.class_name
              : (d.class_id != null ? d.class_id : '?');
        score = (d.score != null) ? d.score : 0;
        track = (d.id != null && d.id !== '') ? d.id : '';
      } else {
        // vision_msgs/Detection2DArray fallback: newer nests a `hypothesis`
        // (class_id/score), older is flat (id/score).
        const res = (d && d.results && d.results[0]) || null;
        if (res) {
          const h = res.hypothesis || res;
          label = (h.class_id != null) ? h.class_id : (h.id != null ? h.id : '?');
          score = (h.score != null) ? h.score : 0;
        }
      }
      const trackHtml = track ? ` <span class="yd-track">#${escapeHTML(String(track))}</span>` : '';
      return `<div class="yolo-det-row"><span class="yd-label">${escapeHTML(String(label))}${trackHtml}</span>` +
        `<span class="yd-score">${(Number(score) * 100).toFixed(0)}%</span></div>`;
    }).join('');
  }
  function subscribeYolo() {
    if (yoloDetSub) { try { yoloDetSub.unsubscribe(); } catch (_) { /* ignore */ } yoloDetSub = null; }
    if (!ros) return;
    const el = $('yolo-det-topic');
    const topic = ((el && el.value) || '').trim();
    if (!topic) return;
    yoloDetSub = new ROSLIB.Topic({
      ros, name: topic, messageType: 'yolo_msgs/msg/DetectionArray', throttle_rate: 200,
    });
    yoloDetSub.subscribe(renderYolo);
  }
  const yoloApplyBtn = $('yolo-det-apply');
  if (yoloApplyBtn) yoloApplyBtn.addEventListener('click', subscribeYolo);

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

  // Voice-component TTS box (各组件状态) shares the same /tts_text publisher.
  const voiceTtsInput = $('voice-tts-input');
  const voiceTtsSend = $('voice-tts-send');
  if (voiceTtsSend) voiceTtsSend.addEventListener('click', () => sendTts(voiceTtsInput.value));
  if (voiceTtsInput) voiceTtsInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') { sendTts(voiceTtsInput.value); voiceTtsInput.value = ''; }
  });

  const vlaClearBtn = $('vla-clear');
  if (vlaClearBtn) vlaClearBtn.addEventListener('click', () => {
    const v = $('vla-timeline');
    if (v) v.innerHTML = '';
  });

  // ---------- 机械臂 (Lebai LM3: lebai_driver + grab_demo) ----------
  // 状态显示走 /robot_status、/gripper_status、/joint_states 等话题；控制走
  // /system_service、/io_service、/motion_service 的服务以及 grab_demo 的
  // /obj_grab_service、/arm_arbiter/*。全部经 rosbridge，无需额外后端。
  let armStatusTime = 0;   // 最近一次 /robot_status 到达时间（离线检测）
  let armDistTime = 0;     // 最近一次 /grab_target/distance 到达时间
  const armJointMap = new Map(); // 关节名 -> { pos, vel }
  let armJointsDirty = false;

  function setArmMetric(id, text, cls) {
    const el = $(id);
    if (!el) return;
    el.textContent = text;
    el.classList.remove('ok', 'warn', 'err');
    if (cls) el.classList.add(cls);
  }

  // 目标距离在"抓取与仲裁"卡片和每个抓取方案子页各有一份显示。
  function setArmDist(text) {
    document.querySelectorAll('.arm-dist').forEach((el) => { el.textContent = text; });
  }

  // lebai_interfaces/TriState: val 1/0/-1(未知)。labels = { on: [文本, 类名], off: [...] }
  function renderArmTri(id, tri, labels) {
    const v = (tri && typeof tri.val === 'number') ? tri.val : -1;
    if (v === 1) setArmMetric(id, labels.on[0], labels.on[1]);
    else if (v === 0) setArmMetric(id, labels.off[0], labels.off[1]);
    else setArmMetric(id, '未知', 'warn');
  }

  function renderArmStatus(msg) {
    armStatusTime = Date.now();
    renderArmTri('arm-estop', msg.e_stopped, { on: ['触发', 'err'], off: ['正常', 'ok'] });
    renderArmTri('arm-powered', msg.drives_powered, { on: ['已上电', 'ok'], off: ['未上电', 'warn'] });
    renderArmTri('arm-motion-possible', msg.motion_possible, { on: ['可运动', 'ok'], off: ['不可运动', 'warn'] });
    renderArmTri('arm-in-motion', msg.in_motion, { on: ['运动中', 'warn'], off: ['静止', 'ok'] });
    renderArmTri('arm-in-error', msg.in_error, { on: ['有错误', 'err'], off: ['正常', 'ok'] });
    const code = msg.error_code || 0;
    setArmMetric('arm-error-code', String(code), code ? 'err' : 'ok');
    const m = (msg.mode && typeof msg.mode.val === 'number') ? msg.mode.val : -1;
    if (m === 2) setArmMetric('arm-mode', '自动', 'ok');
    else if (m === 1) setArmMetric('arm-mode', '手动/示教', 'warn');
    else setArmMetric('arm-mode', '未知', 'warn');
  }

  // 驱动离线时把状态清回 "—"，避免一直显示陈旧值误导操作。
  setInterval(() => {
    if (armStatusTime && Date.now() - armStatusTime > 3000) {
      armStatusTime = 0;
      ['arm-estop', 'arm-powered', 'arm-motion-possible', 'arm-in-motion',
        'arm-in-error', 'arm-error-code', 'arm-mode'].forEach((id) => setArmMetric(id, '—'));
    }
    if (armDistTime && Date.now() - armDistTime > 2500) {
      armDistTime = 0;
      setArmDist('— m');
    }
  }, 1000);

  function ingestArmJointState(msg) {
    const names = msg.name || [];
    for (let i = 0; i < names.length; i++) {
      armJointMap.set(names[i], {
        pos: (msg.position && typeof msg.position[i] === 'number') ? msg.position[i] : 0,
        vel: (msg.velocity && typeof msg.velocity[i] === 'number') ? msg.velocity[i] : 0,
      });
    }
    armJointsDirty = true;
  }

  // 关节表最高 5Hz 重绘（消息可能到得更快，合并渲染）。
  setInterval(() => {
    if (!armJointsDirty) return;
    armJointsDirty = false;
    const view = $('arm-joints');
    if (!view) return;
    const rows = [];
    armJointMap.forEach((v, name) => {
      rows.push(
        `<div class="arm-joint-row"><span class="aj-name" title="${escapeHTML(name)}">${escapeHTML(name)}</span>` +
        `<span>${(v.pos * 180 / Math.PI).toFixed(1)}</span>` +
        `<span>${v.pos.toFixed(3)}</span>` +
        `<span>${v.vel.toFixed(3)}</span></div>`
      );
    });
    view.innerHTML = rows.join('');
  }, 200);

  const ARM_LOG_MAX_LINES = 200;
  function addArmLog(text) {
    const view = $('arm-log');
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
    while (view.childElementCount > ARM_LOG_MAX_LINES) view.removeChild(view.firstChild);
    view.scrollTop = view.scrollHeight;
  }

  function callArmService(name, type, req, onRes) {
    if (!ros) { addArmLog('未连接 rosbridge，无法调用 ' + name); return; }
    const srv = new ROSLIB.Service({ ros, name, serviceType: type });
    srv.callService(new ROSLIB.ServiceRequest(req || {}), (res) => {
      if (onRes) onRes(res);
    }, (err) => addArmLog('✗ ' + name + ' 调用失败：' + err + '（对应驱动节点是否已启动？）'));
  }

  // 系统控制按钮：/system_service/<name>（std_srvs/Empty），危险操作二次确认。
  document.querySelectorAll('#panel-arm .sys-btns button').forEach((btn) => {
    btn.addEventListener('click', () => {
      const name = btn.dataset.sys;
      if (!name) return;
      if (btn.dataset.confirm && !window.confirm(btn.dataset.confirm)) return;
      addArmLog('> 系统命令：' + btn.textContent.trim() + ' (' + name + ')');
      callArmService('/system_service/' + name, 'std_srvs/srv/Empty', {},
        () => addArmLog('✓ ' + name + ' 已执行'));
    });
  });

  // 急停：立即下发，不做确认。
  const armEstopBtn = $('arm-estop-btn');
  if (armEstopBtn) {
    armEstopBtn.addEventListener('click', () => {
      addArmLog('> 机械臂急停 (emergency_stop)');
      callArmService('/system_service/emergency_stop', 'std_srvs/srv/Empty', {},
        () => addArmLog('✓ 急停已下发'));
    });
  }

  // 夹爪：滑块 + 应用按钮 / 张开闭合快捷键 -> SetGripper 服务。
  function initGripSlider(rangeId, valId) {
    const range = $(rangeId);
    const val = $(valId);
    if (range && val) {
      range.addEventListener('input', () => { val.textContent = range.value; });
    }
    return range;
  }
  const gripPosRange = initGripSlider('arm-grip-pos-range', 'arm-grip-pos-val');
  const gripForceRange = initGripSlider('arm-grip-force-range', 'arm-grip-force-val');

  function setGripper(kind, value) {
    const v = Math.max(0, Math.min(100, +value || 0));
    const srvName = kind === 'force'
      ? '/io_service/set_gripper_force' : '/io_service/set_gripper_position';
    addArmLog('> 夹爪' + (kind === 'force' ? '力度' : '位置') + ' → ' + v);
    callArmService(srvName, 'lebai_interfaces/srv/SetGripper', { val: v }, (res) => {
      addArmLog(res && res.ret ? '✓ 夹爪命令已执行' : '✗ 夹爪命令被拒绝');
    });
  }
  const gripPosApply = $('arm-grip-pos-apply');
  if (gripPosApply) gripPosApply.addEventListener('click', () => setGripper('position', gripPosRange.value));
  const gripForceApply = $('arm-grip-force-apply');
  if (gripForceApply) gripForceApply.addEventListener('click', () => setGripper('force', gripForceRange.value));
  const gripOpen = $('arm-grip-open');
  if (gripOpen) gripOpen.addEventListener('click', () => {
    if (gripPosRange) { gripPosRange.value = 100; $('arm-grip-pos-val').textContent = '100'; }
    setGripper('position', 100);
  });
  const gripClose = $('arm-grip-close');
  if (gripClose) gripClose.addEventListener('click', () => {
    if (gripPosRange) { gripPosRange.value = 0; $('arm-grip-pos-val').textContent = '0'; }
    setGripper('position', 0);
  });

  // 关节运动：6 个角度(rad) + acc/vel -> /motion_service/move_joint。
  const armMjInputs = Array.from(document.querySelectorAll('#panel-arm .mj-j'));

  const armMjFill = $('arm-mj-fill');
  if (armMjFill) {
    armMjFill.addEventListener('click', () => {
      // 优先取乐白本体关节（lebai_joint_1..6，按名排序即按编号排序），
      // 取不到（如自定义关节名）时退回 /joint_states 里的前 6 个。
      let names = Array.from(armJointMap.keys()).filter((n) => n.startsWith('lebai_joint')).sort();
      if (names.length < armMjInputs.length) names = Array.from(armJointMap.keys());
      if (names.length === 0) { addArmLog('未收到 /joint_states，无法填入（机械臂驱动是否已启动？）'); return; }
      armMjInputs.forEach((inp, i) => {
        const j = armJointMap.get(names[i]);
        if (j) inp.value = j.pos.toFixed(4);
      });
      addArmLog('已填入当前关节角：' + names.slice(0, armMjInputs.length).join(', '));
    });
  }

  const armMjRun = $('arm-mj-run');
  if (armMjRun) {
    armMjRun.addEventListener('click', () => {
      const pose = armMjInputs.map((inp) => parseFloat(inp.value));
      if (pose.some((v) => Number.isNaN(v))) { alert('关节角必须是数字（弧度）'); return; }
      const acc = parseFloat($('arm-mj-acc').value);
      const vel = parseFloat($('arm-mj-vel').value);
      if (!(acc > 0) || !(vel > 0)) { alert('acc / vel 必须为正数'); return; }
      const txt = pose.map((v) => v.toFixed(3)).join(', ');
      if (!window.confirm('将真实移动机械臂到关节角 [' + txt + '] (rad)，确认执行？')) return;
      addArmLog('> move_joint [' + txt + '] acc=' + acc + ' vel=' + vel);
      // 完整填充请求，rosbridge 才能稳定接受（同 buildParameterValue 的做法）。
      callArmService('/motion_service/move_joint', 'lebai_interfaces/srv/MoveJoint', {
        is_joint_pose: true,
        joint_pose: pose,
        cartesian_pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
        common: { acc, vel, time: 0, radius: 0 },
      }, (res) => {
        addArmLog(res && res.ret ? '✓ move_joint 已执行' : '✗ move_joint 失败（看驱动日志）');
      });
    });
  }

  // 视觉抓取：VLM 指令 / 确认 / 取消 + 手动 TF 抓取 + 抓取仲裁接管。
  const armVlmInstrInput = $('arm-vlm-instr');
  function sendArmVlmInstruction() {
    const t = (armVlmInstrInput && armVlmInstrInput.value || '').trim();
    if (!t) return;
    if (!armVlmInstrPub) { addArmLog('未连接 rosbridge，无法发送指令'); return; }
    armVlmInstrPub.publish(new ROSLIB.Message({ data: t }));
    addArmLog('> VLM 指令：' + t);
    armVlmInstrInput.value = '';
  }
  const armVlmSend = $('arm-vlm-send');
  if (armVlmSend) armVlmSend.addEventListener('click', sendArmVlmInstruction);
  if (armVlmInstrInput) armVlmInstrInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') sendArmVlmInstruction();
  });

  function sendArmVlmConfirm(yes) {
    if (!armVlmConfirmPub) { addArmLog('未连接 rosbridge，无法发送确认'); return; }
    armVlmConfirmPub.publish(new ROSLIB.Message({ data: !!yes }));
    addArmLog(yes ? '> 确认抓取 (/vlm/confirm true)' : '> 取消抓取 (/vlm/confirm false)');
  }
  const armVlmConfirm = $('arm-vlm-confirm');
  if (armVlmConfirm) armVlmConfirm.addEventListener('click', () => sendArmVlmConfirm(true));
  const armVlmCancel = $('arm-vlm-cancel');
  if (armVlmCancel) armVlmCancel.addEventListener('click', () => sendArmVlmConfirm(false));

  function callArmArbiter(action) {
    addArmLog('> 抓取仲裁：' + (action === 'manual_takeover' ? '手动接管' : '释放控制权'));
    callArmService('/arm_arbiter/' + action, 'std_srvs/srv/Trigger', {}, (res) => {
      const msg = (res && res.message) || '';
      addArmLog((res && res.success ? '✓ ' : '✗ ') + (msg || action));
    });
  }
  const armArbTakeover = $('arm-arb-takeover');
  if (armArbTakeover) armArbTakeover.addEventListener('click', () => callArmArbiter('manual_takeover'));
  const armArbRelease = $('arm-arb-release');
  if (armArbRelease) armArbRelease.addEventListener('click', () => callArmArbiter('manual_release'));

  function requestArmGrab(frame) {
    const f = (frame || '').trim();
    if (!f) { alert('请填写目标 TF 名（如 target_frame）'); return; }
    if (!window.confirm('将驱动机械臂抓取 TF 目标 "' + f + '"，确认执行？')) return;
    addArmLog('> 抓取请求 /obj_grab_service (obj_link=' + f + ')，规划可能需要数十秒…');
    callArmService('/obj_grab_service', 'grab_demo/srv/GrabObject', { obj_link: f }, (res) => {
      const msg = (res && res.message) || '';
      addArmLog((res && res.success ? '✓ 抓取成功' : '✗ 抓取失败') + (msg ? '：' + msg : ''));
    });
  }
  const armGrabCall = $('arm-grab-call');
  if (armGrabCall) {
    armGrabCall.addEventListener('click', () => requestArmGrab($('arm-grab-frame').value));
  }
  // 各抓取方案子页的快捷按钮：直接抓 "抓取与仲裁" 卡片里填的 TF（默认 target_frame）。
  document.querySelectorAll('#panel-arm .arm-grab-quick').forEach((btn) => {
    btn.addEventListener('click', () => {
      const inp = $('arm-grab-frame');
      requestArmGrab((inp && inp.value) || 'target_frame');
    });
  });

  const armLogClear = $('arm-log-clear');
  if (armLogClear) armLogClear.addEventListener('click', () => {
    const v = $('arm-log');
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

  // ---------- Navigation (top tabs + function sub-tabs) ----------
  // Switching just toggles a CSS class; every card stays in the DOM so all
  // ROS wiring (which looks elements up by id) keeps working whether or not
  // its tab is visible.
  (function initNav() {
    function initTabGroup(opts) {
      const btns = Array.from(document.querySelectorAll(opts.btnSel));
      function activate(val) {
        if (!btns.some((b) => b.dataset[opts.key] === val)) return;
        btns.forEach((b) => b.classList.toggle('active', b.dataset[opts.key] === val));
        Array.from(document.querySelectorAll(opts.panelSel))
          .forEach((p) => p.classList.toggle('active', p.id === opts.prefix + val));
        // Chart.js canvases and the three.js viewer size to their container,
        // which reads as 0×0 while the panel is display:none. Nudging a resize
        // once the panel is visible makes them re-measure and fill the space.
        window.dispatchEvent(new Event('resize'));
        if (opts.onActivate) opts.onActivate(val);
      }
      btns.forEach((b) => b.addEventListener('click', () => {
        activate(b.dataset[opts.key]);
        if (opts.hash) {
          try { history.replaceState(null, '', '#' + b.dataset[opts.key]); } catch (_) { /* ignore */ }
        }
      }));
      return activate;
    }

    // Function sub-tabs: re-apply the sub-page's MJPEG stream when it becomes
    // visible (an <img> in a display:none parent may never have fetched).
    initTabGroup({
      btnSel: '.subtab-btn', key: 'subtab', panelSel: '.subtab-panel', prefix: 'subpanel-',
      onActivate: (val) => {
        document.querySelectorAll('#subpanel-' + val + ' .fn-img').forEach(applyFnStream);
      },
    });

    // 机械臂子页（监控与控制 / HSV / YOLO / KCF / ArUco / VLM）。独立于
    // 功能模块的 .subtab-* 组，互不干扰。
    initTabGroup({
      btnSel: '.arm-subtab-btn', key: 'armsubtab', panelSel: '.arm-subtab-panel', prefix: 'arm-subpanel-',
      onActivate: (val) => {
        document.querySelectorAll('#arm-subpanel-' + val + ' .fn-img').forEach(applyFnStream);
      },
    });

    const activateTop = initTabGroup({
      btnSel: '.tab-btn', key: 'tab', panelSel: '.tab-panel', prefix: 'panel-', hash: true,
      onActivate: (val) => {
        // 机械臂页的 MJPEG 流首次显示时才开始拉取（同子页面的处理方式），
        // 只拉当前激活子页的流，避免一次拉起所有方案的调试画面。
        if (val === 'arm') {
          document.querySelectorAll('#panel-arm .arm-subtab-panel.active .fn-img').forEach(applyFnStream);
        }
      },
    });

    // Allow deep-linking to a top tab via #hash (e.g. .../#control).
    const initial = (location.hash || '').replace(/^#/, '');
    if (initial) activateTop(initial);
  })();

  // Auto-connect on load.
  connect();
})();
