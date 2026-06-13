/* Wheeltec dashboard — roslibjs front-end */
(function () {
  'use strict';

  const MAX_POINTS = 120; // ~2 min @ 1 Hz, less at higher rates
  const $ = (id) => document.getElementById(id);

  // ---------- Toast 通知 ----------
  // 关键事件（连接断开/急停/手柄/导航目标）的全局即时反馈：用户可能正
  // 盯着别的卡片或标签页，卡内的 setCardMsg 看不到。
  const toastStack = $('toast-stack');
  function toast(text, kind, ms) {
    if (!toastStack) return;
    const el = document.createElement('div');
    el.className = 'toast' + (kind ? ' ' + kind : '');
    el.textContent = text;
    toastStack.appendChild(el);
    while (toastStack.children.length > 5) toastStack.removeChild(toastStack.firstChild);
    setTimeout(() => {
      el.classList.add('leaving');
      setTimeout(() => { if (el.parentNode) el.parentNode.removeChild(el); }, 300);
    }, ms || 3500);
  }

  // ---------- Connection ----------
  const defaultUrl = `ws://${location.hostname || 'localhost'}:9090`;
  const urlInput = $('ws-url');
  const statusEl = $('ws-status');
  const connectBtn = $('ws-connect');
  urlInput.value = defaultUrl;
  // 记住用户改过的 rosbridge 地址（端口转发/异机调试不用每次重填）；
  // 没改过就一直跟随默认值。回车 = 立即连接。
  try {
    const savedUrl = localStorage.getItem('ws_url');
    if (savedUrl) urlInput.value = savedUrl;
  } catch (_) { /* localStorage 不可用就算了 */ }
  urlInput.addEventListener('change', () => {
    try { localStorage.setItem('ws_url', urlInput.value.trim()); } catch (_) { /* ignore */ }
  });
  urlInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') { e.preventDefault(); connect(); }
  });

  let ros = null;
  const subs = [];   // active subscriptions
  let cmdVelPub = null;
  let cmdVelManualPub = null;  // /cmd_vel_manual: 同步发给 nav_arbiter, 手动打断 Nav2/VLA
  let vlaInstrPub = null;  // /vla/instruction publisher
  let ttsPub = null;       // /tts_text publisher
  let armVlmInstrPub = null;   // /vlm/instruction publisher (机械臂 VLM 抓取)
  let armVlmConfirmPub = null; // /vlm/confirm publisher (确认/取消抓取)
  let kcfBboxPub = null;       // /kcf_node/select_bbox publisher (KCF 手动框选)
  let simLaunchPub = null;     // /sim_launch/cmd publisher (网页一键启停 Gazebo 仿真)
  let armSimTrajPub = null;    // /lebai_trajectory_controller/joint_trajectory (仿真臂关节控制)
  let initialPosePub = null;   // /initialpose publisher (Nav2 2D Pose Estimate · AMCL)
  let rechargeFlagPub = null;  // /robot_recharge_flag publisher (自动回充 1开/0关)
  let securityPub = null;      // /chassis_security publisher (固件安全等级 0/1)
  let bodyModePub = null;      // /mode publisher (骨架识别 1=姿态交互 2=跟随)
  let bodyRecoveryPub = null;  // /recoveryid publisher (骨架识别找回锁定目标)
  let wpTf = null;             // map→base_footprint TF (VLA 地图航点管理)
  let wpCmdPub = null;         // /vla/waypoint_cmd publisher (航点增删, 后端持久化)
  let goalPosePub = null;      // /goal_pose publisher (航点列表"导航"按钮直达)
  let chatMsgPub = null;       // /chat_message publisher (AI 对话·话题流式模式)
  let rrtClickPub = null;      // /clicked_point publisher (RRT 探索边界选点, 等价 RViz Publish Point)
  let usTf = null;             // base_footprint 系 TF (超声波 ultrasonic_A..F 安装位姿)
  let lidarSubs = [];      // per-source LaserScan subs (managed separately so
                           // the lidar card can re-subscribe on topic change)
  const lidarLast = {};    // key -> { time, points, min }
  let yoloDetSub = null;   // optional yolo_msgs/DetectionArray sub (yolo_ros)
  let chassisTime = 0;     // 最近一次底盘遥测(/PowerVoltage)到达时间（架构卡在线点）

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

  // ---- 断线自动重连 ----
  // 只对"建立过的连接意外掉线"和"自动尝试失败"重试（指数退避 2s→30s
  // 封顶，状态栏显示倒计时）；用户手动点"连接"失败（多半是地址填错）
  // 不重试，手动"断开"取消一切重试。页面加载时的首次连接也按自动流程
  // 算——rosbridge 常比面板后起，开着页面等它上线即可。
  let reconnectTimer = null;
  let reconnectDelay = 0;   // 0 = 当前不在重连流程
  const RECONNECT_MAX_MS = 30000;

  function cancelReconnect() {
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    reconnectDelay = 0;
  }
  function scheduleReconnect() {
    if (reconnectTimer) return;
    reconnectDelay = reconnectDelay ? Math.min(reconnectDelay * 2, RECONNECT_MAX_MS) : 2000;
    setStatus('conn', `已断开，${Math.round(reconnectDelay / 1000)}s 后自动重连…`);
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null;
      connect(true);
    }, reconnectDelay);
  }

  function connect(isAuto) {
    if (!isAuto) cancelReconnect();
    else if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    if (ros) {
      try { ros.close(); } catch (_) { /* ignore */ }
    }
    userWantsConnected = true;
    setButtonForState(true);
    setStatus('conn', '连接中…');
    const r = new ROSLIB.Ros({ url: urlInput.value });
    ros = r;
    let established = false;  // 本次尝试握手成功过——决定掉线后是否自动重连

    r.on('connection', () => {
      if (ros !== r) return;
      established = true;
      reconnectDelay = 0;     // 连上了，退避归零
      setStatus('on', '已连接');
      toast('rosbridge 已连接', 'ok');
      setupTopics();
    });
    r.on('close', () => {
      // Stale close from a previously discarded ros instance — ignore.
      if (ros !== r) return;
      setStatus('off', '已断开');
      // 主动断开走 disconnect()（那时 ros 已置 null，进不到这里），
      // 能走到这的都是意外掉线，必须显眼提示。
      if (established) toast('rosbridge 连接断开，自动重连中…', 'err');
      teardownTopics();
      setButtonForState(false);
      // Keep intent in sync with the visible label: after an unexpected
      // close, the button now says 连接, so the next click must take the
      // connect path — otherwise users have to click twice to reconnect.
      userWantsConnected = false;
      ros = null;
      if (established || isAuto) scheduleReconnect();
    });
    r.on('error', (err) => {
      if (ros !== r) return;
      console.error('rosbridge error', err);
      setStatus('off', '连接错误');
      // 自动重连的失败尝试不刷 toast——退避倒计时在状态栏可见
      if (established) toast('rosbridge 连接断开，自动重连中…', 'err');
      else if (!isAuto) toast('rosbridge 连接错误，请检查 ws 地址与 rosbridge 是否在跑', 'err');
      // WebSocket usually fires 'close' right after 'error', but not
      // always (e.g. immediate handshake failure on some browsers).
      // Reset the same state here so the button label and intent stay
      // consistent regardless. The 'ros !== r' guard above plus setting
      // ros = null below makes a follow-up 'close' a no-op.
      teardownTopics();
      setButtonForState(false);
      userWantsConnected = false;
      ros = null;
      if (established || isAuto) scheduleReconnect();
    });
  }

  function disconnect() {
    cancelReconnect();
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
    if (pfSub) {
      try { pfSub.unsubscribe(); } catch (_) { /* ignore */ }
      pfSub = null;
    }
    if (cmdVelPub) {
      try { cmdVelPub.unadvertise(); } catch (_) { /* ignore */ }
      cmdVelPub = null;
    }
    if (cmdVelManualPub) {
      try { cmdVelManualPub.unadvertise(); } catch (_) { /* ignore */ }
      cmdVelManualPub = null;
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
    if (kcfBboxPub) {
      try { kcfBboxPub.unadvertise(); } catch (_) { /* ignore */ }
      kcfBboxPub = null;
    }
    if (simLaunchPub) {
      try { simLaunchPub.unadvertise(); } catch (_) { /* ignore */ }
      simLaunchPub = null;
    }
    if (armSimTrajPub) {
      try { armSimTrajPub.unadvertise(); } catch (_) { /* ignore */ }
      armSimTrajPub = null;
    }
    if (initialPosePub) {
      try { initialPosePub.unadvertise(); } catch (_) { /* ignore */ }
      initialPosePub = null;
    }
    if (rechargeFlagPub) {
      try { rechargeFlagPub.unadvertise(); } catch (_) { /* ignore */ }
      rechargeFlagPub = null;
    }
    if (securityPub) {
      try { securityPub.unadvertise(); } catch (_) { /* ignore */ }
      securityPub = null;
    }
    if (bodyModePub) {
      try { bodyModePub.unadvertise(); } catch (_) { /* ignore */ }
      bodyModePub = null;
    }
    if (bodyRecoveryPub) {
      try { bodyRecoveryPub.unadvertise(); } catch (_) { /* ignore */ }
      bodyRecoveryPub = null;
    }
    if (wpTf) {
      try { wpTf.dispose(); } catch (_) { /* ignore */ }
      wpTf = null;
    }
    if (wpCmdPub) {
      try { wpCmdPub.unadvertise(); } catch (_) { /* ignore */ }
      wpCmdPub = null;
    }
    if (goalPosePub) {
      try { goalPosePub.unadvertise(); } catch (_) { /* ignore */ }
      goalPosePub = null;
    }
    if (chatMsgPub) {
      try { chatMsgPub.unadvertise(); } catch (_) { /* ignore */ }
      chatMsgPub = null;
    }
    if (rrtClickPub) {
      try { rrtClickPub.unadvertise(); } catch (_) { /* ignore */ }
      rrtClickPub = null;
    }
    if (usTf) {
      try { usTf.dispose(); } catch (_) { /* ignore */ }
      usTf = null;
    }
    // 航点列表属于上一个连接的后端, 清掉避免误导; 地图位图保留(重连通常同一张图)
    vlaWaypoints = [];
    renderWpList();
    // 机械臂侧状态属于上一个连接：清掉，避免换机器人重连后残留
    // 旧关节行 / 旧的 YOLO 目标选中态（armJointMap 也因此不会无限增长）。
    armJointMap.clear();
    armJointsDirty = true;
    armYoloSelected = '';
    armYoloLastHtml = '';
    const yoloBtnsView = $('arm-yolo-buttons');
    if (yoloBtnsView) yoloBtnsView.innerHTML = '';
  }

  function setupTopics() {
    teardownTopics();

    sub('/PowerVoltage', 'std_msgs/msg/Float32', (msg) => {
      chassisTime = Date.now();
      stm32Time = Date.now();
      const v = msg.data;
      const el = $('m-voltage');
      el.textContent = v.toFixed(2) + ' V';
      el.classList.remove('ok', 'warn', 'err');
      // 24V 电池组：<20V 固件禁止底盘移动（err），<22V 提前预警（warn）。
      if (v < 20) el.classList.add('err');
      else if (v < 22) el.classList.add('warn');
      else el.classList.add('ok');
      updateStm32Voltage(v);
      pushChart(chartVoltage, v);
    });

    sub('/robot_charging_flag', 'std_msgs/msg/Bool', (msg) => {
      ['m-charging', 'rc-charging'].forEach((id) => {
        const el = $(id);
        if (!el) return;
        el.textContent = msg.data ? '是' : '否';
        el.classList.toggle('ok', !!msg.data);
      });
    });
    sub('/robot_charging_current', 'std_msgs/msg/Float32', (msg) => {
      ['m-charge-current', 'rc-current'].forEach((id) => {
        const el = $(id);
        if (el) el.textContent = msg.data.toFixed(2) + ' A';
      });
    });
    // red flag = 回充红外：固件回充帧 rx[3] 是收到充电桩红外的对管个数
    // (ChargeDev.RedNum 0-4)，驱动转成 Bool 发布 —— 不是急停信号。
    sub('/robot_red_flag', 'std_msgs/msg/Bool', (msg) => {
      ['m-red', 'stm32-red', 'rc-red'].forEach((id) => {
        const el = $(id);
        if (!el) return;
        el.textContent = msg.data ? '检测到充电桩' : '未检测';
        el.classList.remove('ok', 'warn', 'err');
        if (msg.data) el.classList.add('ok');   // 未检测是常态，不着色
      });
    });
    sub('/self_check_data', 'std_msgs/msg/UInt32', (msg) => {
      $('m-selfcheck').textContent = '0x' + msg.data.toString(16).toUpperCase();
    });
    // 传感器在线监控（sensor_watchdog）：各传感器绿/红灯 + 串口设备表。
    sub('/sensor_watchdog/status', 'std_msgs/msg/String', (msg) => {
      let obj = null;
      try { obj = JSON.parse(msg.data || '{}'); } catch (_) { return; }
      swTime = Date.now();
      renderSensorWatchdog(obj);
    }, { throttle_rate: 500 });

    // 下位机使能位（24字节帧 rx[1]）：固件真实的"允许移动"信号。
    // 需重编译 turn_on_wheeltec_robot 才有此话题，没有时指标保持 "—"。
    sub('/robot_enable_flag', 'std_msgs/msg/Bool', (msg) => {
      stm32Time = Date.now();
      updateStm32Enable(!!msg.data);
    }, { throttle_rate: 200 });
    // 下位机回充模式回读（回充帧 rx[5]）：固件确认已进入/退出 ChargeMode。
    sub('/robot_recharge_mode', 'std_msgs/msg/Bool', (msg) => {
      updateRechargeMode(!!msg.data);
    }, { throttle_rate: 200 });

    sub('/odom', 'nav_msgs/msg/Odometry', (msg) => {
      stm32Time = Date.now();   // /odom 只在串口帧校验通过时发布 → 下位机在线信号
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
    // 同一份手动速度再发一路给 nav_arbiter（没跑仲裁节点时此话题无人订阅，
    // 无副作用）：仲裁器收到即取消 Nav2/VLA 的导航目标，手动随时打断自主。
    cmdVelManualPub = new ROSLIB.Topic({
      ros, name: '/cmd_vel_manual', messageType: 'geometry_msgs/msg/Twist',
    });
    cmdVelManualPub.advertise();
    // 导航仲裁状态（nav_arbiter）：VLA 卡显示 + 接管/释放边沿进时间线。
    sub('/nav_arbiter/status', 'std_msgs/msg/String', renderNavArbiter);

    // 自动回充开关与固件安全等级：都是驱动里的标志位，随下一帧 cmd_vel
    // 序列化进串口帧（frame[1]/frame[2]）下发，因此发布后要补发一帧 cmd_vel。
    rechargeFlagPub = new ROSLIB.Topic({
      ros, name: '/robot_recharge_flag', messageType: 'std_msgs/msg/Int8',
    });
    rechargeFlagPub.advertise();
    securityPub = new ROSLIB.Topic({
      ros, name: '/chassis_security', messageType: 'std_msgs/msg/Int8',
    });
    securityPub.advertise();

    // 驱动发给下位机的 11 字节控制帧原样回发（/robot_serial_tx，需重编译驱动），
    // 按通信协议表解析后进 STM32 卡的"最近下发指令"与事件栏。
    sub('/robot_serial_tx', 'std_msgs/msg/UInt8MultiArray', (msg) => {
      ingestStm32Tx(msg.data || []);
    }, { throttle_rate: 100 });

    // 骨架识别 (wheeltec_bodyreader)：姿态/人数/模式订阅 + 模式切换/找回目标发布。
    bodyModePub = new ROSLIB.Topic({
      ros, name: '/mode', messageType: 'std_msgs/msg/Int8',
    });
    bodyModePub.advertise();
    bodyRecoveryPub = new ROSLIB.Topic({
      ros, name: '/recoveryid', messageType: 'std_msgs/msg/Int16',
    });
    bodyRecoveryPub.advertise();
    sub('/body_posture', 'bodyreader_msg/msg/Bodyposture', renderBodyPosture, { throttle_rate: 200 });
    sub('/bodylist', 'bodyreader_msg/msg/Bodylist', (msg) => {
      const el = $('body-count');
      if (el) el.textContent = String(msg.count != null ? msg.count : '—');
    }, { throttle_rate: 500 });
    sub('/mode', 'std_msgs/msg/Int8', (msg) => {
      const el = $('body-mode');
      if (!el) return;
      const m = +msg.data;
      el.textContent = m === 2 ? '跟随' : (m === 1 ? '姿态交互' : String(m));
      el.classList.remove('ok', 'warn');
      el.classList.add(m === 2 ? 'warn' : 'ok');   // 跟随会动真车，标 warn 提醒
    });

    // VLA 地图航点管理：独立 TF 客户端固定以 map 为参考系（3D 视图那份的
    // fixed frame 跟随用户输入，默认 odom_combined，不能复用）。
    wpTf = makeTfClient(ros, 'map');
    wpCmdPub = new ROSLIB.Topic({
      ros, name: '/vla/waypoint_cmd', messageType: 'std_msgs/msg/String',
    });
    wpCmdPub.advertise();
    goalPosePub = new ROSLIB.Topic({
      ros, name: '/goal_pose', messageType: 'geometry_msgs/msg/PoseStamped',
    });
    goalPosePub.advertise();
    // Nav2 初始位姿 (AMCL 2D Pose Estimate) + 定位状态。
    initialPosePub = new ROSLIB.Topic({
      ros, name: '/initialpose', messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    });
    initialPosePub.advertise();
    sub('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', (msg) => {
      const p = (msg.pose && msg.pose.pose) || {};
      const pos = p.position || {}; const ori = p.orientation || { x: 0, y: 0, z: 0, w: 1 };
      const el = $('nav2-amcl');
      if (el) el.textContent = `已定位 · x=${(+pos.x).toFixed(2)} y=${(+pos.y).toFixed(2)} yaw=${(yawFromQuat(ori) * 180 / Math.PI).toFixed(1)}°`;
    }, { throttle_rate: 500 });
    // 后端 vla_navigator 广播的航点列表（变更即发 + 3s 周期，晚连也能拿到）
    sub('/vla/waypoints', 'std_msgs/msg/String', (msg) => {
      let obj = null;
      try { obj = JSON.parse(msg.data || '{}'); } catch (_) { return; }
      vlaWaypoints = (obj && obj.waypoints) || [];
      wpFile = (obj && obj.file) || '';
      renderWpList();
      wpRedraw();
    });
    // SLAM 建图中 /map 周期发布（volatile）可直接收到；map_server 的
    // transient_local 帧可能收不到 —— 由 GetMap 服务兜底（fetchWpMap）。
    sub('/map', 'nav_msgs/msg/OccupancyGrid', ingestWpMap,
      { queue_length: 1, throttle_rate: 2000 });
    wpMapTried = false;   // 每次连接自动尝试一次 GetMap

    // VLA: publish instructions / TTS text, watch recognition + status.
    vlaInstrPub = new ROSLIB.Topic({
      ros, name: '/vla/instruction', messageType: 'std_msgs/msg/String',
    });
    vlaInstrPub.advertise();
    ttsPub = new ROSLIB.Topic({
      ros, name: '/tts_text', messageType: 'std_msgs/msg/String',
    });
    ttsPub.advertise();

    // AI 对话 (ollama_ros_chat 话题流式模式): 发 /chat_message, 收 /chat_response
    chatMsgPub = new ROSLIB.Topic({
      ros, name: '/chat_message', messageType: 'std_msgs/msg/String',
    });
    chatMsgPub.advertise();
    sub('/chat_response', 'std_msgs/msg/String', onChatChunk);

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
    // 驱动改造后: 串口打开失败/拔线断开都会发 voice_flag=0(失败态每 10s 随
    // 重试重发), 重连成功发 1 —— 这里据此渲染并把边沿写进卡内提示。
    sub('/voice_flag', 'std_msgs/msg/Int8', (msg) => {
      const el = $('voice-mic'); if (!el) return;
      const ok = msg.data === 1 || msg.data === true;
      el.textContent = ok ? '已初始化' : '离线（串口未连接）';
      el.classList.remove('ok', 'err');
      el.classList.add(ok ? 'ok' : 'err');
      if (voiceMicState !== ok) {
        voiceMicState = ok;
        const tip = $('voice-mic-tip');
        if (tip) {
          tip.textContent = (ok ? '✓ 麦克风已连接' : '⚠ 麦克风串口连接失败，驱动每 10s 自动重试中——查看下方日志面板（按 mic 过滤）')
            + ' · ' + new Date().toLocaleTimeString();
          tip.classList.toggle('err-text', !ok);
        }
      }
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
      const c = $('pf-cmd'); if (c) c.textContent = txt;
      const d = $('mp-cmd'); if (d) d.textContent = txt;
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

    // 仿真启停 (sim_launcher): 发命令到 /sim_launch/cmd, 订阅 /sim_launch/status 反映状态。
    simLaunchPub = new ROSLIB.Topic({
      ros, name: '/sim_launch/cmd', messageType: 'std_msgs/msg/String',
    });
    simLaunchPub.advertise();
    // 仿真臂关节控制: 直接发 JointTrajectory 给 Gazebo 里的 ros2_control 控制器
    armSimTrajPub = new ROSLIB.Topic({
      ros, name: '/lebai_trajectory_controller/joint_trajectory',
      messageType: 'trajectory_msgs/msg/JointTrajectory',
    });
    armSimTrajPub.advertise();
    sub('/sim_launch/status', 'std_msgs/msg/String', (msg) => {
      let obj = null;
      try { obj = JSON.parse(msg.data || '{}'); } catch (_) { return; }
      renderSimLaunch(obj);
    });

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
      let text = s || '—';
      let cls = '';
      if (s === 'manual') { text = '手动接管'; cls = 'warn'; }
      else if (s === 'idle') { text = '自动/空闲'; cls = 'ok'; }
      // 状态卡片与"抓取与仲裁"卡片各有一份显示（同 .arm-dist 的做法）。
      document.querySelectorAll('.arm-arb').forEach((el) => {
        el.textContent = text;
        el.classList.remove('ok', 'warn', 'err');
        if (cls) el.classList.add(cls);
      });
    });
    sub('/vlm/result', 'std_msgs/msg/String', (msg) => {
      const el = $('arm-vlm-result');
      if (el) el.textContent = msg.data || '—';
      if (msg.data) addArmLog('VLM: ' + msg.data);
    });
    // KCF 手动框选: 在跟踪画面上拖拽框选 -> 发布像素框给 kcf_node 重新播种。
    kcfBboxPub = new ROSLIB.Topic({
      ros, name: '/kcf_node/select_bbox', messageType: 'sensor_msgs/msg/RegionOfInterest',
    });
    kcfBboxPub.advertise();
    // YOLO 抓取子页: 把识别到的物体渲染成按钮, 点击设为抓取目标(target_label)。
    sub('/yolo/detections', 'yolo_msgs/msg/DetectionArray', renderArmYoloButtons, { throttle_rate: 300 });

    // RRT 自主探索 (wheeltec_robot_rrt / wheeltec_rrt_msg)：边界选点发布 +
    // 前沿点流订阅。上一次连接的前沿数据属于旧会话，连接时清掉。
    rrtClickPub = new ROSLIB.Topic({
      ros, name: '/clicked_point', messageType: 'geometry_msgs/msg/PointStamped',
    });
    rrtClickPub.advertise();
    rrtDetected = [];
    rrtFrontiers = [];
    rrtFrontTime = 0;
    // RRT 树检出的前沿点（流式单点，高频 → 节流并只留最近 300 个画淡蓝点）
    sub('/detected_frontiers', 'geometry_msgs/msg/PointStamped', (msg) => {
      const p = (msg && msg.point) || null;
      if (!p) return;
      rrtDetected.push({ x: +p.x || 0, y: +p.y || 0 });
      if (rrtDetected.length > 300) rrtDetected.splice(0, rrtDetected.length - 300);
    }, { throttle_rate: 200 });
    // filter 聚类过滤后的候选探索目标（assigner 的输入）
    sub('/filtered_goal_points', 'wheeltec_rrt_msg/msg/PointArray', (msg) => {
      rrtFrontiers = ((msg && msg.points) || []).map((p) => ({ x: +p.x || 0, y: +p.y || 0 }));
      rrtFrontTime = Date.now();
    }, { throttle_rate: 500 });

    // 超声波转换 (wheeltec_ultrasonic): /Distance → 标准 Range + 点云。
    // 安装位姿经 base_footprint 系 TF 取 ultrasonic_A..F（静态 TF，随底盘
    // description launch 发布）；话题 3 秒内有数据 = 转换节点在线。
    usTf = makeTfClient(ros, 'base_footprint');
    usOnline = null;
    Object.keys(usRanges).forEach((k) => { delete usRanges[k]; });
    usPointsInfo = { n: 0, time: 0 };
    US_LABELS.forEach((lb) => {
      sub('/ultrasonic/' + lb, 'sensor_msgs/msg/Range', (msg) => {
        // 无效/超量程时 converter 发 Infinity，rosbridge 序列化成 null
        const r = (msg && typeof msg.range === 'number' && isFinite(msg.range)) ? msg.range : null;
        usRanges[lb] = { range: r, time: Date.now() };
      }, { throttle_rate: 200 });
    });
    sub('/ultrasonic/points', 'sensor_msgs/msg/PointCloud2', (msg) => {
      const n = (msg && msg.width ? msg.width : 0) * (msg && msg.height ? msg.height : 1);
      usPointsInfo = { n, time: Date.now() };
    }, { throttle_rate: 500 });

    // 建图 (wheeltec_robot_slam): Cartographer 跟踪位姿 + ORB-SLAM2 相机位姿。
    sub('/tracked_pose', 'geometry_msgs/msg/PoseStamped', (msg) => {
      const el = $('mp-carto-pose');
      if (!el || !msg.pose) return;
      const p = msg.pose.position || {};
      const yaw = yawFromQuat(msg.pose.orientation || { x: 0, y: 0, z: 0, w: 1 });
      el.textContent = `x=${(+p.x).toFixed(2)}  y=${(+p.y).toFixed(2)}  yaw=${(yaw * 180 / Math.PI).toFixed(1)}°`;
    }, { throttle_rate: 500 });
    sub('/RGBD/pose', 'geometry_msgs/msg/PoseStamped', (msg) => {
      const el = $('mp-orb-pose');
      if (!el || !msg.pose) return;
      const p = msg.pose.position || {};
      el.textContent = `x=${(+p.x).toFixed(2)}  y=${(+p.y).toFixed(2)}  z=${(+p.z).toFixed(2)}`;
    }, { throttle_rate: 500 });

    // Lidar health (fused + per-sensor) and optional YOLO detections.
    subscribeLidar();
    subscribeYolo();
    // 路径跟随 (wheeltec_path_follow): /followpath 实时路径
    subscribePathFollow();

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

  // 速度上限本地持久化——刷新页面不用重新拉滑块（越界的存量值丢弃）
  try {
    const sl = localStorage.getItem('teleop_lin_max');
    const sa = localStorage.getItem('teleop_ang_max');
    if (sl !== null && +sl >= +linMax.min && +sl <= +linMax.max) linMax.value = sl;
    if (sa !== null && +sa >= +angMax.min && +sa <= +angMax.max) angMax.value = sa;
  } catch (_) { /* ignore */ }
  linVal.textContent = (+linMax.value).toFixed(2);
  angVal.textContent = (+angMax.value).toFixed(2);

  linMax.addEventListener('input', () => {
    linVal.textContent = (+linMax.value).toFixed(2);
    try { localStorage.setItem('teleop_lin_max', linMax.value); } catch (_) { /* ignore */ }
  });
  angMax.addEventListener('input', () => {
    angVal.textContent = (+angMax.value).toFixed(2);
    try { localStorage.setItem('teleop_ang_max', angMax.value); } catch (_) { /* ignore */ }
  });

  let curVx = 0, curWz = 0;
  let activeBtn = null;
  let repeatTimer = null;
  let noConnWarnAt = 0;   // 未连接时按遥控的提示节流

  function publishCmd(vx, wz) {
    curVx = vx; curWz = wz;
    cmdReadout.textContent = `vx=${vx.toFixed(2)}, wz=${wz.toFixed(2)}`;
    if (!cmdVelPub) {
      // 没连上时遥控不能静默吞掉——按住连发只提示一次（5s 节流），
      // 松开归零的 (0,0) 不提示
      const now = Date.now();
      if ((vx || wz) && now - noConnWarnAt > 5000) {
        noConnWarnAt = now;
        toast('未连接 rosbridge，速度指令没有发出', 'warn');
      }
      return;
    }
    const msg = new ROSLIB.Message({
      linear: { x: vx, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: wz },
    });
    cmdVelPub.publish(msg);
    // 手动通道：nav_arbiter 据此打断 Nav2/VLA 的自主导航
    if (cmdVelManualPub) cmdVelManualPub.publish(msg);
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

  // ---------- 虚拟摇杆 ----------
  // 拖拽遥控：纵向=线速度（上正），横向=角速度（左正，与 WASD 的 A=左转
  // 一致）。Pointer Events 同时覆盖鼠标和触屏；按住时与键盘一样按 10 Hz
  // 连发（看门狗语义），松开/取消/失焦立即归零停车。
  let joyStop = () => {};
  const joyBase = $('joystick');
  const joyKnob = $('joystick-knob');
  if (joyBase && joyKnob) {
    let joyTimer = null;
    let joyVec = { x: 0, y: 0 }; // 归一化偏移，x 右正 y 下正（屏幕系）

    function setKnob(nx, ny) {
      const r = joyBase.clientWidth / 2;
      const max = r - joyKnob.offsetWidth / 2; // knob 不出底盘
      joyKnob.style.left = (r + nx * max) + 'px';
      joyKnob.style.top = (r + ny * max) + 'px';
    }

    function joyPublish() {
      publishCmd(-joyVec.y * (+linMax.value), -joyVec.x * (+angMax.value));
    }

    function joyMove(ev) {
      const rect = joyBase.getBoundingClientRect();
      const r = rect.width / 2;
      let nx = (ev.clientX - rect.left - r) / r;
      let ny = (ev.clientY - rect.top - r) / r;
      const len = Math.hypot(nx, ny);
      if (len > 1) { nx /= len; ny /= len; }
      joyVec = { x: nx, y: ny };
      setKnob(nx, ny);
    }

    function joyEnd() {
      joyVec = { x: 0, y: 0 };
      setKnob(0, 0);
      joyBase.classList.remove('dragging');
      if (joyTimer) { clearInterval(joyTimer); joyTimer = null; }
      publishCmd(0, 0);
    }
    joyStop = () => { if (joyBase.classList.contains('dragging')) joyEnd(); };

    joyBase.addEventListener('pointerdown', (ev) => {
      ev.preventDefault();
      joyBase.setPointerCapture(ev.pointerId);
      joyBase.classList.add('dragging');
      joyMove(ev);
      joyPublish();
      clearInterval(joyTimer);
      joyTimer = setInterval(joyPublish, 100);
    });
    joyBase.addEventListener('pointermove', (ev) => {
      if (joyBase.classList.contains('dragging')) joyMove(ev);
    });
    joyBase.addEventListener('pointerup', joyEnd);
    joyBase.addEventListener('pointercancel', joyEnd);
    // 与键盘遥控同样的安全语义：窗口失焦就再也收不到 pointerup 了
    window.addEventListener('blur', joyStop);
  }

  // ---------- 游戏手柄 ----------
  // Gamepad API：左摇杆驱动底盘（上=前进 左=左转），须勾选启用，避免
  // 手柄误碰乱发速度。浏览器不推杆量事件，只能 poll，同样 10 Hz。
  let gpStop = () => {};
  const gpEnable = $('gamepad-enable');
  const gpStatus = $('gamepad-status');
  if (gpEnable && gpStatus) {
    const GP_DEADZONE = 0.15;
    let gpTimer = null;
    let gpWasActive = false; // 上一帧杆量出过死区——回中时补发一帧零速

    function gpPad() {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      for (const p of pads) { if (p && p.connected) return p; }
      return null;
    }

    function gpUpdateStatus() {
      const pad = gpPad();
      gpStatus.textContent = pad ? pad.id : '未检测到手柄';
    }

    function gpPoll() {
      const pad = gpPad();
      if (!pad) return;
      const x = pad.axes[0] || 0;
      const y = pad.axes[1] || 0;
      if (Math.hypot(x, y) < GP_DEADZONE) {
        if (gpWasActive) { gpWasActive = false; publishCmd(0, 0); }
        return;
      }
      gpWasActive = true;
      publishCmd(-y * (+linMax.value), -x * (+angMax.value));
    }

    gpStop = () => {
      if (!gpEnable.checked && !gpTimer) return;
      gpEnable.checked = false;
      clearInterval(gpTimer);
      gpTimer = null;
      gpWasActive = false;
    };

    gpEnable.addEventListener('change', () => {
      if (gpEnable.checked) {
        clearInterval(gpTimer);
        gpTimer = setInterval(gpPoll, 100);
        gpUpdateStatus();
        toast(gpPad() ? '手柄遥控已启用（左摇杆驱动）' : '手柄遥控已启用，等待手柄接入（接好后动一下摇杆）', 'ok');
      } else {
        const wasActive = gpWasActive;
        gpStop();
        if (wasActive) publishCmd(0, 0);
        toast('手柄遥控已关闭', 'warn');
      }
    });

    window.addEventListener('gamepadconnected', (e) => {
      gpUpdateStatus();
      toast('检测到手柄: ' + e.gamepad.id, 'ok');
    });
    window.addEventListener('gamepaddisconnected', () => {
      gpUpdateStatus();
      if (gpWasActive) { gpWasActive = false; publishCmd(0, 0); }
    });
  }

  // Emergency stop button: always publish (0,0), regardless of held inputs.
  // 同时切断所有会继续连发的遥控源（按键九宫格/键盘/摇杆/手柄），否则
  // 急停发完零速后它们的 10 Hz 循环又会把速度顶回去。
  const eStopBtn = $('e-stop');
  if (eStopBtn) {
    eStopBtn.addEventListener('click', () => {
      heldKeys.clear();
      stopKeyboardLoop(false);
      stopBtn();
      joyStop();
      gpStop();
      // Send the stop a few times in case rosbridge / WiFi drops one.
      publishCmd(0, 0);
      setTimeout(() => publishCmd(0, 0), 50);
      setTimeout(() => publishCmd(0, 0), 150);
      toast('已急停：连发 3 帧零速到 /cmd_vel', 'warn');
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
  // "2D 导航目标"工具（等价 RViz 2D Nav Goal）：开关 + 进行中拖拽的取消
  // 钩子（拖拽状态在 buildViewer 闭包里，Esc 退出工具时要把预览箭头清掉）
  let navGoalMode = false;
  let navGoalCancelDrag = () => {};

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
      // frame 在它自己所属 TF 树根下的位姿（fixed frame 与机器人树不连通时
      // 的兜底显示用）。frame 自己就是树根（只当过 parent）时返回原点位姿；
      // 完全不在 /tf 里时返回 null。
      lookupInOwnRoot(frame) {
        const f = norm(frame);
        if (edges[f]) {
          const a = toRoot(f);
          return { root: a.root, pose: a.pose };
        }
        for (const k in edges) {
          if (edges[k].parent === f) return { root: f, pose: TF_IDENTITY };
        }
        return null;
      },
      // 当前收到的所有 TF 树根（odom_combined / map / 未连底盘时的
      // base_footprint 等），用于状态栏提示用户该填什么 fixed frame。
      roots() {
        const set = new Set();
        for (const k in edges) set.add(toRoot(k).root);
        return Array.from(set).sort();
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

  // ---- 机器人模型 (URDF) 图层 ----
  // 经 rosbridge 调 <node>/get_parameters 取已展开的 robot_description（xacro
  // 在 launch 时已处理），浏览器 DOMParser 解析 link/visual。每个 link 的位姿
  // 不做关节运动学——robot_state_publisher 已把所有 link 发进 /tf，直接用
  // 既有的 tfClient.lookup(link) 摆放。网格经 web_server 的 /pkg/ 路由加载。
  function urdfRpyToQuat(rpyStr) {
    const p = String(rpyStr || '0 0 0').trim().split(/\s+/).map(Number);
    // URDF rpy 是固定轴 XYZ（R = Rz·Ry·Rx），对应 THREE 内旋序 'ZYX'
    return new THREE.Quaternion().setFromEuler(
      new THREE.Euler(p[0] || 0, p[1] || 0, p[2] || 0, 'ZYX'));
  }
  function urdfVec(str, def) {
    const p = String(str || '').trim().split(/\s+/).map(Number);
    return [p[0] || def, p[1] || def, p[2] || def];
  }
  function meshUrlFromPackageUri(uri) {
    const m = /^package:\/\/([^/]+)\/(.+)$/.exec((uri || '').trim());
    return m ? '/pkg/' + m[1] + '/' + m[2] : null;
  }

  function makeUrdfLayer(nodeName) {
    let disposed = false;
    const linkGroups = new Map();   // link 名 -> THREE.Group
    const disposables = [];         // geometry/material 待释放

    function materialFor(colorArr) {
      const c = colorArr || [0.55, 0.6, 0.66, 1];
      const mat = new THREE.MeshLambertMaterial({
        color: new THREE.Color(c[0], c[1], c[2]),
        transparent: c[3] < 0.99,
        opacity: c[3],
      });
      disposables.push(mat);
      return mat;
    }

    function buildFromXml(xml) {
      if (disposed || !viewer) return;
      const doc = new DOMParser().parseFromString(xml, 'text/xml');
      const robot = doc.querySelector('robot');
      if (!robot) { setViewerStatus('URDF 解析失败: ' + nodeName); return; }
      // robot 级命名材质表（visual 里可只写名字引用）
      const namedColors = {};
      Array.from(robot.children).filter((n) => n.tagName === 'material').forEach((m) => {
        const c = m.querySelector('color');
        if (m.getAttribute('name') && c) {
          namedColors[m.getAttribute('name')] =
            String(c.getAttribute('rgba') || '').trim().split(/\s+/).map(Number);
        }
      });

      let linkCount = 0;
      Array.from(robot.children).filter((n) => n.tagName === 'link').forEach((link) => {
        const visuals = Array.from(link.children).filter((n) => n.tagName === 'visual');
        if (!visuals.length) return;
        const grp = new THREE.Group();
        grp.visible = false;   // 等 TF 就位再显示
        visuals.forEach((vis) => {
          const holder = new THREE.Group();
          const origin = Array.from(vis.children).find((n) => n.tagName === 'origin');
          if (origin) {
            const xyz = urdfVec(origin.getAttribute('xyz'), 0);
            holder.position.set(xyz[0], xyz[1], xyz[2]);
            holder.quaternion.copy(urdfRpyToQuat(origin.getAttribute('rpy')));
          }
          // 颜色：visual 内联 color > 命名材质 > 默认灰
          let colorArr = null;
          const matEl = vis.querySelector('material');
          if (matEl) {
            const c = matEl.querySelector('color');
            if (c) colorArr = String(c.getAttribute('rgba') || '').trim().split(/\s+/).map(Number);
            else if (matEl.getAttribute('name')) colorArr = namedColors[matEl.getAttribute('name')] || null;
          }
          const geomEl = vis.querySelector('geometry');
          const meshEl = geomEl && geomEl.querySelector('mesh');
          const boxEl = geomEl && geomEl.querySelector('box');
          const cylEl = geomEl && geomEl.querySelector('cylinder');
          const sphEl = geomEl && geomEl.querySelector('sphere');
          if (meshEl) {
            const url = meshUrlFromPackageUri(meshEl.getAttribute('filename'));
            const scale = urdfVec(meshEl.getAttribute('scale'), 1);
            if (url && /\.stl$/i.test(url) && THREE.STLLoader) {
              new THREE.STLLoader().load(url, (geom) => {
                if (disposed) { geom.dispose(); return; }
                disposables.push(geom);
                const mesh = new THREE.Mesh(geom, materialFor(colorArr));
                mesh.scale.set(scale[0], scale[1], scale[2]);
                holder.add(mesh);
              }, undefined, () => setViewerStatus('URDF 网格加载失败: ' + url +
                '（web_server 是否已重编译启用 /pkg/ 路由？）'));
            } else if (url && /\.dae$/i.test(url) && THREE.ColladaLoader) {
              new THREE.ColladaLoader().load(url, (dae) => {
                if (disposed || !dae || !dae.scene) return;
                dae.scene.scale.set(scale[0], scale[1], scale[2]);
                holder.add(dae.scene);
              }, undefined, () => setViewerStatus('URDF 网格加载失败: ' + url));
            }
          } else if (boxEl) {
            const s = urdfVec(boxEl.getAttribute('size'), 0.1);
            const geom = new THREE.BoxGeometry(s[0], s[1], s[2]);
            disposables.push(geom);
            holder.add(new THREE.Mesh(geom, materialFor(colorArr)));
          } else if (cylEl) {
            const r = parseFloat(cylEl.getAttribute('radius')) || 0.05;
            const l = parseFloat(cylEl.getAttribute('length')) || 0.1;
            const geom = new THREE.CylinderGeometry(r, r, l, 24);
            disposables.push(geom);
            const mesh = new THREE.Mesh(geom, materialFor(colorArr));
            mesh.rotation.x = Math.PI / 2;   // URDF 圆柱沿 Z，THREE 沿 Y
            holder.add(mesh);
          } else if (sphEl) {
            const r = parseFloat(sphEl.getAttribute('radius')) || 0.05;
            const geom = new THREE.SphereGeometry(r, 20, 14);
            disposables.push(geom);
            holder.add(new THREE.Mesh(geom, materialFor(colorArr)));
          }
          grp.add(holder);
        });
        viewer.scene.add(grp);
        linkGroups.set(link.getAttribute('name'), grp);
        linkCount++;
      });
      setViewerStatus('');
      console.info('URDF 模型已加载: %s（%d 个可视 link）', nodeName, linkCount);
    }

    // 取 robot_description 参数（xacro 已在 launch 时展开为纯 URDF）
    const req = new ROSLIB.ServiceRequest({ names: ['robot_description'] });
    paramService(nodeName, 'get_parameters').callService(req, (res) => {
      const val = res && res.values && res.values[0];
      const xml = (val && val.type === PT_STRING) ? val.string_value : '';
      if (!xml) { setViewerStatus('URDF: ' + nodeName + ' 没有 robot_description 参数'); return; }
      buildFromXml(xml);
    }, (err) => setViewerStatus('URDF 获取失败 (' + nodeName + '): ' + err +
      '（robot_state_publisher 是否在跑？）'));

    // 每 100ms 按 TF 摆放各 link（robot_state_publisher 已发布全部 link 的 TF）。
    // fixed frame 与机器人 TF 树不连通（fixed frame 填错 / 底盘 EKF 没启动）时
    // 不再整车隐藏：兜底以机器人自身树根为原点显示，并在状态栏说明原因。
    let fellBack = false;
    const timer = setInterval(() => {
      if (!tfClient) return;
      let connected = false;
      let fallbackRoot = '';
      linkGroups.forEach((grp, name) => {
        let tf = tfClient.lookup(name);
        if (tf) {
          connected = true;
        } else {
          const own = tfClient.lookupInOwnRoot(name);
          if (own) { tf = own.pose; fallbackRoot = own.root; }
        }
        if (!tf) { grp.visible = false; return; }
        grp.visible = true;
        grp.position.set(tf.translation.x, tf.translation.y, tf.translation.z);
        grp.quaternion.set(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
      });
      if (!connected && fallbackRoot) {
        if (!fellBack) {
          fellBack = true;
          setViewerStatus('URDF(' + nodeName + '): fixed frame "' + tfClient.fixedFrame +
            '" 与机器人 TF 树不连通（机器人树根: ' + fallbackRoot + '）——模型暂以 ' +
            fallbackRoot + ' 为原点显示。当前 TF 根: ' + tfClient.roots().join(', ') +
            '；底盘正常时应把 Fixed frame 填为 odom_combined（EKF 发布），并确认底盘 launch 已启动');
        }
      } else if (fellBack && connected) {
        fellBack = false;
        setViewerStatus('');
      }
    }, 100);

    return {
      dispose() {
        disposed = true;
        clearInterval(timer);
        linkGroups.forEach((grp) => { try { viewer.scene.remove(grp); } catch (_) { /* ignore */ } });
        linkGroups.clear();
        disposables.forEach((d) => { try { d.dispose(); } catch (_) { /* ignore */ } });
        disposables.length = 0;
      },
    };
  }

  // ---- SLAM 地图 (/map) 图层 ----
  // 复用 VLA 航点卡维护的全局 wpMap（GetMap 服务 + /map 话题，含预翻转位图），
  // 以 CanvasTexture 平面铺在地面，位姿 = map 帧原点经 TF 变换到 fixed frame。
  function makeViewerMapLayer() {
    let mesh = null;
    let lastBmp = null;
    let warnedNoTf = false;   // "有地图无 map TF" 提示只发一次，TF 出现即清
    function destroyMesh() {
      if (!mesh) return;
      try {
        if (viewer) viewer.scene.remove(mesh);
        mesh.geometry.dispose();
        if (mesh.material.map) mesh.material.map.dispose();
        mesh.material.dispose();
      } catch (_) { /* ignore */ }
      mesh = null;
    }
    const timer = setInterval(() => {
      if (!viewer) return;
      if (!wpMap) { if (mesh) mesh.visible = false; return; }
      if (wpMap.bmp !== lastBmp) {   // 首次或 SLAM 更新了地图 -> 重建贴图平面
        destroyMesh();
        lastBmp = wpMap.bmp;
        const tex = new THREE.CanvasTexture(wpMap.bmp);
        tex.minFilter = THREE.LinearFilter;   // NPOT 画布禁 mipmap
        tex.generateMipmaps = false;
        const geo = new THREE.PlaneGeometry(wpMap.w * wpMap.res, wpMap.h * wpMap.res);
        const mat = new THREE.MeshBasicMaterial({
          map: tex, transparent: true, opacity: 0.85, depthWrite: false,
        });
        mesh = new THREE.Mesh(geo, mat);
        mesh.renderOrder = -1;   // 垫底，雷达点/模型画在其上
        viewer.scene.add(mesh);
      }
      // 地图平面中心在 map 帧的坐标 -> 经 TF 转到 fixed frame
      const tf = tfClient ? tfClient.lookup('map') : null;
      if (!tf) {
        mesh.visible = false;
        // 有地图数据但没 map TF（SLAM/AMCL 没跑）时不再静默隐藏，说明原因。
        // 不抢占别的图层已写的状态。
        if (!warnedNoTf) {
          warnedNoTf = true;
          const cur = (($('viewer-status') || {}).textContent) || '';
          if (!cur) {
            setViewerStatus('SLAM 地图: 已有 ' + wpMap.w + '×' + wpMap.h +
              ' 地图数据，但无 map→fixed frame 的 TF（SLAM/AMCL 未跑），地图层暂隐藏' +
              '——把 Fixed frame 填成 map 可直接查看');
          }
        }
        return;
      }
      if (warnedNoTf) {
        warnedNoTf = false;
        const cur = (($('viewer-status') || {}).textContent) || '';
        if (cur.indexOf('SLAM 地图:') === 0) setViewerStatus('');
      }
      const cx = wpMap.ox + wpMap.w * wpMap.res / 2;
      const cy = wpMap.oy + wpMap.h * wpMap.res / 2;
      const p = quatRotateVec(tf.rotation, { x: cx, y: cy, z: 0 });
      mesh.position.set(p.x + tf.translation.x, p.y + tf.translation.y,
        p.z + tf.translation.z - 0.01);   // 略低于地面防 Z-fighting
      mesh.quaternion.set(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
      mesh.visible = true;
    }, 500);
    return {
      dispose() { clearInterval(timer); destroyMesh(); },
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

    // URDF 网格用受光材质，需要光源（点云/线条材质不受影响）。
    scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.6);
    dirLight.position.set(3, -2, 6);
    scene.add(dirLight);

    viewer = { scene, camera, renderer, controls, host };

    // --- "2D 导航目标"手势：按下选位置、拖动定朝向，松开确认发布 ---
    // 模式开着时 OrbitControls 已被禁用，指针事件归这里。目标点取相机
    // 射线与 fixed frame 地面 z=0 的交点（地图平面就铺在那），发布前再经
    // map TF 转回 map 系。
    const navRay = new THREE.Raycaster();
    const navPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    let navDrag = null;    // 按下点（fixed frame 地面坐标），null = 没在拖
    let navArrow = null;   // 朝向预览箭头

    function navGroundPoint(ev) {
      const rect = renderer.domElement.getBoundingClientRect();
      const ndc = new THREE.Vector2(
        ((ev.clientX - rect.left) / rect.width) * 2 - 1,
        -((ev.clientY - rect.top) / rect.height) * 2 + 1);
      navRay.setFromCamera(ndc, camera);
      const hit = new THREE.Vector3();
      return navRay.ray.intersectPlane(navPlane, hit) ? hit : null;
    }
    function navRemoveArrow() {
      if (navArrow) { try { scene.remove(navArrow); } catch (_) { /* ignore */ } navArrow = null; }
    }
    navGoalCancelDrag = () => { navDrag = null; navRemoveArrow(); };

    renderer.domElement.addEventListener('pointerdown', (ev) => {
      if (!navGoalMode) return;
      const p = navGroundPoint(ev);
      if (!p) return;   // 视角太平射不到地面——忽略这次按下
      ev.preventDefault();
      renderer.domElement.setPointerCapture(ev.pointerId);
      navDrag = { x: p.x, y: p.y };
      navRemoveArrow();
      navArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(p.x, p.y, 0.02), 0.5, 0x3fb950, 0.2, 0.12);
      scene.add(navArrow);
    });
    renderer.domElement.addEventListener('pointermove', (ev) => {
      if (!navDrag || !navArrow) return;
      const p = navGroundPoint(ev);
      if (!p) return;
      const dx = p.x - navDrag.x, dy = p.y - navDrag.y;
      const len = Math.hypot(dx, dy);
      if (len > 1e-3) {
        navArrow.setDirection(new THREE.Vector3(dx / len, dy / len, 0));
        navArrow.setLength(Math.max(0.4, len), 0.2, 0.12);
      }
    });
    renderer.domElement.addEventListener('pointerup', (ev) => {
      if (!navDrag) return;
      const start = navDrag;
      navDrag = null;
      const p = navGroundPoint(ev);
      navRemoveArrow();
      if (!navGoalMode) return;   // 拖到一半 Esc 退出了
      const end = p || start;
      const dx = end.x - start.x, dy = end.y - start.y;
      let yaw;
      if (Math.hypot(dx, dy) >= 0.15) {
        yaw = Math.atan2(dy, dx);
      } else {
        // 没拖出朝向：默认 = 从小车当前位置指向目标点（拿不到就 0）
        const base = tfClient ? tfClient.lookup('base_footprint') : null;
        yaw = base
          ? Math.atan2(start.y - base.translation.y, start.x - base.translation.x)
          : 0;
      }
      publishNavGoalFixed(start.x, start.y, yaw);
    });
    renderer.domElement.addEventListener('pointercancel', navGoalCancelDrag);

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

  const NAV_GOAL_HINT = '2D 导航目标: 在地面上按下选位置、拖动定朝向，松开确认发布 /goal_pose（Esc 退出）';
  function setNavGoalMode(on) {
    navGoalMode = !!on;
    if (!navGoalMode) navGoalCancelDrag();
    const btn = $('vw-nav-goal');
    if (btn) btn.classList.toggle('tool-active', navGoalMode);
    const host = $('viewer');
    if (host) host.classList.toggle('nav-goal-mode', navGoalMode);
    if (viewer && viewer.controls) viewer.controls.enabled = !navGoalMode;
    if (navGoalMode) {
      setViewerStatus(NAV_GOAL_HINT);
    } else {
      const cur = (($('viewer-status') || {}).textContent) || '';
      if (cur.indexOf('2D 导航目标:') === 0) setViewerStatus('');
    }
  }

  // fixed frame 地面坐标 → map 系 /goal_pose。与其它发目标入口同样带二次
  // 确认；成功后与 RViz 一致自动退出工具，防误点连发。
  function publishNavGoalFixed(fx, fy, yawFixed) {
    if (!goalPosePub) { toast('未连接 rosbridge，无法发布导航目标', 'err'); return; }
    const mapTf = tfClient ? tfClient.lookup('map') : null;
    if (!mapTf) { toast('无 map→fixed frame 的 TF（SLAM/AMCL 未跑），无法发布 map 系导航目标', 'err'); return; }
    // p_fixed = R_map·p_map + t_map  =>  p_map = R_map⁻¹·(p_fixed − t_map)
    const inv = tfInverse(mapTf);
    const pm = quatRotateVec(inv.rotation, { x: fx, y: fy, z: 0 });
    const x = pm.x + inv.translation.x;
    const y = pm.y + inv.translation.y;
    const yaw = yawFixed - yawFromQuat(mapTf.rotation);
    if (!window.confirm(`导航到 x=${x.toFixed(2)}, y=${y.toFixed(2)}（map 系）？小车将开始移动。`)) return;
    goalPosePub.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      pose: { position: { x, y, z: 0 }, orientation: quatFromYaw(yaw) },
    }));
    toast(`导航目标已发布 (x=${x.toFixed(2)}, y=${y.toFixed(2)})，小车开始移动`, 'ok');
    setNavGoalMode(false);
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
    // scan / odom layers can't be placed and the view stays empty. 常驻检测：
    // 后启动底盘（EKF）时提示自动消失；提示里列出当前 TF 树根，便于发现
    // odom / odom_combined 这类填错的 fixed frame。URDF 层的兜底提示更具体，
    // 不覆盖它。
    setViewerStatus(`等待 frame "${fixedFrame}" …`);
    let waited = 0;
    const statusTimer = setInterval(() => {
      waited += 500;
      const cur = (($('viewer-status') || {}).textContent) || '';
      const mine = !cur || cur.indexOf('等待 frame') === 0 || cur.indexOf('未收到 frame') === 0;
      if (tfClient && tfClient.knows(fixedFrame)) {
        if (mine && cur) setViewerStatus('');
      } else if (waited >= 5000 && mine) {
        const roots = tfClient ? tfClient.roots() : [];
        setViewerStatus(`未收到 frame "${fixedFrame}" 的 TF` +
          (roots.length
            ? `——当前 TF 树根: ${roots.join(', ')}（Fixed frame 填其中之一即可；底盘的根是 EKF 发布的 odom_combined）`
            : ' — 检查底盘 launch（EKF）/ robot_state_publisher 是否启动'));
      }
    }, 500);
    viewerLayers.push({ dispose() { clearInterval(statusTimer); } });

    if (scanTopic) {
      viewerLayers.push(makeScanLayer(scanTopic));
    }

    // SLAM 地图层（复用 wpMap 全局地图数据；没有时尝试 GetMap 拉一次）
    const showMap = $('vw-show-map');
    if (!showMap || showMap.checked) {
      viewerLayers.push(makeViewerMapLayer());
      if (!wpMap && ros) fetchWpMap();
    }

    // 机器人模型层：每个 robot_state_publisher 节点一层（底盘/机械臂可并列）
    const showUrdf = $('vw-show-urdf');
    if (!showUrdf || showUrdf.checked) {
      const nodes = (($('vw-urdf') || {}).value || '/robot_state_publisher')
        .split(/[,，]/).map((s) => s.trim()).filter(Boolean);
      nodes.forEach((n) => viewerLayers.push(makeUrdfLayer(n)));
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
  const navGoalBtn = $('vw-nav-goal');
  if (navGoalBtn) navGoalBtn.addEventListener('click', () => {
    if (navGoalMode) { setNavGoalMode(false); return; }
    if (!viewer) { toast('3D 视图未初始化，请先连接 rosbridge', 'warn'); return; }
    if (!tfClient || !tfClient.lookup('map')) {
      toast('无 map→fixed frame 的 TF（SLAM/AMCL 未跑），无法发布 map 系导航目标', 'err');
      return;
    }
    setNavGoalMode(true);
  });
  // Esc 随时退出工具（拖到一半也行，预览箭头一并清掉）
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && navGoalMode) setNavGoalMode(false);
  });
  // 勾选机器人模型 / SLAM 地图开关即时生效（与"应用"等价，重建图层）
  ['vw-show-urdf', 'vw-show-map'].forEach((id) => {
    const el = $(id);
    if (el) el.addEventListener('change', rebuildViewer);
  });
  $('vw-clear-path').addEventListener('click', () => {
    odomPoints.length = 0;
    if (odomPath) {
      odomPath.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
      odomPath.geometry.setDrawRange(0, 0);
    }
  });

  // ---------- 点击放大查看（lightbox）----------
  // 所有 MJPEG 流画面（.cam-img）点一下铺满全屏看细节——直接复用同一个
  // 流 URL（多开一路 web_video_server 客户端而已）。KCF 框选画面
  // （.bbox-select 容器内）按下是框选语义，不抢；占位图没内容不放大。
  const lightbox = $('lightbox');
  const lightboxImg = $('lightbox-img');
  if (lightbox && lightboxImg) {
    const closeLightbox = () => {
      lightbox.hidden = true;
      lightboxImg.removeAttribute('src');   // 断开放大那路流，省带宽
    };
    document.addEventListener('click', (e) => {
      const img = e.target.closest ? e.target.closest('img.cam-img') : null;
      if (!img || img.closest('.bbox-select')) return;
      if ((img.src || '').indexOf('placeholder') !== -1) return;
      lightboxImg.src = img.src;
      lightbox.hidden = false;
    });
    lightbox.addEventListener('click', closeLightbox);
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && !lightbox.hidden) closeLightbox();
    });
  }

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

  // ---------- 单帧快照轮询（替代 multipart/x-mixed-replace 长连接）----------
  // 浏览器经 WSL2 的 localhost 转发 / 某些代理访问时, MJPEG 长连接常被缓冲住:
  // HTTP 200 到了但后续帧刷不到 <img> -> onerror -> "无法加载流"。改用 /snapshot
  // 短请求按帧轮询(每帧独立请求、立即结束), 跨浏览器/转发/代理都稳, 后端一有帧
  // 就出图、不必手动刷新。隐藏页签的 <img> 跳过请求以省带宽。
  const snapState = new Map();   // img -> { alive, firstOk, t }(链式轮询状态)

  function snapshotUrl(topic) {
    const q = Math.max(1, Math.min(100, parseInt(camQuality.value, 10) || 60));
    const params = `topic=${encodeURIComponent(topic)}&quality=${q}&_=${Date.now()}`;
    // 默认走"同源代理" /video/snapshot —— 由 web_server(:8000) 在 WSL 本地转发到
    // web_video_server, 浏览器只连 8000(能过 WSL2 localhost 转发), 避免直连 8081 空响应。
    // 若手填了"基址"覆盖(camBase, 如局域网直连/反代场景), 则按它直连 /snapshot。
    const override = (camBase && camBase.value || '').trim();
    if (override) return `${override.replace(/\/+$/, '')}/snapshot?${params}`;
    return `/video/snapshot?${params}`;
  }

  function stopSnapshot(img) {
    const st = snapState.get(img);
    if (st) { st.alive = false; if (st.t) clearTimeout(st.t); snapState.delete(img); }
  }

  // 自限速链式轮询: 每路面板最多 1 个在途请求, 上一张完成后才发下一张, 失败则退避。
  // (旧版用固定 setInterval 不等返回就发, 失败时请求疯狂堆积, 把经 WSL 转发的
  //  web_video_server 冲爆 -> ERR_EMPTY_RESPONSE。改链式后最多 ~每面板 1 个并发。)
  function startSnapshot(img, topic, errEl) {
    stopSnapshot(img);
    if (!img || !topic) return;
    const st = { alive: true, firstOk: false, t: null };
    snapState.set(img, st);
    const next = (delay) => { if (st.alive) st.t = setTimeout(loop, delay); };
    function loop() {
      if (!st.alive) return;
      if (!img.offsetParent) { next(300); return; }   // 隐藏页签: 慢轮询, 不请求
      const pre = new Image();
      pre.onload = () => {
        if (!st.alive) return;
        img.src = pre.src;
        if (!st.firstOk) { st.firstOk = true; if (errEl) errEl.hidden = true; }
        next(120);                                     // 成功: 隔 120ms 再来(≈8fps 上限)
      };
      pre.onerror = () => {
        if (!st.alive) return;
        if (!st.firstOk && errEl) {
          errEl.hidden = false;
          errEl.textContent = '无法加载流，检查 web_video_server 与话题';
        }
        next(500);                                     // 失败: 退避 500ms, 不猛刷
      };
      pre.src = snapshotUrl(topic);
    }
    loop();
  }

  function applyCam(slot) {
    const img = document.querySelector(`.cam-img[data-slot="${slot}"]`);
    const topicInput = document.querySelector(`.cam-topic[data-slot="${slot}"]`);
    const enable = document.querySelector(`.cam-enable[data-slot="${slot}"]`);
    const errEl = document.querySelector(`.cam-err[data-slot="${slot}"]`);
    if (!img || !topicInput || !enable) return;
    errEl.hidden = true;
    if (!enable.checked) { stopSnapshot(img); showPlaceholder(slot, '已禁用'); return; }
    const topic = topicInput.value.trim();
    if (!topic) { stopSnapshot(img); showPlaceholder(slot, '未设置 topic'); return; }
    startSnapshot(img, topic, errEl);
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
      stopSnapshot(img);
      img.src = CAM_PLACEHOLDER;
      if (errEl) { errEl.hidden = false; errEl.textContent = '未设置 topic'; }
      return;
    }
    startSnapshot(img, topic, errEl);
  }
  function applyAllFnStreams() {
    document.querySelectorAll('.fn-img').forEach(applyFnStream);
  }
  // 页签/子页激活时调用：只启动 root 下"当前可见且还停在占位图"的流。
  // 已在播放的流不重启（避免来回切页时 MJPEG 闪断）；出错回落到占位图的流
  // 会在下次切回时自动重试；手动重启用各自的"刷新"按钮。
  function kickVisibleFnStreams(root) {
    if (!root) return;
    root.querySelectorAll('.fn-img').forEach((img) => {
      if (!img.offsetParent) return;            // display:none
      if (snapState.has(img)) return;           // 已在轮询
      applyFnStream(img);
    });
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
  // 首页（系统总览）默认可见，其"相机原始流"卡片也立即拉流。
  kickVisibleFnStreams($('panel-overview'));

  // ---------- 视频流兜底：定时确保"可见但还没在轮询"的面板启动起来 ----------
  // 快照轮询本身逐帧自愈(后端晚起/重启都会自动出图), 这里只是兜底:
  // 把当前可见、却还没在轮询的相机/功能流启动起来(切页/首次显示时补漏)。
  function retryErroredStreams() {
    kickVisibleFnStreams(document);                       // fn-img(功能页/机械臂仿真)
    document.querySelectorAll('.cam-img[data-slot]').forEach((img) => {
      if (!img.offsetParent) return;                      // 隐藏页签
      if (snapState.has(img)) return;                     // 已在轮询
      const slot = img.dataset.slot;
      const enable = document.querySelector(`.cam-enable[data-slot="${slot}"]`);
      if (enable && !enable.checked) return;              // 用户主动禁用
      applyCam(slot);
    });
  }
  setInterval(retryErroredStreams, 5000);

  // ---------- 仿真启停 (sim_launcher: /sim_launch/cmd + /sim_launch/status) ----------
  function sendSimCmd(text) {
    if (!simLaunchPub) { alert('未连接 rosbridge，无法发送仿真命令'); return; }
    simLaunchPub.publish(new ROSLIB.Message({ data: text }));
  }
  document.querySelectorAll('.sim-start').forEach((btn) => {
    btn.addEventListener('click', () => {
      const key = btn.dataset.key;
      const sel = document.getElementById(btn.dataset.worldSel);
      const world = sel ? sel.value : '';
      if (key && world) sendSimCmd('start ' + key + ' ' + world);
    });
  });
  document.querySelectorAll('.sim-stop').forEach((btn) => {
    btn.addEventListener('click', () => {
      if (btn.dataset.key) sendSimCmd('stop ' + btn.dataset.key);
    });
  });
  // 后端 sim_launcher 广播的状态(JSON) -> 更新各页"仿真:"徽标。
  function renderSimLaunch(obj) {
    const items = (obj && obj.items) || {};
    const enable = !(obj && obj.enable === false);
    document.querySelectorAll('.sim-launch-status').forEach((el) => {
      const it = items[el.dataset.key];
      el.classList.remove('ctrl-src-idle', 'ctrl-src-active');
      if (it && it.running) {
        el.textContent = '仿真: 运行中 · ' + (it.world || '');
        el.classList.add('ctrl-src-active');
      } else {
        el.textContent = enable ? '仿真: 未运行' : '仿真: 只读(未启用启停)';
        el.classList.add('ctrl-src-idle');
      }
    });
  }

  // ---------- Nav2 导航 (/initialpose · /goal_pose · 取消) ----------
  function nav2CurPose() {
    const tf = wpTf ? wpTf.lookup('base_footprint') : null;
    if (!tf) return null;
    return { x: tf.translation.x, y: tf.translation.y, yaw: yawFromQuat(tf.rotation) };
  }
  function nav2ReadXYYaw(prefix) {
    return {
      x: parseFloat($(prefix + '-x').value) || 0,
      y: parseFloat($(prefix + '-y').value) || 0,
      yaw: (parseFloat($(prefix + '-yaw').value) || 0) * Math.PI / 180,
    };
  }
  function nav2FillCur(prefix, msgId) {
    const p = nav2CurPose();
    if (!p) { setCardMsg(msgId, '拿不到 map→base_footprint TF（定位未就绪？）'); return; }
    $(prefix + '-x').value = p.x.toFixed(2);
    $(prefix + '-y').value = p.y.toFixed(2);
    $(prefix + '-yaw').value = (p.yaw * 180 / Math.PI).toFixed(0);
    const cur = $('nav2-cur');
    if (cur) cur.textContent = `x=${p.x.toFixed(2)} y=${p.y.toFixed(2)} yaw=${(p.yaw * 180 / Math.PI).toFixed(1)}°`;
    setCardMsg(msgId, '已填入当前位姿');
  }
  function quatFromYaw(yaw) { return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }; }
  (function wireNav2() {
    const initCur = $('nav2-init-cur'); if (initCur) initCur.addEventListener('click', () => nav2FillCur('nav2-init', 'nav2-init-msg'));
    const goalCur = $('nav2-goal-cur'); if (goalCur) goalCur.addEventListener('click', () => nav2FillCur('nav2-goal', 'nav2-goal-msg'));
    const initSend = $('nav2-init-send');
    if (initSend) initSend.addEventListener('click', () => {
      if (!initialPosePub) { setCardMsg('nav2-init-msg', '未连接 rosbridge'); return; }
      const p = nav2ReadXYYaw('nav2-init');
      const cov = new Array(36).fill(0); cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.0685;
      initialPosePub.publish(new ROSLIB.Message({
        header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
        pose: { pose: { position: { x: p.x, y: p.y, z: 0 }, orientation: quatFromYaw(p.yaw) }, covariance: cov },
      }));
      setCardMsg('nav2-init-msg', `已发布 /initialpose (x=${p.x.toFixed(2)}, y=${p.y.toFixed(2)})`);
    });
    const goalSend = $('nav2-goal-send');
    if (goalSend) goalSend.addEventListener('click', () => {
      if (!goalPosePub) { setCardMsg('nav2-goal-msg', '未连接 rosbridge'); return; }
      const p = nav2ReadXYYaw('nav2-goal');
      if (!window.confirm(`导航到 x=${p.x.toFixed(2)}, y=${p.y.toFixed(2)}？小车将开始移动。`)) return;
      goalPosePub.publish(new ROSLIB.Message({
        header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
        pose: { position: { x: p.x, y: p.y, z: 0 }, orientation: quatFromYaw(p.yaw) },
      }));
      setCardMsg('nav2-goal-msg', `已发布 /goal_pose (x=${p.x.toFixed(2)}, y=${p.y.toFixed(2)})`);
      toast(`导航目标已发布 (x=${p.x.toFixed(2)}, y=${p.y.toFixed(2)})，小车开始移动`, 'ok');
    });
    const cancel = $('nav2-cancel');
    if (cancel) cancel.addEventListener('click', () => {
      if (!cmdVelManualPub) { setCardMsg('nav2-goal-msg', '未连接 rosbridge'); return; }
      cmdVelManualPub.publish(new ROSLIB.Message({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }));
      setCardMsg('nav2-goal-msg', '已发零速到 /cmd_vel_manual（nav_arbiter 取消当前导航目标）');
      toast('已请求取消当前导航目标', 'warn');
    });
  })();

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

  // 通用时间线：VLA 时间线与机械臂事件共用同一渲染（时间戳 + 文本 + 环形截断）。
  function appendTimeline(viewId, maxLines, text) {
    const view = $(viewId);
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
    while (view.childElementCount > maxLines) view.removeChild(view.firstChild);
    view.scrollTop = view.scrollHeight;
  }

  function addVlaStatus(text) {
    appendTimeline('vla-timeline', VLA_MAX_LINES, text);
  }

  // 导航仲裁状态（nav_arbiter/status: "MANUAL|FUNC|AUTO: 说明"）。
  // 优先级 手动 > 功能模块(KCF/巡线/YOLO) > Nav2/VLA；指标实时刷，
  // 层级切换的边沿写进 VLA 时间线。
  let navArbState = '';
  function renderNavArbiter(msg) {
    const text = (msg.data || '').trim();
    const stripped = text.replace(/^(MANUAL|FUNC|AUTO):\s*/i, '');
    const state = /^MANUAL/i.test(text) ? 'MANUAL'
      : (/^FUNC/i.test(text) ? 'FUNC' : 'AUTO');
    ['vla-arbiter', 'nav2-arbiter'].forEach((id) => {
      const el = $(id);
      if (!el) return;
      el.textContent = stripped || '—';
      el.classList.remove('ok', 'warn');
      el.classList.add(state === 'AUTO' ? 'ok' : 'warn');
    });
    if (state !== navArbState) {
      const first = navArbState === '';   // 首条状态只记录不渲染成"切换"
      navArbState = state;
      if (!first) {
        if (state === 'MANUAL') addVlaStatus('[仲裁] 手动接管，自主导航目标已取消');
        else if (state === 'FUNC') addVlaStatus('[仲裁] ' + (stripped || '功能模块接管，自主导航已让位'));
        else addVlaStatus('[仲裁] 恢复自主导航可用');
      }
    }
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

  // 语音组件卡: 麦克风连接状态边沿提示 + 一键跳到日志面板按 mic 过滤。
  let voiceMicState = null;
  const voiceLogBtn = $('voice-log-btn');
  if (voiceLogBtn) voiceLogBtn.addEventListener('click', () => {
    const f = $('log-filter');
    if (f) {
      f.value = 'mic';
      f.dispatchEvent(new Event('input'));   // 触发既有 rerenderAll
    }
    const logCard = $('log-view');
    if (logCard) logCard.scrollIntoView({ behavior: 'smooth', block: 'center' });
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

  // ---------- 传感器在线监控 (sensor_watchdog) ----------
  // 各传感器模块"绿灯=在线 / 红灯=不在线 / 灰灯=暂无数据"，外加上位机
  // 当前连接的串口设备表（udev 别名 · 占用串口 · USB 设备 ID）。
  let swTime = 0;   // 最近一次 /sensor_watchdog/status 到达时间

  function renderSensorWatchdog(obj) {
    const view = $('sw-sensors');
    if (view) {
      const sensors = (obj && obj.sensors) || [];
      view.innerHTML = sensors.map((s) => {
        const cls = s.online === true ? 'on' : (s.online === false ? 'off' : '');
        const state = s.online === true ? '在线' : (s.online === false ? '不在线' : '暂无数据');
        return `<div class="sw-tile" title="${escapeHTML(s.topic || '')}">` +
          `<span class="sw-dot ${cls}"></span>` +
          `<span class="sw-label">${escapeHTML(s.label || '?')}</span>` +
          `<span class="sw-state ${cls}">${state}</span></div>`;
      }).join('') || '<span class="muted">（watchdog 未配置任何监控项）</span>';
    }
    const dev = $('sw-devices');
    if (dev) {
      const devices = (obj && obj.devices) || [];
      const cnt = $('sw-dev-count');
      if (cnt) cnt.textContent = devices.length + ' 个';
      dev.innerHTML = devices.map((d) =>
        `<div class="sw-row">` +
        `<span class="sw-alias">${escapeHTML(d.alias || '（无别名）')}</span>` +
        `<span class="sw-port">${escapeHTML(d.port || '—')}</span>` +
        `<span class="sw-usbid" title="${escapeHTML(d.usb_id || '')}">${escapeHTML(d.usb_id || '—')}</span></div>`
      ).join('') || '<div class="muted">（未发现串口设备）</div>';
    }
  }

  // watchdog 节点本身断流（未启动/未重编译）时灯全部回灰并提示。
  setInterval(() => {
    if (!swTime || Date.now() - swTime < 5000) return;
    swTime = 0;
    const view = $('sw-sensors');
    if (view) {
      view.querySelectorAll('.sw-dot, .sw-state').forEach((el) => {
        el.classList.remove('on', 'off');
      });
      view.querySelectorAll('.sw-state').forEach((el) => { el.textContent = '未知'; });
      view.insertAdjacentHTML('beforeend',
        '<span class="muted">（sensor_watchdog 数据中断——节点是否在跑？）</span>');
    }
  }, 2000);

  // ---------- 下位机 STM32F407 (turn_on_wheeltec_robot · serial) ----------
  // /odom、/PowerVoltage 等话题只有串口帧校验通过才发布（wheeltec_robot.cpp
  // Get_Sensor_Data 帧头 0x7B 校验）—— 话题数据流动即下位机在线。
  // 固件在电压 <20V（Plus 型）时禁止底盘移动：边沿与持续低压都记录到事件栏。
  let stm32Time = 0;        // 最近一次串口帧话题（/odom、/PowerVoltage）到达时间
  let stm32Online = null;   // null=未知；用于在线/离线边沿记录日志
  let stm32LowVolt = null;  // null=未知；低压状态边沿
  let stm32LowVoltLogTime = 0;
  const STM32_MIN_MOVE_VOLT = 20;   // 与 turn_on_wheeltec_robot 固件阈值一致

  function addStm32Log(text) {
    appendTimeline('stm32-log', 100, text);
  }

  function updateStm32Voltage(v) {
    const volt = $('stm32-voltage');
    if (volt) {
      volt.textContent = v.toFixed(2) + ' V';
      volt.classList.remove('ok', 'warn', 'err');
      volt.classList.add(v < STM32_MIN_MOVE_VOLT ? 'err' : (v < 22 ? 'warn' : 'ok'));
    }
    const low = v < STM32_MIN_MOVE_VOLT;
    const move = $('stm32-move');
    if (move) {
      // 固件只在"低压且未回充"时禁动（robot_en_check: Vol<20 && ChargeMode==0）。
      if (low && stm32RechargeMode === true) {
        move.textContent = '允许（回充中低压豁免）';
        move.classList.remove('ok', 'err');
        move.classList.add('warn');
      } else {
        move.textContent = low ? '禁动 (<' + STM32_MIN_MOVE_VOLT + 'V)' : '允许';
        move.classList.remove('ok', 'err', 'warn');
        move.classList.add(low ? 'err' : 'ok');
      }
    }
    const now = Date.now();
    if (stm32LowVolt !== low) {
      stm32LowVolt = low;
      stm32LowVoltLogTime = now;
      if (low) {
        addStm32Log('⚠ 电压 ' + v.toFixed(2) + 'V 低于 ' + STM32_MIN_MOVE_VOLT +
          'V，底盘禁止移动，请尽快充电');
      } else {
        addStm32Log('电压 ' + v.toFixed(2) + 'V 恢复，底盘允许移动');
      }
    } else if (low && now - stm32LowVoltLogTime > 60000) {
      stm32LowVoltLogTime = now;   // 持续低压每 60s 重复提醒一次，不刷屏
      addStm32Log('⚠ 持续低压 ' + v.toFixed(2) + 'V（<' + STM32_MIN_MOVE_VOLT +
        'V 禁动），请充电');
    }
  }

  // 固件使能位 en_flag（/robot_enable_flag，24字节帧 rx[1]）。失能边沿写事件
  // 日志并列出固件的可能失能原因（RobotControl_task.h errCode 枚举）。
  let stm32Enable = null;   // null=未知（话题没来过，旧驱动没有此话题）
  function updateStm32Enable(en) {
    const el = $('stm32-enable');
    if (el) {
      el.textContent = en ? '使能（可移动）' : '失能（禁止移动）';
      el.classList.remove('ok', 'err');
      el.classList.add(en ? 'ok' : 'err');
    }
    if (stm32Enable !== en) {
      stm32Enable = en;
      if (en) {
        addStm32Log('固件使能恢复，底盘允许移动');
      } else {
        addStm32Log('⚠ 固件已失能禁止移动 —— 可能原因：低压(<20V 且未回充) / ' +
          '急停开关按下 / 软件急停 / 驱动器离线或报错');
      }
    }
  }

  // 下位机回充模式回读（/robot_recharge_mode，回充帧 rx[5]）。
  let stm32RechargeMode = null;
  function updateRechargeMode(on) {
    ['rc-mode', 'stm32-recharge-mode'].forEach((id) => {
      const el = $(id);
      if (!el) return;
      el.textContent = on ? '回充中' : '未开启';
      el.classList.remove('ok', 'warn');
      if (on) el.classList.add('warn');
    });
    if (stm32RechargeMode !== on) {
      stm32RechargeMode = on;
      addStm32Log(on ? '固件已进入自动回充模式（充电桩红外引导控制底盘，低压不禁动）'
        : '固件已退出自动回充模式');
    }
  }

  // 在线/离线检测：2 秒没有任何串口帧话题判离线，1s 检查一次。
  setInterval(() => {
    if (stm32Time === 0) return;   // 从未收到数据，保持 "—"
    const on = Date.now() - stm32Time < 2000;
    const el = $('stm32-online');
    if (el) {
      el.textContent = on ? '在线' : '离线';
      el.classList.remove('ok', 'err');
      el.classList.add(on ? 'ok' : 'err');
    }
    if (stm32Online !== on) {
      stm32Online = on;
      addStm32Log(on ? '下位机在线（串口数据流正常）'
        : '⚠ 下位机数据流中断（串口断开 / 驱动未运行 / rosbridge 断开）');
    }
  }, 1000);

  const stm32LogClear = $('stm32-log-clear');
  if (stm32LogClear) {
    stm32LogClear.addEventListener('click', () => {
      const v = $('stm32-log');
      if (v) v.innerHTML = '';
    });
  }

  // ---------- 自动回充 / 安全等级 / RGB 灯带（底盘控制页，对齐新固件协议） ----------
  // 回充与安全等级都只是驱动里的标志位，要随下一帧 cmd_vel 序列化进串口帧
  // （frame[1]=AutoRecharge / frame[2]=SecurityPLY）才真正到达固件，所以
  // 发布标志位后都补发一帧 cmd_vel 立即生效。
  function setCardMsg(id, text) {
    const el = $(id);
    if (!el) return;
    el.textContent = text ? (text + ' · ' + new Date().toLocaleTimeString()) : '';
  }

  function sendRechargeFlag(v) {
    if (!rechargeFlagPub) { setCardMsg('rc-msg', '未连接 rosbridge'); return; }
    rechargeFlagPub.publish(new ROSLIB.Message({ data: v }));
    publishCmd(0, 0);   // 推一帧零速把标志位带下去（同时确保寻桩从静止开始）
    setCardMsg('rc-msg', v ? '已下发回充指令' : '已退出回充');
    addStm32Log(v ? '> 开始自动回充（/robot_recharge_flag=1，等待固件回读确认）'
      : '> 退出自动回充（/robot_recharge_flag=0）');
  }
  const rcStart = $('rc-start');
  if (rcStart) rcStart.addEventListener('click', () => {
    if (!window.confirm('开始自动回充？小车将由充电桩红外引导自主移动寻桩' +
        '（手动遥控可随时打断）。')) return;
    sendRechargeFlag(1);
  });
  const rcStop = $('rc-stop');
  if (rcStop) rcStop.addEventListener('click', () => sendRechargeFlag(0));

  const secApply = $('sec-apply');
  if (secApply) secApply.addEventListener('click', () => {
    const sel = $('sec-level');
    const v = sel ? parseInt(sel.value, 10) : 0;
    if (!securityPub) { setCardMsg('sec-msg', '未连接 rosbridge'); return; }
    if (v === 1 && !window.confirm('安全等级 1：固件将保持最后一次速度，' +
        '速度流中断（断网/节点崩溃）后小车不会自动停车。确认切换？')) return;
    securityPub.publish(new ROSLIB.Message({ data: v }));
    publishCmd(curVx, curWz);   // 重发当前速度把等级带下去
    setCardMsg('sec-msg', '已应用等级 ' + v);
    addStm32Log('> 底盘安全等级 → ' + v + (v === 0 ? '（速度流中断自动停车）' : '（保持最后速度，注意安全）'));
  });

  // RGB 灯带：颜色选择器 -> /set_rgb_color 服务（驱动转 0x04 串口帧）。
  const rgbColor = $('rgb-color');
  function rgbFromPicker() {
    const hex = (rgbColor && rgbColor.value || '#000000').replace('#', '');
    return {
      r: parseInt(hex.slice(0, 2), 16) || 0,
      g: parseInt(hex.slice(2, 4), 16) || 0,
      b: parseInt(hex.slice(4, 6), 16) || 0,
    };
  }
  function showRgbText() {
    const el = $('rgb-rgbtext');
    if (!el) return;
    const c = rgbFromPicker();
    el.textContent = `R ${c.r} · G ${c.g} · B ${c.b}`;
  }
  if (rgbColor) rgbColor.addEventListener('input', showRgbText);
  showRgbText();

  function callSetRgb(en, c) {
    if (!ros) { setCardMsg('rgb-msg', '未连接 rosbridge'); return; }
    const srv = new ROSLIB.Service({
      ros, name: '/set_rgb_color', serviceType: 'robot_interfaces/srv/SetRgb',
    });
    srv.callService(new ROSLIB.ServiceRequest({ en: !!en, r: c.r, g: c.g, b: c.b }), (res) => {
      const ok = res && /success/i.test(res.res || '');
      setCardMsg('rgb-msg', (ok ? '✓ ' : '✗ ') + ((res && res.res) || '无响应'));
    }, (err) => {
      setCardMsg('rgb-msg', '✗ 调用失败（驱动是否已重编译启用 set_rgb_color？）');
      console.error('set_rgb_color failed', err);
    });
  }
  const rgbApply = $('rgb-apply');
  if (rgbApply) rgbApply.addEventListener('click', () => callSetRgb(true, rgbFromPicker()));
  const rgbOff = $('rgb-off');
  if (rgbOff) rgbOff.addEventListener('click', () => callSetRgb(false, { r: 0, g: 0, b: 0 }));

  // ---------- 下发串口帧解析（/robot_serial_tx，按通信协议表） ----------
  // 11 字节控制帧: 7B [模式选择位] [预留/安全级] [Vx高 Vx低 Vy高 Vy低 Vz高 Vz低] BCC 7D
  // 模式选择位: 0=速度控制(关闭自动回充) 1/2=自动回充 3=红外对接速度 4=灯带RGB。
  // "最近下发指令"实时刷新；命令类型(签名)变化才写事件栏，速度数值变化不刷屏。
  let stm32TxLastSig = null;

  function parseStm32Tx(bytes) {
    if (!bytes || bytes.length < 11 || bytes[0] !== 0x7B || bytes[10] !== 0x7D) return null;
    let bcc = 0;
    for (let i = 0; i < 9; i++) bcc ^= bytes[i];
    const bccOk = bcc === bytes[9];
    const s16 = (h, l) => { let v = ((h & 0xff) << 8) | (l & 0xff); if (v > 32767) v -= 65536; return v; };
    const mode = bytes[1];
    let text, sig;
    if (mode === 4) {
      text = bytes[2]
        ? `设置灯带颜色 R${bytes[3]} G${bytes[4]} B${bytes[5]}`
        : '灯带恢复默认模式';
      sig = `rgb|${bytes[2]}|${bytes[3]},${bytes[4]},${bytes[5]}`;
    } else {
      const vx = s16(bytes[3], bytes[4]) / 1000;
      const vy = s16(bytes[5], bytes[6]) / 1000;
      const vz = s16(bytes[7], bytes[8]) / 1000;
      const vel = `Vx=${vx.toFixed(2)} Vy=${vy.toFixed(2)} Vz=${vz.toFixed(2)}`;
      if (mode === 3) {
        text = `红外对接速度 ${vel}`;
        sig = 'red_vel';
      } else if (mode === 1 || mode === 2) {
        text = `自动回充模式 ${vel}`;
        sig = 'recharge';
      } else {
        text = `速度控制 ${vel}` + (bytes[2] === 1 ? '（安全级1·保持速度）' : '');
        sig = `vel|sec${bytes[2]}`;
      }
    }
    if (!bccOk) { text += ' [BCC 校验错]'; sig += '|badbcc'; }
    return { text, sig, bccOk };
  }

  function ingestStm32Tx(bytes) {
    const p = parseStm32Tx(bytes);
    const el = $('stm32-lastcmd');
    if (!p) {
      if (el) el.textContent = '无法解析（帧头/帧尾不符）';
      return;
    }
    if (el) {
      el.textContent = p.text;
      el.classList.remove('ok', 'err');
      el.classList.add(p.bccOk ? 'ok' : 'err');
    }
    if (p.sig !== stm32TxLastSig) {
      stm32TxLastSig = p.sig;
      const hex = Array.from(bytes.slice(0, 11), (b) => (b & 0xff).toString(16).padStart(2, '0').toUpperCase()).join(' ');
      addStm32Log('↓ 下发: ' + p.text + ' [' + hex + ']');
    }
  }

  // ---------- 骨架识别 (wheeltec_bodyreader) ----------
  // /body_posture: 锁定状态/质心(mm)/姿态标志; /bodylist: 人数; /mode: 1交互 2跟随。
  let bodyPostureTime = 0;

  function renderBodyPosture(msg) {
    bodyPostureTime = Date.now();
    const lock = +msg.lock_status;
    const lockEl = $('body-lock');
    if (lockEl) {
      lockEl.textContent = lock === 2 ? '已锁定' : (lock === 1 ? '检测到人（未锁定）' : '无人');
      lockEl.classList.remove('ok', 'warn', 'err');
      lockEl.classList.add(lock === 2 ? 'ok' : 'warn');
    }
    const idEl = $('body-id');
    if (idEl) idEl.textContent = lock === 2 ? String(msg.bodyid) : '—';
    const z = +msg.centerofmass_z;   // mm，深度方向
    const x = +msg.centerofmass_x;
    const distEl = $('body-dist');
    if (distEl) distEl.textContent = z > 0 ? (z / 1000).toFixed(2) + ' m' : '— m';
    const angEl = $('body-angle');
    if (angEl) angEl.textContent = z > 0 ? (Math.atan2(x, z) * 180 / Math.PI).toFixed(1) + ' °' : '— °';
    const gestures = [];
    if (msg.akimibo) gestures.push('叉腰(锁定)');
    if (msg.left_hand_raised) gestures.push('举左手');
    if (msg.right_hand_raised) gestures.push('举右手');
    if (msg.left_arm_out) gestures.push('平举左臂');
    if (msg.right_arm_out) gestures.push('平举右臂');
    if (msg.left_foot_up) gestures.push('抬左脚');
    if (msg.right_foot_up) gestures.push('抬右脚');
    const gEl = $('body-gesture');
    if (gEl) gEl.textContent = gestures.length ? gestures.join('、') : '—';
    const fEl = $('body-fall');
    if (fEl) {
      fEl.textContent = msg.fall ? '⚠ 跌倒' : '正常';
      fEl.classList.remove('ok', 'err');
      fEl.classList.add(msg.fall ? 'err' : 'ok');
    }
  }

  // 节点停止 3 秒后清回 "—"，避免陈旧姿态误导。
  setInterval(() => {
    if (bodyPostureTime && Date.now() - bodyPostureTime > 3000) {
      bodyPostureTime = 0;
      ['body-lock', 'body-id', 'body-gesture', 'body-fall'].forEach((id) => {
        const el = $(id);
        if (el) { el.textContent = '—'; el.classList.remove('ok', 'warn', 'err'); }
      });
      const d = $('body-dist'); if (d) d.textContent = '— m';
      const a = $('body-angle'); if (a) a.textContent = '— °';
    }
  }, 1000);

  function sendBodyMode(m) {
    if (!bodyModePub) { alert('未连接 rosbridge'); return; }
    bodyModePub.publish(new ROSLIB.Message({ data: m }));
  }
  const bodyModeFollow = $('body-mode-follow');
  if (bodyModeFollow) bodyModeFollow.addEventListener('click', () => {
    if (!window.confirm('切换到跟随模式？锁定目标后机器人会直接发 /cmd_vel 跟人移动。')) return;
    sendBodyMode(2);
  });
  const bodyModeInteract = $('body-mode-interact');
  if (bodyModeInteract) bodyModeInteract.addEventListener('click', () => sendBodyMode(1));
  const bodyRecoverSend = $('body-recover-send');
  if (bodyRecoverSend) bodyRecoverSend.addEventListener('click', () => {
    if (!bodyRecoveryPub) { alert('未连接 rosbridge'); return; }
    const v = parseInt(($('body-recover-id') || {}).value, 10);
    if (Number.isNaN(v)) { alert('请输入数字 ID'); return; }
    bodyRecoveryPub.publish(new ROSLIB.Message({ data: v }));
  });

  // ---------- VLA 地图航点管理 ----------
  // 在已建好的地图(OccupancyGrid)上选点保存命名航点：后端 vla_navigator 收到
  // /vla/waypoint_cmd 后立即生效并持久化到 ~/.ros/vla_waypoints.yaml（重启
  // 优先加载）。地图来源：/map 话题（SLAM 建图中可直接收到）+ GetMap 服务
  // 兜底（map_server 的 transient_local 帧 rosbridge 可能收不到）。
  let wpPose = null;        // { x, y, yaw } 小车当前位姿（map 下）
  let wpSel = null;         // { x, y, yaw } 地图上选中的目标位姿
  let wpMap = null;         // { w, h, res, ox, oy, bmp } 已渲染地图位图
  let wpMapTime = 0;        // 最近一次地图数据更新时间（建图页新鲜度显示用）
  let wpMapTried = false;   // 本次连接是否已自动尝试 GetMap
  let vlaWaypoints = [];    // 后端广播的航点列表
  let wpFile = '';          // 后端持久化文件路径（显示用）
  let wpZoom = 1;           // 画布缩放（相对"整图适配宽度"的倍数，1=整图）
  let wpPanX = 0;           // 画布像素平移（wpRedraw 里钳制，1 倍时锁回 0）
  let wpPanY = 0;

  function setWpMapStatus(text) {
    const el = $('wp-map-status');
    if (el) el.textContent = text || '';
  }

  // OccupancyGrid.data: int8[]，rosbridge 可能给普通数组或 base64 串，都处理。
  function decodeGridData(data) {
    if (typeof data === 'string') {
      const bin = atob(data);
      const out = new Int8Array(bin.length);
      for (let i = 0; i < bin.length; i++) {
        const b = bin.charCodeAt(i) & 0xff;
        out[i] = b > 127 ? b - 256 : b;
      }
      return out;
    }
    return data || [];
  }

  function ingestWpMap(grid) {
    if (!grid || !grid.info || !grid.info.width || !grid.info.height) return;
    const w = grid.info.width, h = grid.info.height;
    const data = decodeGridData(grid.data);
    const bmp = document.createElement('canvas');
    bmp.width = w; bmp.height = h;
    const ctx = bmp.getContext('2d');
    const img = ctx.createImageData(w, h);
    for (let y = 0; y < h; y++) {
      const srcRow = y * w;
      const dstRow = (h - 1 - y) * w;   // 预翻转：map 的 y 向上，canvas 向下
      for (let x = 0; x < w; x++) {
        const v = data[srcRow + x];
        let r, g, b;
        if (v < 0) { r = 0x2d; g = 0x38; b = 0x45; }        // 未知
        else if (v < 50) { r = 0xe6; g = 0xed; b = 0xf3; }  // 空闲
        else { r = 0x0d; g = 0x11; b = 0x17; }              // 占用
        const o = (dstRow + x) * 4;
        img.data[o] = r; img.data[o + 1] = g; img.data[o + 2] = b; img.data[o + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
    wpMap = {
      w, h,
      res: grid.info.resolution,
      ox: grid.info.origin ? grid.info.origin.position.x : 0,
      oy: grid.info.origin ? grid.info.origin.position.y : 0,
      bmp,
    };
    wpMapTime = Date.now();
    setWpMapStatus(`${w}×${h} @ ${(+grid.info.resolution).toFixed(3)} m/格`);
    wpRedraw();
  }

  function fetchWpMap() {
    if (!ros) { setWpMapStatus('未连接 rosbridge'); return; }
    const srvName = (($('wp-map-srv') || {}).value || '/map_server/map').trim();
    setWpMapStatus('加载地图中…（大地图可能要几秒）');
    const srv = new ROSLIB.Service({ ros, name: srvName, serviceType: 'nav_msgs/srv/GetMap' });
    srv.callService(new ROSLIB.ServiceRequest({}), (res) => {
      if (res && res.map && res.map.info && res.map.info.width) ingestWpMap(res.map);
      else setWpMapStatus('地图服务返回空地图');
    }, (err) => {
      // GetMap 失败（map_server 没跑）→ 自动兜底：从包内地图文件直接读
      if ((($('wp-map-file') || {}).value || '').trim()) {
        setWpMapStatus('GetMap 失败（' + err + '），改从包内地图文件加载…');
        fetchWpMapFromFile();
      } else {
        setWpMapStatus('加载失败：' + err + '（map_server 在跑吗？SLAM 建图中可等 /map 话题）');
      }
    });
  }
  const wpMapLoad = $('wp-map-load');
  if (wpMapLoad) wpMapLoad.addEventListener('click', fetchWpMap);

  // ---- 包内地图文件直读（map_server YAML+PGM 格式）----
  // 经面板 HTTP 的 /pkg/<包名>/… 路由取 wheeltec_nav2 等包 share 里的地图，
  // 浏览器自己解析 —— 不依赖 map_server/GetMap，WSL 无硬件、未启动导航时
  // 也能看图/标航点。GetMap 失败时自动兜底到这里。
  function parseMapYaml(text) {
    const meta = { negate: 0, occupied_thresh: 0.65, free_thresh: 0.196, origin: [0, 0, 0] };
    text.split(/\r?\n/).forEach((line) => {
      const m = /^\s*([A-Za-z_]+)\s*:\s*(.+?)\s*$/.exec(line);
      if (!m) return;
      const key = m[1];
      const val = m[2];
      if (key === 'image') meta.image = val.replace(/^['"]|['"]$/g, '');
      else if (key === 'resolution') meta.resolution = parseFloat(val);
      else if (key === 'negate') meta.negate = parseInt(val, 10) || 0;
      else if (key === 'occupied_thresh') meta.occupied_thresh = parseFloat(val);
      else if (key === 'free_thresh') meta.free_thresh = parseFloat(val);
      else if (key === 'origin') {
        const nums = val.replace(/[[\]]/g, '').split(',').map(Number);
        if (nums.length >= 2) meta.origin = nums;
      }
    });
    return meta;
  }

  // P5(二进制)/P2(文本) PGM → OccupancyGrid 同构对象。行序要翻转：PGM 第 0
  // 行是图片顶部，OccupancyGrid 第 0 行是地图底部（origin 在左下角）。
  function pgmToGrid(bytes, meta) {
    let pos = 0;
    const isSpace = (c) => c === 32 || c === 9 || c === 10 || c === 13;
    function token() {
      for (;;) {
        while (pos < bytes.length && isSpace(bytes[pos])) pos++;
        if (bytes[pos] === 35) {   // '#' 注释到行尾
          while (pos < bytes.length && bytes[pos] !== 10) pos++;
        } else break;
      }
      let s = '';
      while (pos < bytes.length && !isSpace(bytes[pos])) s += String.fromCharCode(bytes[pos++]);
      return s;
    }
    const magic = token();
    if (magic !== 'P5' && magic !== 'P2') throw new Error('不是 PGM(P5/P2) 文件: ' + magic);
    const w = parseInt(token(), 10);
    const h = parseInt(token(), 10);
    const maxval = parseInt(token(), 10) || 255;
    if (!w || !h) throw new Error('PGM 尺寸解析失败');
    if (magic === 'P5' && maxval > 255) throw new Error('不支持 16bit PGM');
    const px = new Array(w * h);
    if (magic === 'P5') {
      pos++;   // maxval 后恰好一个空白字节，再往后是裸像素
      for (let i = 0; i < w * h; i++) px[i] = bytes[pos + i];
    } else {
      for (let i = 0; i < w * h; i++) px[i] = parseInt(token(), 10);
    }
    // 按 map_server trinary 规则转占用值：p=(max-v)/max（negate=1 时 v/max），
    // p>occupied→100、p<free→0、其余 -1（未知）
    const data = new Int8Array(w * h);
    for (let y = 0; y < h; y++) {
      const src = y * w;
      const dst = (h - 1 - y) * w;
      for (let x = 0; x < w; x++) {
        const v = px[src + x];
        const p = meta.negate ? v / maxval : (maxval - v) / maxval;
        data[dst + x] = p > meta.occupied_thresh ? 100 : (p < meta.free_thresh ? 0 : -1);
      }
    }
    return {
      info: {
        width: w,
        height: h,
        resolution: meta.resolution,
        origin: { position: { x: meta.origin[0] || 0, y: meta.origin[1] || 0 } },
      },
      data,
    };
  }

  function fetchWpMapFromFile() {
    const path = ((($('wp-map-file') || {}).value) || '').trim();
    if (!path) { setWpMapStatus('未填包内地图文件路径'); return; }
    setWpMapStatus('从包内地图文件加载…');
    fetch(path).then((resp) => {
      if (!resp.ok) throw new Error('HTTP ' + resp.status + '：' + path);
      return resp.text();
    }).then((ytext) => {
      const meta = parseMapYaml(ytext);
      if (!meta.image || !meta.resolution) throw new Error('YAML 缺 image/resolution 字段');
      const imgUrl = /^(\/|https?:)/.test(meta.image)
        ? meta.image
        : path.replace(/[^/]*$/, '') + meta.image;   // image 相对 yaml 所在目录
      return fetch(imgUrl).then((resp) => {
        if (!resp.ok) throw new Error('HTTP ' + resp.status + '：' + imgUrl);
        return resp.arrayBuffer();
      }).then((buf) => {
        ingestWpMap(pgmToGrid(new Uint8Array(buf), meta));
      });
    }).catch((err) => {
      setWpMapStatus('包文件加载失败：' + ((err && err.message) || err) +
        '（wheeltec_dashboard 重编译启用 /pkg/ 路由了吗？包名/路径对吗？）');
    });
  }
  const wpMapFileLoad = $('wp-map-file-load');
  if (wpMapFileLoad) wpMapFileLoad.addEventListener('click', fetchWpMapFromFile);

  // map 世界坐标 ↔ 显示画布坐标（位图已预翻转，y 轴只需一次镜像换算；
  // view.s 已含缩放倍数，tx/ty 是平移）
  function wpWorldToCanvas(wx, wy, view) {
    return {
      x: (wx - wpMap.ox) / wpMap.res * view.s + view.tx,
      y: view.h - (wy - wpMap.oy) / wpMap.res * view.s + view.ty,
    };
  }
  function wpCanvasToWorld(cx, cy, view) {
    return {
      x: (cx - view.tx) / view.s * wpMap.res + wpMap.ox,
      y: (view.h + view.ty - cy) / view.s * wpMap.res + wpMap.oy,
    };
  }

  // 平移钳制：画布始终被地图盖住不留空边（140px 最小高度下地图比画布
  // 矮的罕见情况取反向区间，效果是允许贴边）。1 倍缩放时区间收敛为 0。
  function wpClampPan(view) {
    const mapW = wpMap.w * view.s, mapH = wpMap.h * view.s;
    wpPanX = Math.max(Math.min(0, view.w - mapW), Math.min(Math.max(0, view.w - mapW), wpPanX));
    wpPanY = Math.max(Math.min(0, mapH - view.h), Math.min(Math.max(0, mapH - view.h), wpPanY));
    view.tx = wpPanX;
    view.ty = wpPanY;
  }

  function drawWpArrow(ctx, p, worldYaw, color, r) {
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-worldYaw);   // 画布 y 翻转 → 旋向取负
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(r, 0);
    ctx.lineTo(-r * 0.6, r * 0.55);
    ctx.lineTo(-r * 0.6, -r * 0.55);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function wpRedraw() {
    const canvas = $('wp-map');
    if (!canvas || !canvas.offsetParent || !wpMap) return;
    const wrap = canvas.parentElement;
    const dispW = Math.max(240, Math.min(wrap.clientWidth || 640, 900));
    const s = dispW / wpMap.w;
    const dispH = Math.max(140, Math.round(wpMap.h * s));
    if (canvas.width !== dispW || canvas.height !== dispH) {
      canvas.width = dispW;
      canvas.height = dispH;
    }
    const view = { s: s * wpZoom, w: dispW, h: dispH, tx: 0, ty: 0 };
    wpClampPan(view);
    canvas._view = view;   // pointer 事件换算用
    const ctx = canvas.getContext('2d');
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, dispW, dispH);
    // 位图左上角 = map 帧 (ox, oy + h·res)，按 view 变换摆放——与
    // wpWorldToCanvas 同一套公式，标记和底图永远对得上
    ctx.drawImage(wpMap.bmp, view.tx, view.h - wpMap.h * view.s + view.ty,
      wpMap.w * view.s, wpMap.h * view.s);
    // 已保存航点（蓝点 + 名字）
    ctx.font = '11px sans-serif';
    vlaWaypoints.forEach((wp) => {
      const p = wpWorldToCanvas(+wp.x, +wp.y, view);
      ctx.fillStyle = '#58a6ff';
      ctx.beginPath();
      ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillText(wp.name, p.x + 6, p.y - 4);
    });
    // 小车实时位姿（绿色箭头）
    const tf = wpTf ? wpTf.lookup('base_footprint') : null;
    if (tf) {
      drawWpArrow(ctx, wpWorldToCanvas(tf.translation.x, tf.translation.y, view),
        yawFromQuat(tf.rotation), '#3fb950', 9);
    }
    // 当前选点（黄色箭头）
    if (wpSel) {
      drawWpArrow(ctx, wpWorldToCanvas(wpSel.x, wpSel.y, view), wpSel.yaw, '#d29922', 9);
    }
  }

  function updateWpSelText() {
    const el = $('wp-sel');
    if (!el) return;
    if (!wpSel) { el.textContent = '—'; return; }
    el.textContent = `x=${wpSel.x.toFixed(3)}  y=${wpSel.y.toFixed(3)}  yaw=${wpSel.yaw.toFixed(3)} rad (${(wpSel.yaw * 180 / Math.PI).toFixed(1)}°)`;
  }

  // 地图取点 + 视图操控：
  //   左键/单指     按下定位置、按住拖动定朝向（同 RViz 2D Goal Pose）
  //   滚轮/双指捏合  以光标（两指中点）为锚缩放，1~12 倍
  //   中键/右键拖动  平移（画布上的右键菜单已屏蔽）
  //   视图复位按钮   回到整图
  (function initWpMapPick() {
    const canvas = $('wp-map');
    if (!canvas) return;
    let downWorld = null;
    let panLast = null;          // 中/右键平移：上一点画布坐标
    const touches = new Map();   // 活动触点 id -> 画布坐标（双指捏合用）
    let pinchPrev = null;        // 上一帧 { dist, mx, my }

    function evCanvas(e) {
      const r = canvas.getBoundingClientRect();
      if (!r.width || !r.height) return null;
      return {
        x: (e.clientX - r.left) * (canvas.width / r.width),
        y: (e.clientY - r.top) * (canvas.height / r.height),
      };
    }
    function evWorld(e) {
      const view = canvas._view;
      if (!view || !wpMap) return null;
      const c = evCanvas(e);
      return c ? wpCanvasToWorld(c.x, c.y, view) : null;
    }

    const ZOOM_MIN = 1, ZOOM_MAX = 12;
    function wpZoomLabel() {
      const el = $('wp-zoom');
      if (el) el.textContent = wpZoom > 1.001 ? '×' + wpZoom.toFixed(1) : '';
    }
    // 以画布点 (cx,cy) 为锚缩放：锚点对应的世界坐标在缩放前后不动——
    // 先记锚点世界坐标，换缩放后反解平移（wpRedraw 里再钳制）。
    function zoomAt(cx, cy, factor) {
      const view = canvas._view;
      if (!view || !wpMap) return;
      const z = Math.max(ZOOM_MIN, Math.min(ZOOM_MAX, wpZoom * factor));
      const w = wpCanvasToWorld(cx, cy, view);
      const s1 = (view.s / wpZoom) * z;
      wpZoom = z;
      wpPanX = cx - (w.x - wpMap.ox) / wpMap.res * s1;
      wpPanY = cy - view.h + (w.y - wpMap.oy) / wpMap.res * s1;
      wpRedraw();
      wpZoomLabel();
    }

    canvas.addEventListener('wheel', (e) => {
      if (!wpMap || !canvas._view) return;
      e.preventDefault();
      const c = evCanvas(e);
      if (c) zoomAt(c.x, c.y, Math.pow(1.0015, -e.deltaY));
    }, { passive: false });

    canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    const wpViewReset = $('wp-view-reset');
    if (wpViewReset) wpViewReset.addEventListener('click', () => {
      wpZoom = 1; wpPanX = 0; wpPanY = 0;
      wpRedraw();
      wpZoomLabel();
    });

    canvas.addEventListener('pointerdown', (e) => {
      if (e.pointerType === 'mouse' && (e.button === 1 || e.button === 2)) {
        panLast = evCanvas(e);
        try { canvas.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
        e.preventDefault();
        return;
      }
      if (e.pointerType === 'mouse' && e.button !== 0) return;
      if (e.pointerType === 'touch') {
        const c = evCanvas(e);
        if (c) touches.set(e.pointerId, c);
        if (touches.size === 2) {
          // 第二指落下 → 切捏合模式，放弃进行中的选点（位置已选上，
          // 不再跟着拖朝向）
          downWorld = null;
          pinchPrev = null;
          try { canvas.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
          e.preventDefault();
          return;
        }
      }
      const w = evWorld(e);
      if (!w) return;
      downWorld = w;
      wpSel = { x: w.x, y: w.y, yaw: wpSel ? wpSel.yaw : 0 };
      updateWpSelText();
      wpRedraw();
      try { canvas.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
      e.preventDefault();
    });
    canvas.addEventListener('pointermove', (e) => {
      if (panLast) {
        const c = evCanvas(e);
        if (!c) return;
        wpPanX += c.x - panLast.x;
        wpPanY += c.y - panLast.y;
        panLast = c;
        wpRedraw();
        return;
      }
      if (e.pointerType === 'touch' && touches.size === 2) {
        const c = evCanvas(e);
        if (!c || !touches.has(e.pointerId)) return;
        touches.set(e.pointerId, c);
        const pts = Array.from(touches.values());
        const cur = {
          dist: Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y),
          mx: (pts[0].x + pts[1].x) / 2,
          my: (pts[0].y + pts[1].y) / 2,
        };
        if (pinchPrev && pinchPrev.dist > 0) {
          wpPanX += cur.mx - pinchPrev.mx;
          wpPanY += cur.my - pinchPrev.my;
          zoomAt(cur.mx, cur.my, cur.dist / pinchPrev.dist);   // 内部钳制+重绘
        }
        pinchPrev = cur;
        return;
      }
      if (!downWorld) return;
      const w = evWorld(e);
      if (!w) return;
      const dx = w.x - downWorld.x, dy = w.y - downWorld.y;
      // 拖出 3 个栅格以上才算在定朝向，避免手抖把 yaw 打飞
      if (Math.hypot(dx, dy) > wpMap.res * 3) wpSel.yaw = Math.atan2(dy, dx);
      updateWpSelText();
      wpRedraw();
    });
    const finish = (e) => {
      downWorld = null;
      panLast = null;
      if (e && e.pointerType === 'touch') {
        touches.delete(e.pointerId);
        if (touches.size < 2) pinchPrev = null;
      }
    };
    canvas.addEventListener('pointerup', finish);
    canvas.addEventListener('pointercancel', finish);
  })();

  // 周期刷新：当前位姿 + 地图重绘 + 首次自动拉地图（仅子页可见时工作）。
  setInterval(() => {
    const el = $('wp-cur');
    if (!el || !el.offsetParent) return;   // 子页不可见时不刷新
    const tf = wpTf ? wpTf.lookup('base_footprint') : null;
    if (!tf) {
      wpPose = null;
      el.textContent = ros ? '未定位（map TF 不可用，需启动 Nav2/AMCL）' : '未连接';
      el.classList.remove('ok');
    } else {
      const yaw = yawFromQuat(tf.rotation);
      wpPose = { x: tf.translation.x, y: tf.translation.y, yaw };
      el.textContent = `x=${wpPose.x.toFixed(3)}  y=${wpPose.y.toFixed(3)}  yaw=${yaw.toFixed(3)} rad (${(yaw * 180 / Math.PI).toFixed(1)}°)`;
      el.classList.add('ok');
    }
    if (!wpMap && ros && !wpMapTried) { wpMapTried = true; fetchWpMap(); }
    wpRedraw();
  }, 500);

  function wpAliases() {
    return (($('wp-alias') || {}).value || '')
      .split(/[,，]/).map((s) => s.trim()).filter(Boolean);
  }

  function sendWpCmd(obj) {
    if (!wpCmdPub) { setCardMsg('wp-msg', '未连接 rosbridge'); return false; }
    wpCmdPub.publish(new ROSLIB.Message({ data: JSON.stringify(obj) }));
    return true;
  }

  const wpSave = $('wp-save');
  if (wpSave) wpSave.addEventListener('click', () => {
    const name = (($('wp-name') || {}).value || '').trim();
    if (!name) { alert('请先填航点名（如：厨房）'); return; }
    const pose = wpSel || wpPose;
    if (!pose) { alert('请先在地图上选点，或等定位收敛后用当前位姿'); return; }
    const ok = sendWpCmd({
      action: 'add',
      name,
      aliases: wpAliases(),
      x: +pose.x.toFixed(3),
      y: +pose.y.toFixed(3),
      yaw: +pose.yaw.toFixed(3),
    });
    if (ok) setCardMsg('wp-msg', '已发送保存 "' + name + '"（等列表刷新确认）');
  });

  const wpUseCur = $('wp-use-cur');
  if (wpUseCur) wpUseCur.addEventListener('click', () => {
    if (!wpPose) { alert('当前没有有效定位（map→base_footprint TF 不可用）'); return; }
    wpSel = { x: wpPose.x, y: wpPose.y, yaw: wpPose.yaw };
    updateWpSelText();
    wpRedraw();
  });

  // 直接把地图上"选中点"作为 Nav2 目标发出（无需先存航点）——
  // 等价 RViz 的 2D Nav Goal：地图上按下选点、拖动定朝向，再点此按钮发 /goal_pose。
  const wpGotoSel = $('wp-goto-sel');
  if (wpGotoSel) wpGotoSel.addEventListener('click', () => {
    if (!goalPosePub) { setCardMsg('wp-msg', '未连接 rosbridge'); return; }
    const pose = wpSel || wpPose;
    if (!pose) { alert('请先在地图上选点（按下选位置、拖动定朝向），或用当前位姿'); return; }
    if (!window.confirm(`导航到选中点 x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}？小车将开始移动。`)) return;
    goalPosePub.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      pose: {
        position: { x: +pose.x, y: +pose.y, z: 0 },
        orientation: { x: 0, y: 0, z: Math.sin(pose.yaw / 2), w: Math.cos(pose.yaw / 2) },
      },
    }));
    setCardMsg('wp-msg', `已发布 /goal_pose（选中点 x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)}）`);
    toast(`导航目标已发布 (x=${pose.x.toFixed(2)}, y=${pose.y.toFixed(2)})，小车开始移动`, 'ok');
  });

  // 航点列表：来自后端广播，带"导航 / 删除"操作（事件委托，重渲染不丢绑定）。
  function renderWpList() {
    const view = $('wp-list');
    if (!view) return;
    const cnt = $('wp-count');
    if (cnt) cnt.textContent = vlaWaypoints.length + ' 个' + (wpFile ? ' · ' + wpFile : '');
    if (!vlaWaypoints.length) {
      view.innerHTML = '<div class="muted">（后端暂无航点，或 vla_navigator 未启动 / 未重编译）</div>';
      return;
    }
    view.innerHTML = vlaWaypoints.map((wp) => {
      const alias = (wp.aliases && wp.aliases.length)
        ? ` <span class="muted">(${escapeHTML(wp.aliases.join('/'))})</span>` : '';
      return `<div class="wp-row" data-name="${escapeHTML(wp.name)}">` +
        `<span class="wp-row-name">${escapeHTML(wp.name)}${alias}</span>` +
        `<span class="wp-row-pos">x=${(+wp.x).toFixed(2)} y=${(+wp.y).toFixed(2)} yaw=${(+(wp.yaw || 0)).toFixed(2)}</span>` +
        `<button type="button" data-act="goto">导航</button>` +
        `<button type="button" data-act="del" class="wp-del">删除</button></div>`;
    }).join('');
  }

  const wpList = $('wp-list');
  if (wpList) wpList.addEventListener('click', (e) => {
    const btn = e.target.closest('button[data-act]');
    if (!btn) return;
    const row = btn.closest('.wp-row');
    const name = row && row.dataset.name;
    const wp = vlaWaypoints.find((w) => w.name === name);
    if (!wp) return;
    if (btn.dataset.act === 'del') {
      if (!window.confirm('删除航点 "' + name + '"？（会同步写入持久化文件）')) return;
      if (sendWpCmd({ action: 'remove', name })) {
        setCardMsg('wp-msg', '已发送删除 "' + name + '"');
      }
    } else if (btn.dataset.act === 'goto') {
      if (!goalPosePub) { setCardMsg('wp-msg', '未连接 rosbridge'); return; }
      if (!window.confirm('导航到 "' + name + '" (x=' + (+wp.x).toFixed(2) +
          ', y=' + (+wp.y).toFixed(2) + ')？小车将开始移动。')) return;
      const yaw = +(wp.yaw || 0);
      goalPosePub.publish(new ROSLIB.Message({
        header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x: +wp.x, y: +wp.y, z: 0 },
          orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
        },
      }));
      setCardMsg('wp-msg', '已发布 /goal_pose → ' + name);
      toast('导航目标已发布 → ' + name + '，小车开始移动', 'ok');
    }
  });

  // 备用手动流程：生成可追加进 config/waypoints.yaml 的片段（优先用地图选点）。
  const wpGen = $('wp-gen');
  if (wpGen) wpGen.addEventListener('click', () => {
    const out = $('wp-yaml');
    if (!out) return;
    const pose = wpSel || wpPose;
    if (!pose) { alert('请先在地图上选点，或等定位收敛'); return; }
    const name = (($('wp-name') || {}).value || '').trim();
    if (!name) { alert('请先填航点名（如：厨房）'); return; }
    const aliases = wpAliases();
    const lines = [`  - name: "${name}"`];
    if (aliases.length) lines.push(`    aliases: [${aliases.map((a) => `"${a}"`).join(', ')}]`);
    lines.push(`    x: ${pose.x.toFixed(3)}`);
    lines.push(`    y: ${pose.y.toFixed(3)}`);
    lines.push(`    yaw: ${pose.yaw.toFixed(3)}`);
    out.value = lines.join('\n');
  });
  const wpCopy = $('wp-copy');
  if (wpCopy) wpCopy.addEventListener('click', () => {
    const out = $('wp-yaml');
    if (!out || !out.value) return;
    out.select();
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(out.value).catch(() => document.execCommand('copy'));
    } else {
      document.execCommand('copy');
    }
  });

  // ---------- 路径跟随 (wheeltec_path_follow) ----------
  // save_path（录制）与 follow_path.py（回放）都把当前路径发布到 /followpath
  // (nav_msgs/Path, map 系)：订阅后画在地图上。地图位图(wpMap)与 map 系 TF
  // 客户端(wpTf)同"地图航点管理"卡共用；节点在线状态靠 ros.getNodes 周期
  // 查 /save_path、/follow_path 是否存在（路径话题两个节点同名，分不开）。
  let pfSub = null;        // /followpath 订阅（话题可改，独立管理便于重订）
  let pfPath = [];         // 话题里的实时路径 [{x, y, yaw}]
  let pfPathTime = 0;      // 最近一次路径消息到达时间
  let pfFilePath = [];     // "从包加载预览"解析出的路径
  const pfNodeState = { save: null, follow: null };   // null=未知（首查不写离线日志）

  function addPfLog(text) { appendTimeline('pf-log', 200, text); }

  function setPfStatus(text) {
    const el = $('pf-status');
    if (el) el.textContent = text || '';
  }

  function subscribePathFollow() {
    if (pfSub) {
      try { pfSub.unsubscribe(); } catch (_) { /* ignore */ }
      pfSub = null;
    }
    if (!ros) return;
    const topic = ((($('pf-topic') || {}).value) || '/followpath').trim();
    pfSub = new ROSLIB.Topic({
      ros, name: topic, messageType: 'nav_msgs/msg/Path',
      throttle_rate: 500, queue_length: 1,
    });
    pfSub.subscribe((msg) => {
      const poses = (msg && msg.poses) || [];
      pfPath = poses.map((p) => {
        const pos = (p.pose && p.pose.position) || {};
        const q = (p.pose && p.pose.orientation) || { x: 0, y: 0, z: 0, w: 1 };
        return { x: +pos.x || 0, y: +pos.y || 0, yaw: yawFromQuat(q) };
      });
      pfPathTime = Date.now();
      updatePfMetrics();
      pfRedraw();
    });
  }

  // 当前显示的路径：话题有数据用话题（录制/回放进行中），否则用文件预览。
  function pfActivePath() { return pfPath.length ? pfPath : pfFilePath; }

  function pfLength(pts) {
    let d = 0;
    for (let i = 1; i < pts.length; i++) {
      d += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    }
    return d;
  }

  function updatePfMetrics() {
    const pts = pfActivePath();
    const src = pfPath.length ? '话题' : (pfFilePath.length ? '文件预览' : '');
    const pEl = $('pf-points');
    if (pEl) pEl.textContent = pts.length ? (pts.length + ' 点（' + src + '）') : '—';
    const lEl = $('pf-length');
    if (lEl) lEl.textContent = pts.length > 1 ? pfLength(pts).toFixed(2) + ' m' : '— m';
  }

  // 视图变换：有地图按地图（与航点卡同一公式），没有按路径外包框自适应。
  function pfView(canvas) {
    const wrap = canvas.parentElement;
    const maxW = Math.max(240, Math.min((wrap && wrap.clientWidth) || 640, 900));
    if (wpMap) {
      const s = maxW / wpMap.w;
      const h = Math.max(140, Math.round(wpMap.h * s));
      return {
        w: maxW, h, map: true,
        toCanvas: (wx, wy) => ({
          x: (wx - wpMap.ox) / wpMap.res * s,
          y: h - (wy - wpMap.oy) / wpMap.res * s,
        }),
      };
    }
    const pts = pfPath.concat(pfFilePath);
    const tf = wpTf ? wpTf.lookup('base_footprint') : null;
    if (tf) pts.push({ x: tf.translation.x, y: tf.translation.y });
    if (!pts.length) return null;
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    pts.forEach((p) => {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.y > maxY) maxY = p.y;
    });
    minX -= 0.5; maxX += 0.5; minY -= 0.5; maxY += 0.5;   // 0.5m 边距
    const s = Math.min(maxW / (maxX - minX), 480 / (maxY - minY));
    const w = Math.max(240, Math.round((maxX - minX) * s));
    const h = Math.max(140, Math.round((maxY - minY) * s));
    return {
      w, h, map: false,
      toCanvas: (wx, wy) => ({ x: (wx - minX) * s, y: h - (wy - minY) * s }),
    };
  }

  function drawPfPolyline(ctx, view, pts, color, dashed) {
    if (!pts.length) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.setLineDash(dashed ? [6, 4] : []);
    ctx.beginPath();
    pts.forEach((p, i) => {
      const c = view.toCanvas(p.x, p.y);
      if (i === 0) ctx.moveTo(c.x, c.y);
      else ctx.lineTo(c.x, c.y);
    });
    ctx.stroke();
    ctx.setLineDash([]);
    // 起点绿点 / 终点红点
    const s0 = view.toCanvas(pts[0].x, pts[0].y);
    ctx.fillStyle = '#3fb950';
    ctx.beginPath(); ctx.arc(s0.x, s0.y, 4, 0, Math.PI * 2); ctx.fill();
    if (pts.length > 1) {
      const s1 = view.toCanvas(pts[pts.length - 1].x, pts[pts.length - 1].y);
      ctx.fillStyle = '#f85149';
      ctx.beginPath(); ctx.arc(s1.x, s1.y, 4, 0, Math.PI * 2); ctx.fill();
    }
    ctx.restore();
  }

  function pfRedraw() {
    const canvas = $('pf-map');
    if (!canvas || !canvas.offsetParent) return;   // 子页不可见时不画
    const ctx = canvas.getContext('2d');
    const view = pfView(canvas);
    if (!view) {
      // 既无地图也无路径：占位提示
      const wrap = canvas.parentElement;
      const w = Math.max(240, Math.min((wrap && wrap.clientWidth) || 640, 900));
      if (canvas.width !== w || canvas.height !== 160) { canvas.width = w; canvas.height = 160; }
      ctx.fillStyle = '#161b22';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#8b949e';
      ctx.font = '13px sans-serif';
      ctx.fillText('等待路径数据…（订阅 ' + ((($('pf-topic') || {}).value) || '/followpath') +
        '，或点"从包加载预览"）', 12, 84);
      return;
    }
    if (canvas.width !== view.w || canvas.height !== view.h) {
      canvas.width = view.w;
      canvas.height = view.h;
    }
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, view.w, view.h);
    if (view.map) {
      ctx.drawImage(wpMap.bmp, 0, 0, view.w, view.h);
    } else {
      ctx.fillStyle = '#161b22';
      ctx.fillRect(0, 0, view.w, view.h);
    }
    drawPfPolyline(ctx, view, pfFilePath, '#d29922', true);   // 文件预览：黄虚线
    drawPfPolyline(ctx, view, pfPath, '#58a6ff', false);      // 实时话题：蓝实线
    // 小车实时位姿（绿色箭头，map 系 TF 与航点卡共用）
    const tf = wpTf ? wpTf.lookup('base_footprint') : null;
    if (tf) {
      drawWpArrow(ctx, view.toCanvas(tf.translation.x, tf.translation.y),
        yawFromQuat(tf.rotation), '#3fb950', 9);
    }
  }

  // 节点上线即读其参数显示在面板上（录制/回放各自的 pathfilename 等）。
  function pfReadNodeParams(kind) {
    const node = kind === 'save' ? '/save_path' : '/follow_path';
    const names = kind === 'save' ? ['pathfilename'] : ['pathfilename', 'run_in_loop'];
    paramService(node, 'get_parameters').callService(
      new ROSLIB.ServiceRequest({ names }), (res) => {
        const vals = (res && res.values) || [];
        const file = (vals[0] && vals[0].type === PT_STRING) ? vals[0].string_value : '';
        if (!file) return;
        if (kind === 'save') {
          const el = $('pf-rec-file');
          if (el) el.textContent = file;
        } else {
          const loop = (vals[1] && vals[1].type === PT_BOOL) ? vals[1].bool_value : null;
          const el = $('pf-follow-cfg');
          if (el) el.textContent = file + (loop === null ? '' : (loop ? ' · 循环' : ' · 单次'));
        }
      }, () => { /* 节点刚退出等竞态，忽略 */ });
  }

  function setPfNodeMetric(id, on) {
    const el = $(id);
    if (!el) return;
    el.textContent = on === null ? '—' : (on ? '在线' : '离线');
    el.classList.remove('ok', 'warn', 'err');
    if (on) el.classList.add('ok');
  }

  // 节点在线轮询（仅子页可见时）：边沿写事件日志 + 上线时读参数。
  setInterval(() => {
    const panel = $('subpanel-path');
    if (!panel || !panel.offsetParent) return;
    if (!ros) {
      pfNodeState.save = null;
      pfNodeState.follow = null;
      setPfNodeMetric('pf-save-node', null);
      setPfNodeMetric('pf-follow-node', null);
      return;
    }
    ros.getNodes((nodes) => {
      const list = nodes || [];
      const now = {
        save: list.indexOf('/save_path') !== -1,
        follow: list.indexOf('/follow_path') !== -1,
      };
      ['save', 'follow'].forEach((k) => {
        if (pfNodeState[k] !== now[k]) {
          // 首查 null→离线 不写日志，避免每次连接都刷一条"离线"
          if (pfNodeState[k] !== null || now[k]) {
            const label = k === 'save' ? '录制节点 save_path' : '回放节点 follow_path';
            addPfLog(label + (now[k] ? ' 已上线'
              : (' 已退出' + (k === 'save' ? '（save_path 退出时才写盘保存路径文件）' : ''))));
          }
          if (now[k]) pfReadNodeParams(k);
          pfNodeState[k] = now[k];
        }
        setPfNodeMetric(k === 'save' ? 'pf-save-node' : 'pf-follow-node', now[k]);
      });
    }, () => { /* getNodes 偶发失败忽略，下个周期重试 */ });
  }, 3000);

  // 周期刷新（仅子页可见时）：路径数据新鲜度 + 重绘（小车箭头在动）。
  setInterval(() => {
    const canvas = $('pf-map');
    if (!canvas || !canvas.offsetParent) return;
    const fEl = $('pf-fresh');
    if (fEl) {
      if (!pfPathTime) {
        fEl.textContent = ros ? '—（无数据，录制/回放节点未发布）' : '未连接';
        fEl.classList.remove('ok');
      } else {
        const age = (Date.now() - pfPathTime) / 1000;
        fEl.classList.toggle('ok', age < 3);
        fEl.textContent = age < 3 ? '实时更新中' : Math.round(age) + ' 秒前';
      }
    }
    pfRedraw();
  }, 1000);

  // 包内路径文件预览：经 /pkg/ 路由直读 share 里的路径文本（每行 "x y yaw"，
  // EOP 结尾），浏览器解析后画黄虚线 —— 不启动任何节点也能看已录路径。
  function pfLoadFile() {
    const path = ((($('pf-file') || {}).value) || '').trim();
    if (!path) { setPfStatus('未填路径文件'); return; }
    setPfStatus('加载路径文件…');
    fetch(path).then((resp) => {
      if (!resp.ok) throw new Error('HTTP ' + resp.status + '：' + path);
      return resp.text();
    }).then((text) => {
      const pts = [];
      for (const line of text.split(/\r?\n/)) {
        if (line.trim() === 'EOP') break;
        const tok = line.trim().split(/\s+/);
        if (tok.length !== 3) continue;
        const x = parseFloat(tok[0]);
        const y = parseFloat(tok[1]);
        if (Number.isNaN(x) || Number.isNaN(y)) continue;
        pts.push({ x, y, yaw: parseFloat(tok[2]) || 0 });
      }
      if (!pts.length) throw new Error('文件里没有有效路径点（每行应为 "x y yaw"）');
      pfFilePath = pts;
      setPfStatus('文件预览 ' + pts.length + ' 点 · ' + pfLength(pts).toFixed(2) + ' m');
      addPfLog('已加载包内路径文件预览（' + pts.length + ' 点）');
      updatePfMetrics();
      pfRedraw();
    }).catch((err) => {
      setPfStatus('加载失败：' + ((err && err.message) || err) +
        '（wheeltec_dashboard 重编译启用 /pkg/ 路由了吗？文件在包 share 的 path/ 里吗？）');
    });
  }

  const pfApply = $('pf-apply');
  if (pfApply) pfApply.addEventListener('click', () => {
    if (!ros) { setPfStatus('未连接 rosbridge'); return; }
    pfPath = [];
    pfPathTime = 0;
    subscribePathFollow();
    updatePfMetrics();
    pfRedraw();
    setPfStatus('已重新订阅 ' + ((($('pf-topic') || {}).value) || '/followpath'));
  });
  const pfFileLoad = $('pf-file-load');
  if (pfFileLoad) pfFileLoad.addEventListener('click', pfLoadFile);
  const pfClear = $('pf-clear');
  if (pfClear) pfClear.addEventListener('click', () => {
    pfPath = [];
    pfFilePath = [];
    pfPathTime = 0;
    updatePfMetrics();
    pfRedraw();
    setPfStatus('');
  });

  // 回放前的辅助：把车先 Nav2 到路径起点（follow_path.py 自己也会先去起点，
  // 这里只是手动预摆位/单独导航用），直发 /goal_pose，有二次确认。
  const pfGotoStart = $('pf-goto-start');
  if (pfGotoStart) pfGotoStart.addEventListener('click', () => {
    const pts = pfActivePath();
    if (!pts.length) { setCardMsg('pf-msg', '当前没有路径（先收到 /followpath 或加载文件预览）'); return; }
    if (!goalPosePub) { setCardMsg('pf-msg', '未连接 rosbridge'); return; }
    const p0 = pts[0];
    if (!window.confirm('导航到路径起点 (x=' + p0.x.toFixed(2) + ', y=' + p0.y.toFixed(2) +
        ')？小车将开始移动（需 Nav2 已启动并定位）。')) return;
    const yaw = +(p0.yaw || 0);
    goalPosePub.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      pose: {
        position: { x: p0.x, y: p0.y, z: 0 },
        orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
      },
    }));
    setCardMsg('pf-msg', '已发布 /goal_pose → 路径起点');
    addPfLog('> 导航到路径起点 x=' + p0.x.toFixed(2) + ' y=' + p0.y.toFixed(2));
  });
  const pfLogClear = $('pf-log-clear');
  if (pfLogClear) pfLogClear.addEventListener('click', () => {
    const v = $('pf-log');
    if (v) v.innerHTML = '';
  });

  // ---------- 建图 (wheeltec_robot_slam: GMapping / Cartographer / Toolbox / ORB) ----------
  // 四种建图方式都在机器人端 launch 启动；面板负责：实时显示 /map 生长
  // （地图位图 wpMap、map 系 TF 客户端 wpTf 与航点卡共用）、用 ros.getNodes
  // 判活各 SLAM 节点推断当前模式、调 slam_toolbox / ORB-SLAM2 的保存服务。
  const MP_NODE_LABELS = {
    '/slam_gmapping': 'GMapping (slam_gmapping)',
    '/cartographer_node': 'Cartographer (cartographer_node)',
    '/occupancy_grid_node': 'Cartographer 栅格节点 (occupancy_grid_node)',
    '/slam_toolbox': 'Slam Toolbox (slam_toolbox)',
    '/orb_slam2_rgbd': 'ORB-SLAM2 (orb_slam2_rgbd)',
    '/octomap_server': 'ORB-SLAM2 八叉树 (octomap_server)',
    '/global_rrt': 'RRT 探索·全局检测 (global_rrt)',
    '/local_rrt': 'RRT 探索·局部检测 (local_rrt)',
    '/filter': 'RRT 探索·前沿过滤 (filter)',
    '/assigner': 'RRT 探索·任务分配 (assigner)',
  };
  // 模式归属判定只看各自的主节点（栅格/octomap 是从属节点，单独亮灯）。
  const MP_MODES = [
    { label: 'GMapping', node: '/slam_gmapping' },
    { label: 'Cartographer', node: '/cartographer_node' },
    { label: 'Slam Toolbox', node: '/slam_toolbox' },
    { label: 'ORB-SLAM2', node: '/orb_slam2_rgbd' },
  ];
  const mpNodeState = {};   // 节点名 -> true/false（undefined=未知，首查不写离线日志）

  function addMpLog(text) { appendTimeline('mp-log', 200, text); }

  function updateMpActive() {
    const el = $('mp-active');
    if (!el) return;
    const on = MP_MODES.filter((m) => mpNodeState[m.node]).map((m) => m.label);
    el.classList.remove('ok', 'warn', 'err');
    if (!on.length) {
      el.textContent = ros ? '无（未检测到 SLAM 节点）' : '未连接';
    } else if (on.length === 1) {
      el.textContent = on[0];
      el.classList.add('ok');
    } else {
      // 多种 SLAM 同时在跑会互抢 map→odom TF，标红提醒
      el.textContent = on.join(' + ') + '（同时建图会冲突！）';
      el.classList.add('err');
    }
  }

  // 节点在线轮询（仅"建图"页可见时）：徽标 + 模式推断 + 边沿事件日志。
  setInterval(() => {
    const panel = $('panel-mapping');
    if (!panel || !panel.offsetParent) return;
    const badges = Array.from(document.querySelectorAll('[data-mpnode]'));
    if (!ros) {
      badges.forEach((el) => {
        el.textContent = '—';
        el.classList.remove('ok', 'warn', 'err');
      });
      Object.keys(mpNodeState).forEach((k) => { delete mpNodeState[k]; });
      updateMpActive();
      return;
    }
    ros.getNodes((nodes) => {
      const list = nodes || [];
      badges.forEach((el) => {
        const node = el.dataset.mpnode;
        const on = list.indexOf(node) !== -1;
        if (mpNodeState[node] !== on) {
          // 首查 undefined→离线 不写日志，避免每次进页刷一排"离线"
          if (mpNodeState[node] !== undefined || on) {
            addMpLog((MP_NODE_LABELS[node] || node) + (on ? ' 已上线' : ' 已退出'));
          }
          mpNodeState[node] = on;
        }
        el.textContent = on ? '在线' : '离线';
        el.classList.remove('ok', 'warn', 'err');
        if (on) el.classList.add('ok');
      });
      updateMpActive();
    }, () => { /* getNodes 偶发失败忽略，下个周期重试 */ });
  }, 3000);

  // 实时地图视图 + 指标刷新（仅画布可见时）。
  function mpRedraw() {
    const canvas = $('mp-map');
    if (!canvas || !canvas.offsetParent) return;
    const ctx = canvas.getContext('2d');
    const wrap = canvas.parentElement;
    const maxW = Math.max(240, Math.min((wrap && wrap.clientWidth) || 640, 900));
    if (!wpMap) {
      if (canvas.width !== maxW || canvas.height !== 160) { canvas.width = maxW; canvas.height = 160; }
      ctx.fillStyle = '#161b22';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#8b949e';
      ctx.font = '13px sans-serif';
      ctx.fillText('等待 /map …（启动下方任一建图 launch 后地图会在这里实时生长）', 12, 84);
      return;
    }
    const s = maxW / wpMap.w;
    const h = Math.max(140, Math.round(wpMap.h * s));
    if (canvas.width !== maxW || canvas.height !== h) {
      canvas.width = maxW;
      canvas.height = h;
    }
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, maxW, h);
    ctx.drawImage(wpMap.bmp, 0, 0, maxW, h);
    const tf = wpTf ? wpTf.lookup('base_footprint') : null;
    if (tf) {
      drawWpArrow(ctx, {
        x: (tf.translation.x - wpMap.ox) / wpMap.res * s,
        y: h - (tf.translation.y - wpMap.oy) / wpMap.res * s,
      }, yawFromQuat(tf.rotation), '#3fb950', 9);
    }
  }

  setInterval(() => {
    const canvas = $('mp-map');
    if (!canvas || !canvas.offsetParent) return;
    const infoEl = $('mp-map-info');
    if (infoEl) {
      infoEl.textContent = wpMap
        ? `${wpMap.w}×${wpMap.h} @ ${(+wpMap.res).toFixed(3)} m/格`
        : '—';
    }
    const freshEl = $('mp-map-fresh');
    if (freshEl) {
      if (!wpMapTime) {
        freshEl.textContent = '—';
        freshEl.classList.remove('ok');
      } else {
        const age = (Date.now() - wpMapTime) / 1000;
        // SLAM 建图中 /map 订阅按 2s 节流，5s 内有帧即视为实时
        freshEl.classList.toggle('ok', age < 5);
        freshEl.textContent = age < 5 ? '实时更新中' : Math.round(age) + ' 秒前';
      }
    }
    const tfEl = $('mp-tf');
    if (tfEl) {
      const tf = wpTf ? wpTf.lookup('base_footprint') : null;
      tfEl.classList.remove('ok');
      if (tf) {
        const yaw = yawFromQuat(tf.rotation);
        tfEl.textContent = `x=${tf.translation.x.toFixed(2)}  y=${tf.translation.y.toFixed(2)}  yaw=${(yaw * 180 / Math.PI).toFixed(1)}°`;
        tfEl.classList.add('ok');
      } else {
        tfEl.textContent = ros ? '无 map TF（SLAM 未跑或尚未初始化）' : '未连接';
      }
    }
    if (!wpMap && ros && !wpMapTried) { wpMapTried = true; fetchWpMap(); }
    mpRedraw();
  }, 1000);

  // 保存服务（slam_toolbox / ORB-SLAM2 提供服务接口，可直接面板内保存；
  // gmapping/cartographer 没有保存服务，走通用 map_saver_cli 终端命令）。
  function mpCallService(name, type, req, msgId, okText) {
    if (!ros) { setCardMsg(msgId, '未连接 rosbridge'); return; }
    setCardMsg(msgId, '调用 ' + name + ' …');
    const srv = new ROSLIB.Service({ ros, name, serviceType: type });
    srv.callService(new ROSLIB.ServiceRequest(req), (res) => {
      // slam_toolbox 返回 int32 result（0=成功），ORB 返回 bool success
      let ok = true;
      let detail = '';
      if (res && typeof res.result === 'number') {
        ok = res.result === 0;
        if (!ok) detail = 'result=' + res.result;
      } else if (res && typeof res.success === 'boolean') {
        ok = res.success;
      }
      setCardMsg(msgId, ok ? okText : ('失败' + (detail ? '（' + detail + '）' : '')));
      addMpLog('> ' + name + ' → ' + (ok ? '成功' : '失败 ' + detail));
    }, (err) => {
      setCardMsg(msgId, '调用失败：' + err + '（对应建图节点在跑吗？）');
      addMpLog('> ' + name + ' 调用失败: ' + err);
    });
  }

  const mpTbSave = $('mp-tb-save');
  if (mpTbSave) mpTbSave.addEventListener('click', () => {
    const n = ((($('mp-tb-name') || {}).value) || 'WHEELTEC').trim() || 'WHEELTEC';
    // SaveMap 请求字段是 std_msgs/String，要包一层 { data: … }
    mpCallService('/slam_toolbox/save_map', 'slam_toolbox/srv/SaveMap',
      { name: { data: n } }, 'mp-tb-msg', '已保存 ' + n + '.pgm/.yaml（节点工作目录）');
  });
  const mpTbSerialize = $('mp-tb-serialize');
  if (mpTbSerialize) mpTbSerialize.addEventListener('click', () => {
    const f = ((($('mp-tb-file') || {}).value) || 'WHEELTEC_posegraph').trim() || 'WHEELTEC_posegraph';
    mpCallService('/slam_toolbox/serialize_map', 'slam_toolbox/srv/SerializePoseGraph',
      { filename: f }, 'mp-tb-msg', '已序列化 ' + f + '.posegraph/.data（节点工作目录）');
  });
  const mpOrbSave = $('mp-orb-save');
  if (mpOrbSave) mpOrbSave.addEventListener('click', () => {
    const n = ((($('mp-orb-name') || {}).value) || 'map.bin').trim() || 'map.bin';
    mpCallService('/RGBD/save_map', 'orb_slam2_ros/srv/SaveMap',
      { name: n }, 'mp-orb-msg', '已保存特征地图 ' + n + '（节点工作目录）');
  });
  const mpOrbCloud = $('mp-orb-cloud');
  if (mpOrbCloud) mpOrbCloud.addEventListener('click', () => {
    const n = ((($('mp-orb-name') || {}).value) || 'map.bin').trim() || 'map.bin';
    mpCallService('/RGBD/save_cloud', 'orb_slam2_ros/srv/SaveCloud',
      { name: n }, 'mp-orb-msg', '已保存点云（savePCDDirectory 目录）');
  });
  const mpLogClear = $('mp-log-clear');
  if (mpLogClear) mpLogClear.addEventListener('click', () => {
    const v = $('mp-log');
    if (v) v.innerHTML = '';
  });

  // ---- RRT 自主探索 (wheeltec_robot_rrt + wheeltec_rrt_msg 接口包) ----
  // 探索由 /clicked_point 的 5 个点引导：前 4 个为边界多边形顶点（逆时针、
  // 须把小车圈在内），第 5 个为 RRT 起始点（发布后探索立即开始）。这里把
  // RViz "Publish Point" 的交互搬进面板：点地图画布即发点，并叠加显示
  // RRT 检出前沿(/detected_frontiers)与候选目标(/filtered_goal_points)。
  let rrtPicked = [];      // 本面板已发布的选点 [{x,y}]（节点端无法撤回，仅显示用）
  let rrtDetected = [];    // /detected_frontiers 最近 300 个检出前沿
  let rrtFrontiers = [];   // /filtered_goal_points 当前候选目标
  let rrtFrontTime = 0;    // 最近一次候选目标更新时间

  function setRrtStatus(text) {
    const el = $('mp-rrt-status');
    if (el) el.textContent = text || '';
  }

  function rrtPublishPoint(x, y) {
    if (!rrtClickPub) { setRrtStatus('未连接 rosbridge'); return false; }
    rrtClickPub.publish(new ROSLIB.Message({
      header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
      point: { x, y, z: 0 },
    }));
    return true;
  }

  function updateRrtPickText() {
    const el = $('mp-rrt-pick');
    if (!el) return;
    const n = rrtPicked.length;
    el.classList.remove('ok', 'warn');
    if (!n) {
      el.textContent = '0/5 — 在地图上点第 1 个边界顶点（逆时针圈定区域）';
    } else if (n < 4) {
      el.textContent = n + '/5 — 继续点边界顶点（逆时针）';
      el.classList.add('warn');
    } else if (n === 4) {
      el.textContent = '4/5 — 最后在小车附近点起始点（发布后探索开始）';
      el.classList.add('warn');
    } else {
      el.textContent = '5/5 — 边界与起始点已发布，探索进行中';
      el.classList.add('ok');
    }
  }

  function mpRrtRedraw() {
    const canvas = $('mp-rrt-map');
    if (!canvas || !canvas.offsetParent) return;
    const ctx = canvas.getContext('2d');
    const wrap = canvas.parentElement;
    const maxW = Math.max(240, Math.min((wrap && wrap.clientWidth) || 640, 900));
    if (!wpMap) {
      canvas._view = null;
      if (canvas.width !== maxW || canvas.height !== 160) { canvas.width = maxW; canvas.height = 160; }
      ctx.fillStyle = '#161b22';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#8b949e';
      ctx.font = '13px sans-serif';
      ctx.fillText('等待 /map …（先启动 SLAM/探索 launch，出图后在这里点 5 个点圈定探索区域）', 12, 84);
      return;
    }
    const s = maxW / wpMap.w;
    const h = Math.max(140, Math.round(wpMap.h * s));
    if (canvas.width !== maxW || canvas.height !== h) {
      canvas.width = maxW;
      canvas.height = h;
    }
    canvas._view = { s, h };   // pointer 事件换算用
    const toC = (wx, wy) => ({
      x: (wx - wpMap.ox) / wpMap.res * s,
      y: h - (wy - wpMap.oy) / wpMap.res * s,
    });
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, maxW, h);
    ctx.drawImage(wpMap.bmp, 0, 0, maxW, h);
    // RRT 检出前沿（淡蓝小点，最近 300 个）
    ctx.fillStyle = 'rgba(88, 166, 255, 0.45)';
    rrtDetected.forEach((p) => {
      const c = toC(p.x, p.y);
      ctx.fillRect(c.x - 1.5, c.y - 1.5, 3, 3);
    });
    // 过滤后的候选探索目标（红点）
    ctx.fillStyle = '#f85149';
    rrtFrontiers.forEach((p) => {
      const c = toC(p.x, p.y);
      ctx.beginPath(); ctx.arc(c.x, c.y, 4, 0, Math.PI * 2); ctx.fill();
    });
    // 边界多边形（黄线 + 顶点）与起始点（绿点）
    const bnd = rrtPicked.slice(0, 4);
    if (bnd.length) {
      ctx.strokeStyle = '#d29922';
      ctx.lineWidth = 2;
      ctx.beginPath();
      bnd.forEach((p, i) => {
        const c = toC(p.x, p.y);
        if (!i) ctx.moveTo(c.x, c.y);
        else ctx.lineTo(c.x, c.y);
      });
      if (bnd.length === 4) ctx.closePath();
      ctx.stroke();
      ctx.fillStyle = '#d29922';
      bnd.forEach((p) => {
        const c = toC(p.x, p.y);
        ctx.beginPath(); ctx.arc(c.x, c.y, 3.5, 0, Math.PI * 2); ctx.fill();
      });
    }
    if (rrtPicked.length >= 5) {
      const c = toC(rrtPicked[4].x, rrtPicked[4].y);
      ctx.fillStyle = '#3fb950';
      ctx.beginPath(); ctx.arc(c.x, c.y, 5, 0, Math.PI * 2); ctx.fill();
    }
    // 小车实时位姿（绿色箭头）
    const tf = wpTf ? wpTf.lookup('base_footprint') : null;
    if (tf) {
      drawWpArrow(ctx, toC(tf.translation.x, tf.translation.y),
        yawFromQuat(tf.rotation), '#3fb950', 9);
    }
  }

  // 画布单击选点（不用拖朝向——/clicked_point 只要位置）
  (function initRrtMapPick() {
    const canvas = $('mp-rrt-map');
    if (!canvas) return;
    canvas.addEventListener('pointerdown', (e) => {
      if (e.pointerType === 'mouse' && e.button !== 0) return;
      const view = canvas._view;
      if (!view || !wpMap) {
        setRrtStatus('还没有地图——先启动 SLAM/探索 launch，等 /map 出现再选点');
        return;
      }
      if (rrtPicked.length >= 5) {
        setRrtStatus('已发布 5 个点；重选需重启 rrt_exploration 后点【重新选点】再点图');
        return;
      }
      const r = canvas.getBoundingClientRect();
      if (!r.width || !r.height) return;
      const cx = (e.clientX - r.left) * (canvas.width / r.width);
      const cy = (e.clientY - r.top) * (canvas.height / r.height);
      const wx = cx / view.s * wpMap.res + wpMap.ox;
      const wy = (view.h - cy) / view.s * wpMap.res + wpMap.oy;
      if (rrtPicked.length === 4 &&
          !window.confirm('发布起始点 (x=' + wx.toFixed(2) + ', y=' + wy.toFixed(2) +
            ')？RRT 收到第 5 个点后探索立即开始、小车将自主移动。')) return;
      if (!rrtPublishPoint(wx, wy)) return;
      rrtPicked.push({ x: wx, y: wy });
      addMpLog('> RRT 选点 ' + rrtPicked.length + '/5 → /clicked_point (x=' + wx.toFixed(2) +
        ', y=' + wy.toFixed(2) + ')' + (rrtPicked.length === 5 ? '（探索开始）' : ''));
      setRrtStatus('');
      updateRrtPickText();
      mpRrtRedraw();
      e.preventDefault();
    });
  })();

  // 一键方形边界：以小车当前位姿为中心发 4 顶点（逆时针，与
  // boundary_publisher.py 同序）+ 小车位置为第 5 个起始点。
  const mpRrtSquare = $('mp-rrt-square');
  if (mpRrtSquare) mpRrtSquare.addEventListener('click', () => {
    if (!rrtClickPub) { setRrtStatus('未连接 rosbridge'); return; }
    const tf = wpTf ? wpTf.lookup('base_footprint') : null;
    if (!tf) { setRrtStatus('没有小车位姿（map→base_footprint TF）——SLAM 跑起来了吗？'); return; }
    const half = Math.max(1, +((($('mp-rrt-size') || {}).value) || 5));
    const cx = tf.translation.x;
    const cy = tf.translation.y;
    if (!window.confirm('以小车 (x=' + cx.toFixed(2) + ', y=' + cy.toFixed(2) + ') 为中心发布 ±' +
        half + 'm 方形边界 + 起始点？发布完成后探索立即开始、小车将自主移动。')) return;
    const pts = [
      { x: cx + half, y: cy + half }, { x: cx - half, y: cy + half },
      { x: cx - half, y: cy - half }, { x: cx + half, y: cy - half },
      { x: cx, y: cy },
    ];
    for (const p of pts) {
      if (!rrtPublishPoint(p.x, p.y)) return;
    }
    rrtPicked = pts;
    addMpLog('> RRT 方形边界已发布（中心 x=' + cx.toFixed(2) + ' y=' + cy.toFixed(2) +
      '，±' + half + 'm，探索开始）');
    setRrtStatus('方形边界 + 起始点已发布');
    updateRrtPickText();
    mpRrtRedraw();
  });

  const mpRrtReset = $('mp-rrt-reset');
  if (mpRrtReset) mpRrtReset.addEventListener('click', () => {
    rrtPicked = [];
    setRrtStatus('已清面板选点显示（RRT 节点端已收的点不会撤回，需重启 rrt_exploration 再重点）');
    updateRrtPickText();
    mpRrtRedraw();
  });

  // RRT 子页周期刷新（仅画布可见时）：候选目标数/新鲜度 + 选点进度 + 重绘。
  setInterval(() => {
    const canvas = $('mp-rrt-map');
    if (!canvas || !canvas.offsetParent) return;
    const fEl = $('mp-rrt-frontiers');
    if (fEl) {
      if (!rrtFrontTime) {
        fEl.textContent = ros ? '—（filter 未发布）' : '未连接';
        fEl.classList.remove('ok');
      } else {
        const age = (Date.now() - rrtFrontTime) / 1000;
        fEl.classList.toggle('ok', age < 5 && rrtFrontiers.length > 0);
        fEl.textContent = rrtFrontiers.length + ' 个' +
          (age < 5 ? '（实时）' : '（' + Math.round(age) + ' 秒前）');
      }
    }
    updateRrtPickText();
    if (!wpMap && ros && !wpMapTried) { wpMapTried = true; fetchWpMap(); }
    mpRrtRedraw();
  }, 1000);

  // ---------- 超声波转换 (wheeltec_ultrasonic) ----------
  // supersonic_converter 把下位机 /Distance(Supersonic 多路打包, 源头是
  // 固件 19 字节超声波帧 0xFA…0xFC)拆成标准 /ultrasonic/A..F(Range)与
  // /ultrasonic/points(点云, base_footprint 系)供 Nav2 避障。面板按
  // ultrasonic_A..F 的 TF 真实安装位姿画俯视波束图；话题有数据=节点在线，
  // 上线边沿自动读参数（节点只在启动时读参，运行中改不生效，故不做调参）。
  const US_LABELS = ['A', 'B', 'C', 'D', 'E', 'F'];
  const usRanges = {};            // 'A'.. -> { range|null(∞/无效), time }
  let usPointsInfo = { n: 0, time: 0 };
  let usOnline = null;            // 话题判活状态（null=未知）
  const usCfg = { type: '', minR: 0.02, maxR: 0.8, fov: 0.5, cloud: null };

  function usReadParams() {
    paramService('/supersonic_converter', 'get_parameters').callService(
      new ROSLIB.ServiceRequest({
        names: ['robot_type', 'min_range', 'max_range', 'field_of_view', 'publish_pointcloud'],
      }), (res) => {
        const v = (res && res.values) || [];
        if (v[0] && v[0].type === PT_STRING) usCfg.type = v[0].string_value;
        if (v[1] && v[1].type === PT_DOUBLE) usCfg.minR = v[1].double_value;
        if (v[2] && v[2].type === PT_DOUBLE) usCfg.maxR = v[2].double_value;
        if (v[3] && v[3].type === PT_DOUBLE) usCfg.fov = v[3].double_value;
        if (v[4] && v[4].type === PT_BOOL) usCfg.cloud = v[4].bool_value;
        const el = $('us-cfg');
        if (el) {
          el.textContent = (usCfg.type || '?') + ' · ' + usCfg.minR.toFixed(2) + '–' +
            usCfg.maxR.toFixed(2) + ' m · FOV ' + usCfg.fov.toFixed(2) + ' rad · 点云' +
            (usCfg.cloud === false ? '关' : '开');
        }
      }, () => { /* 节点刚退出等竞态，忽略 */ });
  }

  function usDraw() {
    const canvas = $('us-top');
    if (!canvas || !canvas.offsetParent) return;
    const wrap = canvas.parentElement;
    const W = Math.max(240, Math.min((wrap && wrap.clientWidth) || 480, 640));
    const H = 240;
    if (canvas.width !== W || canvas.height !== H) {
      canvas.width = W;
      canvas.height = H;
    }
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#161b22';
    ctx.fillRect(0, 0, W, H);

    // 传感器位姿：TF 优先；缺失时按均匀扇形近似摆放（A 左 → F 右）
    const labels = usCfg.type === 's300_mini' ? US_LABELS.slice(0, 5) : US_LABELS;
    const sensors = labels.map((lb, i) => {
      const tf = usTf ? usTf.lookup('ultrasonic_' + lb) : null;
      if (tf) {
        return { lb, x: tf.translation.x, y: tf.translation.y, yaw: yawFromQuat(tf.rotation), approx: false };
      }
      return { lb, x: 0, y: 0, yaw: ((labels.length - 1) / 2 - i) * (Math.PI / 6), approx: true };
    });
    const anyTf = sensors.some((s) => !s.approx);

    const maxR = usCfg.maxR || 0.8;
    let extX = 0.3, extY = 0.3;
    sensors.forEach((s) => {
      extX = Math.max(extX, s.x + maxR);
      extY = Math.max(extY, Math.abs(s.y) + maxR);
    });
    // 车头朝上：世界 x(前)→画布上、y(左)→画布左
    const ox = W / 2;
    const oy = H - 34;
    const k = Math.min((oy - 16) / extX, (W / 2 - 16) / extY);
    const toC = (wx, wy) => ({ x: ox - wy * k, y: oy - wx * k });

    // 量程刻度弧线（每 0.25m，前半圆）
    ctx.strokeStyle = 'rgba(139, 148, 158, 0.18)';
    ctx.fillStyle = 'rgba(139, 148, 158, 0.55)';
    ctx.font = '10px sans-serif';
    ctx.lineWidth = 1;
    for (let r = 0.25; r <= extX + 0.01; r += 0.25) {
      ctx.beginPath();
      ctx.arc(ox, oy, r * k, Math.PI, 2 * Math.PI);
      ctx.stroke();
      ctx.fillText(r.toFixed(2), ox + 3, oy - r * k - 2);
    }

    // 各路波束扇形
    const now = Date.now();
    const half = (usCfg.fov || 0.5) / 2;
    sensors.forEach((s) => {
      const st = usRanges[s.lb];
      const fresh = st && (now - st.time) < 2500;
      const r = (fresh && typeof st.range === 'number') ? st.range : null;
      const valid = r !== null && r > 0;
      const beamR = valid ? r : maxR;
      const p0 = toC(s.x, s.y);
      // 世界 yaw=0(正前) → 画布 -90°；yaw 增大(左转) → 画布角减小
      const a0 = -Math.PI / 2 - s.yaw - half;
      const a1 = -Math.PI / 2 - s.yaw + half;
      ctx.beginPath();
      ctx.moveTo(p0.x, p0.y);
      ctx.arc(p0.x, p0.y, beamR * k, a0, a1);
      ctx.closePath();
      if (valid) {
        const col = r < 0.3 ? '248, 81, 73' : (r < 0.6 ? '210, 153, 34' : '63, 185, 80');
        ctx.fillStyle = 'rgba(' + col + ', 0.28)';
        ctx.fill();
        ctx.strokeStyle = 'rgba(' + col + ', 0.9)';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      } else {
        ctx.strokeStyle = 'rgba(139, 148, 158, 0.35)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 3]);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      // 传感器位置点 + 字母与读数（沿波束方向外侧）
      ctx.fillStyle = '#58a6ff';
      ctx.beginPath(); ctx.arc(p0.x, p0.y, 2.5, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = valid ? '#e6edf3' : 'rgba(139, 148, 158, 0.8)';
      ctx.font = '11px sans-serif';
      const tip = toC(s.x + Math.cos(s.yaw) * (beamR + 0.08), s.y + Math.sin(s.yaw) * (beamR + 0.08));
      ctx.fillText(s.lb + (valid ? ' ' + r.toFixed(2) : ' ∞'), tip.x - 10, tip.y);
    });

    // 小车示意（base_footprint 原点 + 航向三角，车头朝上）
    ctx.fillStyle = '#3fb950';
    ctx.beginPath();
    ctx.moveTo(ox, oy - 10);
    ctx.lineTo(ox - 6, oy + 6);
    ctx.lineTo(ox + 6, oy + 6);
    ctx.closePath();
    ctx.fill();
    if (!anyTf) {
      ctx.fillStyle = 'rgba(210, 153, 34, 0.9)';
      ctx.font = '11px sans-serif';
      ctx.fillText('无 ultrasonic_* TF——按近似角度摆放（启动底盘后为真实安装位姿）', 10, 14);
    }
  }

  // 周期刷新（仅卡片可见时）：判活/上线读参 + 点云指标 + 俯视图重绘。
  setInterval(() => {
    const canvas = $('us-top');
    if (!canvas || !canvas.offsetParent) return;
    const now = Date.now();
    let latest = 0;
    US_LABELS.forEach((lb) => {
      const s = usRanges[lb];
      if (s && s.time > latest) latest = s.time;
    });
    const on = !!(ros && latest && (now - latest) < 3000);
    if (on !== usOnline) {
      if (on) usReadParams();
      usOnline = on;
    }
    const convEl = $('us-conv');
    if (convEl) {
      convEl.textContent = !ros ? '未连接' : (on ? '在线' : '离线（converter launch 未启动？）');
      convEl.classList.remove('ok', 'err');
      if (on) convEl.classList.add('ok');
    }
    const ptsEl = $('us-points');
    if (ptsEl) {
      if (!usPointsInfo.time) {
        ptsEl.textContent = usCfg.cloud === false ? '已关闭 (publish_pointcloud=false)' : '—';
        ptsEl.classList.remove('ok');
      } else {
        const age = (now - usPointsInfo.time) / 1000;
        ptsEl.classList.toggle('ok', age < 3);
        ptsEl.textContent = usPointsInfo.n + ' 点' +
          (age < 3 ? '（实时）' : '（' + Math.round(age) + ' 秒前）');
      }
    }
    usDraw();
  }, 500);

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

  // 系统架构卡的在线指示点：底盘遥测 / 机械臂驱动 / 抓取目标 三路数据流。
  function updateArchDots() {
    const now = Date.now();
    const set = (id, on) => {
      const el = $(id);
      if (el) el.classList.toggle('on', !!on);
    };
    set('arch-dot-chassis', chassisTime && now - chassisTime < 3000);
    set('arch-dot-arm', armStatusTime && now - armStatusTime < 3000);
    set('arch-dot-grab', armDistTime && now - armDistTime < 2500);
  }

  // 顶栏"机械臂"徽标：全页面常驻的在线指示（/robot_status 3s 内有数据=在线，
  // 未连 rosbridge 时显示 "—"）。
  function updateArmBadge() {
    const el = $('arm-online');
    if (!el) return;
    let text;
    let cls;
    if (!ros) { text = '机械臂: —'; cls = 'ctrl-src-idle'; }
    else if (armStatusTime && Date.now() - armStatusTime < 3000) {
      text = '机械臂: 在线'; cls = 'ctrl-src-active';
    } else { text = '机械臂: 离线'; cls = 'ctrl-src-err'; }
    el.textContent = text;
    el.className = 'ctrl-src ' + cls;
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
    updateArchDots();
    updateArmBadge();
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
    const view = $('arm-joints');
    if (!view) return;
    // 面板隐藏时（display:none → offsetParent 为 null）不重绘，dirty 保留，
    // 切回机械臂页的下一个周期再画 —— 隐藏期间不做无效 DOM churn。
    if (!view.offsetParent) return;
    armJointsDirty = false;
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
    appendTimeline('arm-log', ARM_LOG_MAX_LINES, text);   // 与 VLA 时间线共用渲染
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
  if (gripPosApply && gripPosRange) gripPosApply.addEventListener('click', () => setGripper('position', gripPosRange.value));
  const gripForceApply = $('arm-grip-force-apply');
  if (gripForceApply && gripForceRange) gripForceApply.addEventListener('click', () => setGripper('force', gripForceRange.value));
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

  // 预设关节位（SRDF 命名位姿 look/zero + arm_demo 的 FK 演示目标）：只填值不执行。
  document.querySelectorAll('#panel-arm .mj-preset').forEach((btn) => {
    btn.addEventListener('click', () => {
      const vals = (btn.dataset.pose || '').split(',').map(Number);
      if (vals.length < armMjInputs.length || vals.some(Number.isNaN)) return;
      armMjInputs.forEach((inp, i) => { inp.value = vals[i]; });
      addArmLog('已填入预设关节角：' + btn.textContent.trim() + '（点"执行运动"生效）');
    });
  });

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

  // ---- 仿真关节控制（"机械臂仿真"页）: 滑块 -> JointTrajectory -> Gazebo ros2_control ----
  // 与真机控制(走乐白驱动服务)互不干扰; 仿真里 /joint_states 的关节与真机同名 lebai_joint_1..6。
  const ARMSIM_JOINTS = ['lebai_joint_1', 'lebai_joint_2', 'lebai_joint_3',
                         'lebai_joint_4', 'lebai_joint_5', 'lebai_joint_6'];
  const armSimSliders = Array.from(document.querySelectorAll('#panel-armsim .armsim-j'));
  const armSimVals = Array.from(document.querySelectorAll('#panel-armsim .armsim-val'));
  const armSimStatusEl = $('armsim-traj-status');

  function armSimSetStatus(text) { if (armSimStatusEl) armSimStatusEl.textContent = text; }

  function armSimShowVals() {
    armSimSliders.forEach((sl, i) => { if (armSimVals[i]) armSimVals[i].textContent = sl.value + '°'; });
  }
  armSimSliders.forEach((sl) => sl.addEventListener('input', armSimShowVals));
  armSimShowVals();

  function armSimCurrentRad() {
    if (!ARMSIM_JOINTS.every((n) => armJointMap.has(n))) return null;
    return ARMSIM_JOINTS.map((n) => armJointMap.get(n).pos);
  }

  function armSimSend(points, label) {
    if (!armSimTrajPub) { armSimSetStatus('rosbridge 未连接，无法控制'); return; }
    armSimTrajPub.publish(new ROSLIB.Message({ joint_names: ARMSIM_JOINTS, points }));
    armSimSetStatus(label + ' 已下发 ' + new Date().toLocaleTimeString());
  }

  // 时长按最大关节行程估算(~0.6 rad/s, 限 1.5~8s); 收不到当前关节角时退回 3s。
  function armSimRunTo(targetRad, label) {
    const cur = armSimCurrentRad();
    let dur = 3;
    if (cur) {
      const maxDelta = Math.max(...targetRad.map((v, i) => Math.abs(v - cur[i])));
      dur = Math.min(8, Math.max(1.5, maxDelta / 0.6));
    }
    const sec = Math.floor(dur);
    armSimSend([{ positions: targetRad, time_from_start: { sec, nanosec: Math.round((dur - sec) * 1e9) } }], label);
  }

  const armSimRunBtn = $('armsim-traj-run');
  if (armSimRunBtn) armSimRunBtn.addEventListener('click', () => {
    const target = armSimSliders.map((sl) => parseFloat(sl.value) * Math.PI / 180);
    armSimRunTo(target, '目标 [' + armSimSliders.map((sl) => sl.value + '°').join(', ') + ']');
  });

  const armSimZeroBtn = $('armsim-traj-zero');
  if (armSimZeroBtn) armSimZeroBtn.addEventListener('click', () => {
    armSimSliders.forEach((sl) => { sl.value = 0; });
    armSimShowVals();
    armSimRunTo([0, 0, 0, 0, 0, 0], '回零位');
  });

  const armSimDemoBtn = $('armsim-traj-demo');
  if (armSimDemoBtn) armSimDemoBtn.addEventListener('click', () => {
    armSimSend([
      { positions: [0.6, -0.4, 0.5, 0.3, 0.5, 0], time_from_start: { sec: 3, nanosec: 0 } },
      { positions: [-0.6, -0.4, 0.5, 0.3, -0.5, 0], time_from_start: { sec: 6, nanosec: 0 } },
      { positions: [0, 0, 0, 0, 0, 0], time_from_start: { sec: 9, nanosec: 0 } },
    ], '演示动作');
  });

  const armSimFillBtn = $('armsim-traj-fill');
  if (armSimFillBtn) armSimFillBtn.addEventListener('click', () => {
    const cur = armSimCurrentRad();
    if (!cur) { armSimSetStatus('未收到 /joint_states（仿真是否已启动？）'); return; }
    armSimSliders.forEach((sl, i) => { sl.value = Math.round(cur[i] * 180 / Math.PI); });
    armSimShowVals();
    armSimSetStatus('已同步当前关节角');
  });

  // 位姿运动 IK：末端 X/Y/Z + RPY -> /motion_service/move_joint|move_line
  // （is_joint_pose=false，逆解在乐白控制器内完成）。arm_demo ik_demo 的面板版。
  function quatFromRpy(roll, pitch, yaw) {
    const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
    const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
    const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
    return {
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy,
      w: cr * cp * cy + sr * sp * sy,
    };
  }

  function armCartesianMove(kind) {
    const x = parseFloat($('arm-ik-x').value);
    const y = parseFloat($('arm-ik-y').value);
    const z = parseFloat($('arm-ik-z').value);
    const roll = parseFloat($('arm-ik-r').value) * Math.PI / 180;
    const pitch = parseFloat($('arm-ik-p').value) * Math.PI / 180;
    const yaw = parseFloat($('arm-ik-yw').value) * Math.PI / 180;
    if ([x, y, z, roll, pitch, yaw].some(Number.isNaN)) { alert('位姿必须是数字'); return; }
    const acc = parseFloat($('arm-ik-acc').value);
    const vel = parseFloat($('arm-ik-vel').value);
    if (!(acc > 0) || !(vel > 0)) { alert('acc / vel 必须为正数'); return; }
    const srv = kind === 'line' ? 'move_line' : 'move_joint';
    const type = kind === 'line' ? 'lebai_interfaces/srv/MoveLine' : 'lebai_interfaces/srv/MoveJoint';
    const txt = `x=${x.toFixed(3)}, y=${y.toFixed(3)}, z=${z.toFixed(3)}`;
    if (!window.confirm('将真实移动机械臂末端到 (' + txt + ') m（' +
        (kind === 'line' ? '笛卡尔直线' : '关节插补') + '，控制器解 IK，无碰撞检查），确认执行？')) return;
    addArmLog('> ' + srv + ' (' + txt + ') acc=' + acc + ' vel=' + vel);
    callArmService('/motion_service/' + srv, type, {
      is_joint_pose: false,
      joint_pose: [],
      cartesian_pose: {
        position: { x, y, z },
        orientation: quatFromRpy(roll, pitch, yaw),
      },
      common: { acc, vel, time: 0, radius: 0 },
    }, (res) => {
      addArmLog(res && res.ret ? '✓ ' + srv + ' 已执行'
        : '✗ ' + srv + ' 失败（目标可能不可达，看驱动日志）');
    });
  }

  const armIkRun = $('arm-ik-run');
  if (armIkRun) armIkRun.addEventListener('click', () => armCartesianMove('joint'));
  const armIkLine = $('arm-ik-line');
  if (armIkLine) armIkLine.addEventListener('click', () => armCartesianMove('line'));

  const armIkDemo = $('arm-ik-demo');
  if (armIkDemo) {
    armIkDemo.addEventListener('click', () => {
      // arm_demo ik_demo.cpp 的演示目标位姿（四元数转 RPY 填入）。
      $('arm-ik-x').value = 0.6;
      $('arm-ik-y').value = 0.17;
      $('arm-ik-z').value = 0.46;
      const rpy = rpyFromQuat({ x: 0.16, y: 0.05, z: 0.013, w: 0.98 });
      $('arm-ik-r').value = (rpy.roll * 180 / Math.PI).toFixed(1);
      $('arm-ik-p').value = (rpy.pitch * 180 / Math.PI).toFixed(1);
      $('arm-ik-yw').value = (rpy.yaw * 180 / Math.PI).toFixed(1);
      addArmLog('已填入 arm_demo IK 演示位（点"执行运动"生效）');
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

  // ---- HSV 阈值滑条 (HSV 抓取 /color_node、KCF 播种 /kcf_node 共用组件) ----
  // 拖动即下发 set_parameters(int)（120ms 防抖，同 dist-slider 的做法），
  // "读取当前值"用 get_parameters 把节点当前阈值同步回滑条。
  function initHsvGroup(root) {
    const nodeInput = root.querySelector('.hsv-node');
    if (!nodeInput) return;
    const node = () => nodeInput.value.trim();
    const sliders = Array.from(root.querySelectorAll('.hsv-slider[data-param]'));

    sliders.forEach((row) => {
      const range = row.querySelector('input[type=range]');
      const valEl = row.querySelector('.hs-val');
      if (!range) return;
      const show = () => { if (valEl) valEl.textContent = range.value; };
      const send = () => {
        if (!ros) return;
        const value = buildParameterValue('int', range.value);
        if (!value) return;
        const req = new ROSLIB.ServiceRequest({
          parameters: [{ name: row.dataset.param, value }],
        });
        paramService(node(), 'set_parameters').callService(req, () => {},
          (err) => console.error(`set ${row.dataset.param} failed`, err));
      };
      let timer = null;
      range.addEventListener('input', () => {
        show();
        if (timer) clearTimeout(timer);    // 拖动中防抖
        timer = setTimeout(() => { timer = null; send(); }, 120);
      });
      range.addEventListener('change', () => {
        if (timer) { clearTimeout(timer); timer = null; }  // 松手立即发一次
        send();
      });
      show();
    });

    const refreshBtn = root.querySelector('.hsv-refresh');
    if (refreshBtn) refreshBtn.addEventListener('click', () => {
      if (!ros) { alert('未连接 rosbridge'); return; }
      const names = sliders.map((r) => r.dataset.param);
      const req = new ROSLIB.ServiceRequest({ names });
      paramService(node(), 'get_parameters').callService(req, (res) => {
        (res.values || []).forEach((val, i) => {
          const row = sliders[i];
          if (!row || !val) return;
          let v = null;
          if (val.type === PT_INTEGER) v = val.integer_value;
          else if (val.type === PT_DOUBLE) v = val.double_value;
          if (v === null) return;
          const range = row.querySelector('input[type=range]');
          const valEl = row.querySelector('.hs-val');
          if (range) range.value = v;
          if (valEl) valEl.textContent = String(v);
        });
      }, (err) => alert('读取失败：' + err + '（节点是否已启动？）'));
    });
  }
  document.querySelectorAll('.hsv-group').forEach(initHsvGroup);

  // ---- KCF: 在跟踪画面上拖拽框选目标 ----
  // 显示坐标 -> object-fit:contain 内容区 -> 原始图像像素，发布
  // sensor_msgs/RegionOfInterest 到 /kcf_node/select_bbox，节点收到即重新播种。
  (function initKcfBoxSelect() {
    const frame = $('arm-kcf-frame');
    const img = $('arm-kcf-stream');
    const rect = $('arm-kcf-rect');
    if (!frame || !img || !rect) return;
    let dragging = false;
    let sx = 0, sy = 0;   // 起点（frame 内坐标）

    function frameXY(e) {
      const r = frame.getBoundingClientRect();
      return {
        x: Math.max(0, Math.min(r.width, e.clientX - r.left)),
        y: Math.max(0, Math.min(r.height, e.clientY - r.top)),
        fw: r.width, fh: r.height,
      };
    }
    // object-fit: contain 的实际渲染区域（扣掉上下/左右黑边）
    function contentBox(fw, fh) {
      const nw = img.naturalWidth, nh = img.naturalHeight;
      if (!nw || !nh) return null;
      const s = Math.min(fw / nw, fh / nh);
      return { x: (fw - nw * s) / 2, y: (fh - nh * s) / 2, scale: s, nw, nh };
    }

    frame.addEventListener('pointerdown', (e) => {
      if (e.pointerType === 'mouse' && e.button !== 0) return;
      if ((img.src || '').indexOf('placeholder') !== -1) {
        addArmLog('跟踪画面尚未加载，无法框选（kcf_grab 是否已启动？）');
        return;
      }
      dragging = true;
      const p = frameXY(e);
      sx = p.x; sy = p.y;
      rect.hidden = false;
      rect.style.left = sx + 'px';
      rect.style.top = sy + 'px';
      rect.style.width = '0px';
      rect.style.height = '0px';
      try { frame.setPointerCapture(e.pointerId); } catch (_) { /* ignore */ }
      e.preventDefault();
    });
    frame.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      const p = frameXY(e);
      rect.style.left = Math.min(sx, p.x) + 'px';
      rect.style.top = Math.min(sy, p.y) + 'px';
      rect.style.width = Math.abs(p.x - sx) + 'px';
      rect.style.height = Math.abs(p.y - sy) + 'px';
    });
    frame.addEventListener('pointerup', (e) => {
      if (!dragging) return;
      dragging = false;
      rect.hidden = true;
      const p = frameXY(e);
      const cb = contentBox(p.fw, p.fh);
      if (!cb) { addArmLog('跟踪画面尺寸未知，无法框选'); return; }
      const x0 = Math.min(sx, p.x), y0 = Math.min(sy, p.y);
      const x1 = Math.max(sx, p.x), y1 = Math.max(sy, p.y);
      const ix0 = Math.max(0, Math.round((x0 - cb.x) / cb.scale));
      const iy0 = Math.max(0, Math.round((y0 - cb.y) / cb.scale));
      const ix1 = Math.min(cb.nw, Math.round((x1 - cb.x) / cb.scale));
      const iy1 = Math.min(cb.nh, Math.round((y1 - cb.y) / cb.scale));
      const w = ix1 - ix0, h = iy1 - iy0;
      if (w < 8 || h < 8) { addArmLog('框选太小（不足 8×8 像素），已忽略'); return; }
      if (!kcfBboxPub) { addArmLog('未连接 rosbridge，无法发送框选'); return; }
      kcfBboxPub.publish(new ROSLIB.Message({
        x_offset: ix0, y_offset: iy0, height: h, width: w, do_rectify: false,
      }));
      addArmLog(`> KCF 手动框选 [x=${ix0} y=${iy0} w=${w} h=${h}] → /kcf_node/select_bbox`);
    });
    frame.addEventListener('pointercancel', () => { dragging = false; rect.hidden = true; });
  })();

  const armKcfReinit = $('arm-kcf-reinit');
  if (armKcfReinit) {
    armKcfReinit.addEventListener('click', () => {
      addArmLog('> KCF 重新播种 (/kcf_node/reinit)');
      callArmService('/kcf_node/reinit', 'std_srvs/srv/Trigger', {}, (res) => {
        addArmLog((res && res.success ? '✓ ' : '✗ ') + ((res && res.message) || 'reinit'));
      });
    });
  }

  // ---- YOLO: 识别物体按钮 -> 选为抓取目标 ----
  // 订阅 /yolo/detections，把每个类别渲染成按钮；点击把类别名写进桥接节点
  // (/yolo_ros_node) 的 target_label 参数 —— 调试图与 target_frame 立刻只跟该类别。
  let armYoloSelected = '';
  let armYoloLastHtml = '';   // 上次渲染的按钮 HTML，内容没变就不重建（保留 hover/焦点）

  function armYoloNode() {
    const inp = document.querySelector('#arm-subpanel-yolo .pg-node');
    return ((inp && inp.value) || '/yolo_ros_node').trim();
  }

  function renderArmYoloButtons(msg) {
    const view = $('arm-yolo-buttons');
    if (!view) return;
    const dets = (msg && msg.detections) || [];
    const cntEl = $('arm-yolo-count');
    if (cntEl) cntEl.textContent = dets.length + ' 个';
    // 同类合并: 显示一次 + 数量 + 最高置信度（target_label 按类别筛选，无法区分同类个体）
    const byLabel = new Map();
    dets.forEach((d) => {
      const label = (d && d.class_name != null && d.class_name !== '')
        ? String(d.class_name)
        : String((d && d.class_id != null) ? d.class_id : '?');
      const score = (d && typeof d.score === 'number') ? d.score : 0;
      const cur = byLabel.get(label);
      if (!cur) byLabel.set(label, { count: 1, best: score });
      else { cur.count += 1; if (score > cur.best) cur.best = score; }
    });
    const sel = (armYoloSelected || '').toLowerCase();
    const html = [];
    byLabel.forEach((v, label) => {
      const active = (label.toLowerCase() === sel && sel !== '') ? ' active' : '';
      html.push(
        `<button type="button" data-label="${escapeHTML(label)}" class="yp${active}">` +
        `${escapeHTML(label)}${v.count > 1 ? ' ×' + v.count : ''}` +
        ` <span>${(v.best * 100).toFixed(0)}%</span></button>`
      );
    });
    const out = html.join('');
    if (out === armYoloLastHtml) return;   // 同样的检测结果不重建 DOM
    armYoloLastHtml = out;
    view.innerHTML = out;
  }

  function selectArmYoloTarget(label) {
    if (!ros) { addArmLog('未连接 rosbridge，无法设置抓取目标'); return; }
    const value = buildParameterValue('string', label);
    const req = new ROSLIB.ServiceRequest({ parameters: [{ name: 'target_label', value }] });
    paramService(armYoloNode(), 'set_parameters').callService(req, (res) => {
      const ok = res.results && res.results[0] && res.results[0].successful;
      if (!ok) { addArmLog('✗ 设置 target_label 失败（节点是否已启动？）'); return; }
      armYoloSelected = label;
      addArmLog(label ? ('> 抓取目标类别 → ' + label) : '> 已清除类别筛选（抓任意类别）');
      const view = $('arm-yolo-buttons');
      if (view) {
        view.querySelectorAll('button[data-label]').forEach((b) => {
          b.classList.toggle('active', b.dataset.label.toLowerCase() === label.toLowerCase() && label !== '');
        });
      }
    }, (err) => addArmLog('✗ 设置 target_label 失败：' + err));
  }

  const armYoloButtonsView = $('arm-yolo-buttons');
  if (armYoloButtonsView) {
    // 按钮随检测重渲染，用事件委托避免反复绑定
    armYoloButtonsView.addEventListener('click', (e) => {
      const b = e.target.closest('button[data-label]');
      if (b) selectArmYoloTarget(b.dataset.label);
    });
  }
  const armYoloClear = $('arm-yolo-clear');
  if (armYoloClear) armYoloClear.addEventListener('click', () => selectArmYoloTarget(''));

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
  // 系统总览页的"系统日志"镜像视图：共用 logEntries 缓冲与暂停/清空，
  // 只有等级/节点过滤、自动滚动是自己的。
  const ovLogView = $('ov-log-view');
  const ovLogLevel = $('ov-log-level');
  const ovLogFilter = $('ov-log-filter');
  const ovLogAutoscroll = $('ov-log-autoscroll');

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

  function matchesOvFilters(e) {
    if (e.level < +((ovLogLevel && ovLogLevel.value) || 20)) return false;
    const f = ((ovLogFilter && ovLogFilter.value) || '').trim().toLowerCase();
    if (f && !e.name.toLowerCase().includes(f)) return false;
    return true;
  }

  function rerenderAll() {
    logView.innerHTML = logEntries.filter(matchesFilters).map(renderRow).join('');
    logCountEl.textContent = String(logEntries.length);
    if (logAutoscroll.checked) logView.scrollTop = logView.scrollHeight;
  }

  function rerenderOvLog() {
    if (!ovLogView) return;
    ovLogView.innerHTML = logEntries.filter(matchesOvFilters).map(renderRow).join('');
    if (!ovLogAutoscroll || ovLogAutoscroll.checked) {
      ovLogView.scrollTop = ovLogView.scrollHeight;
    }
  }

  function pushEntry(e) {
    const cap = Math.max(50, Math.min(5000, parseInt(logBuffer.value, 10) || 500));
    logEntries.push(e);
    if (logEntries.length > cap) {
      logEntries.splice(0, logEntries.length - cap);
    }
    logCountEl.textContent = String(logEntries.length);
    if (matchesFilters(e)) {
      // Cheap append: also trim rendered children to ~cap.
      logView.insertAdjacentHTML('beforeend', renderRow(e));
      while (logView.childElementCount > cap) logView.removeChild(logView.firstChild);
      if (logAutoscroll.checked) logView.scrollTop = logView.scrollHeight;
    }
    if (ovLogView && matchesOvFilters(e)) {
      ovLogView.insertAdjacentHTML('beforeend', renderRow(e));
      while (ovLogView.childElementCount > cap) ovLogView.removeChild(ovLogView.firstChild);
      if (!ovLogAutoscroll || ovLogAutoscroll.checked) {
        ovLogView.scrollTop = ovLogView.scrollHeight;
      }
    }
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
    rerenderOvLog();
  });
  if (ovLogLevel) ovLogLevel.addEventListener('change', rerenderOvLog);
  if (ovLogFilter) ovLogFilter.addEventListener('input', rerenderOvLog);
  if (ovLogAutoscroll) {
    ovLogAutoscroll.addEventListener('change', () => {
      if (ovLogAutoscroll.checked && ovLogView) {
        ovLogView.scrollTop = ovLogView.scrollHeight;
      }
    });
  }
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
    if (ovLogView) ovLogView.innerHTML = '';   // 共用缓冲，总览镜像一并清
  });

  // ---------- AI 对话 (ollama_ros_chat / DeepSeek API) ----------
  // 三种后端共用一套气泡 UI：
  //   service  — rosbridge 调 /chat_service (ollama_ros_msgs/srv/Chat)，同步等完整回答
  //   topic    — 发 /chat_message、订阅 /chat_response 按 chunk 流式渲染 (topic_server)
  //   deepseek — 浏览器直连 DeepSeek API (OpenAI 兼容 /chat/completions, SSE 流式)，
  //              Key/模型/Base URL 存 localStorage，上下文在前端维护（车端无依赖）
  const chatBox = $('chat-box');
  const chatInput = $('chat-input');
  const chatSendBtn = $('chat-send');
  const chatBackendSel = $('chat-backend');
  const chatModelEl = $('chat-model');
  const chatDsCfg = $('chat-ds-cfg');
  const chatDsKey = $('chat-ds-key');
  const chatDsModel = $('chat-ds-model');
  const chatDsBase = $('chat-ds-base');

  let chatPendingEl = null;   // 正在生成的 assistant 气泡（流式追加目标）
  let chatBusy = false;
  let chatBusyTimer = null;
  const CHAT_DS_SYSTEM = { role: 'system', content: 'You are a helpful assistant' };
  let chatDsHistory = [CHAT_DS_SYSTEM];   // DeepSeek 模式的前端上下文（≤20 条）

  function chatScroll() {
    if (chatBox) chatBox.scrollTop = chatBox.scrollHeight;
  }

  function chatAppend(role, text, opts) {
    if (!chatBox) return null;
    const empty = chatBox.querySelector('.chat-empty');
    if (empty) empty.remove();
    const div = document.createElement('div');
    div.className = 'chat-msg ' + (role === 'user' ? 'user' : 'assistant')
      + (role === 'error' ? ' error' : '')
      + ((opts && opts.pending) ? ' pending' : '');
    const roleEl = document.createElement('div');
    roleEl.className = 'chat-role';
    roleEl.textContent = role === 'user' ? '我'
      : (role === 'error' ? '错误' : 'AI · ' + new Date().toLocaleTimeString());
    const textEl = document.createElement('div');
    textEl.className = 'chat-text';
    textEl.textContent = text || '';
    div.appendChild(roleEl);
    div.appendChild(textEl);
    chatBox.appendChild(div);
    chatScroll();
    return div;
  }

  function chatSetModel(name) {
    if (chatModelEl && name) chatModelEl.textContent = '模型: ' + name;
  }

  function chatSetBusy(busy) {
    chatBusy = busy;
    if (chatSendBtn) {
      chatSendBtn.disabled = busy;
      chatSendBtn.textContent = busy ? '生成中…' : '发送';
    }
    clearTimeout(chatBusyTimer);
    if (busy) {
      // 兜底：后端没回 is_done / 服务超时等异常情况下 3 分钟自动解锁输入
      chatBusyTimer = setTimeout(() => chatFinishPending(null,
        '\n[等待超时，已解锁输入——检查后端节点 / ollama / 网络]'), 180000);
    }
  }

  // 结束当前挂起气泡：fullText 非空则整体覆盖，extra 追加在末尾
  function chatFinishPending(fullText, extra) {
    if (chatPendingEl) {
      chatPendingEl.classList.remove('pending');
      const t = chatPendingEl.querySelector('.chat-text');
      if (fullText != null) t.textContent = fullText;
      if (extra) t.textContent += extra;
      if (!t.textContent) t.textContent = '(空响应)';
      chatPendingEl = null;
    }
    chatSetBusy(false);
    chatScroll();
  }

  function chatFail(message) {
    if (chatPendingEl) {
      chatPendingEl.classList.remove('pending');
      chatPendingEl.classList.add('error');
      chatPendingEl.querySelector('.chat-role').textContent = '错误';
      chatPendingEl.querySelector('.chat-text').textContent = message;
      chatPendingEl = null;
    } else {
      chatAppend('error', message);
    }
    chatSetBusy(false);
    chatScroll();
  }

  // /chat_response 流式 chunk（topic_server）。仅话题模式消费；没有挂起气泡时
  // （如终端 topic_client 触发的对话）也镜像出来。
  function onChatChunk(msg) {
    if (((chatBackendSel || {}).value || 'service') !== 'topic') return;
    let obj = null;
    try { obj = JSON.parse(msg.data || ''); } catch (_) { return; }
    if (!obj) return;
    if (obj.model) chatSetModel(obj.model);
    if (!chatPendingEl) {
      chatPendingEl = chatAppend('assistant', '', { pending: true });
      chatSetBusy(true);
    }
    if (obj.content) {
      chatPendingEl.querySelector('.chat-text').textContent += obj.content;
      chatScroll();
    }
    if (obj.is_done === true) chatFinishPending(null);
  }

  function chatSendRosService(text) {
    chatSetBusy(true);
    chatPendingEl = chatAppend('assistant', '', { pending: true });
    const srv = new ROSLIB.Service({
      ros, name: '/chat_service', serviceType: 'ollama_ros_msgs/srv/Chat',
    });
    srv.callService(new ROSLIB.ServiceRequest({ content: text }), (res) => {
      if (res && res.model) chatSetModel(res.model);
      chatFinishPending((res && res.content) || '');
    }, (err) => {
      chatFail('调用 /chat_service 失败: ' + err +
        '（chat_service 节点与 ollama 是否在跑？）');
    });
  }

  function chatSendRosTopic(text) {
    if (!chatMsgPub) {
      chatAppend('error', 'rosbridge 未就绪，无法发布 /chat_message');
      return;
    }
    chatSetBusy(true);
    chatPendingEl = chatAppend('assistant', '', { pending: true });
    chatMsgPub.publish(new ROSLIB.Message({
      data: JSON.stringify({ content: text }),
    }));
  }

  async function chatSendDeepseek(text) {
    const key = ((chatDsKey && chatDsKey.value) || '').trim();
    const model = ((chatDsModel && chatDsModel.value) || 'deepseek-chat').trim();
    const base = (((chatDsBase && chatDsBase.value) || 'https://api.deepseek.com').trim())
      .replace(/\/+$/, '');
    if (!key) {
      chatAppend('error', '请先填 DeepSeek API Key（platform.deepseek.com 申请；只保存在本浏览器）');
      return;
    }
    chatDsHistory.push({ role: 'user', content: text });
    // 上下文限长：保住 system，丢最早的对话轮
    while (chatDsHistory.length > 20) chatDsHistory.splice(1, 1);
    chatSetBusy(true);
    chatPendingEl = chatAppend('assistant', '', { pending: true });
    try {
      const resp = await fetch(base + '/chat/completions', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: 'Bearer ' + key,
        },
        body: JSON.stringify({ model, messages: chatDsHistory, stream: true }),
      });
      if (!resp.ok) {
        let detail = '';
        try { detail = (await resp.text()).slice(0, 200); } catch (_) { /* ignore */ }
        throw new Error('HTTP ' + resp.status + (detail ? ' — ' + detail : ''));
      }
      chatSetModel(model);
      // SSE 流：data: {...}\n\n，逐 delta 渲染（deepseek-reasoner 的思维链
      // reasoning_content 不进气泡也不进上下文）
      const reader = resp.body.getReader();
      const decoder = new TextDecoder();
      let buf = '';
      let full = '';
      for (;;) {
        const { done, value } = await reader.read();
        if (done) break;
        buf += decoder.decode(value, { stream: true });
        const lines = buf.split('\n');
        buf = lines.pop();
        for (const raw of lines) {
          const line = raw.trim();
          if (!line.startsWith('data:')) continue;
          const payload = line.slice(5).trim();
          if (!payload || payload === '[DONE]') continue;
          let j = null;
          try { j = JSON.parse(payload); } catch (_) { continue; }
          const delta = j && j.choices && j.choices[0] && j.choices[0].delta;
          const piece = (delta && delta.content) || '';
          if (piece && chatPendingEl) {
            full += piece;
            chatPendingEl.querySelector('.chat-text').textContent += piece;
            chatScroll();
          }
        }
      }
      chatDsHistory.push({ role: 'assistant', content: full });
      chatFinishPending(null);
    } catch (err) {
      chatDsHistory.pop();   // 失败的这轮 user 不留在上下文里
      chatFail('DeepSeek 请求失败: ' + (err && err.message || err) +
        '（检查 Key / 网络 / Base URL；浏览器跨域被拒时可在 Base URL 填自建反代）');
    }
  }

  function chatSend() {
    if (!chatInput || chatBusy) return;
    const text = (chatInput.value || '').trim();
    if (!text) return;
    const backend = ((chatBackendSel || {}).value || 'service');
    chatAppend('user', text);
    chatInput.value = '';
    if (backend === 'deepseek') { chatSendDeepseek(text); return; }
    if (!ros) {
      chatAppend('error', '未连接 rosbridge——先点右上角"连接"，或把后端切成 DeepSeek API');
      return;
    }
    if (backend === 'topic') chatSendRosTopic(text);
    else chatSendRosService(text);
  }

  if (chatSendBtn) chatSendBtn.addEventListener('click', chatSend);
  if (chatInput) {
    chatInput.addEventListener('keydown', (ev) => {
      if (ev.key === 'Enter' && !ev.shiftKey) {
        ev.preventDefault();
        chatSend();
      }
    });
  }
  const chatClearBtn = $('chat-clear');
  if (chatClearBtn) {
    chatClearBtn.addEventListener('click', () => {
      if (chatBox) {
        chatBox.innerHTML = '<div class="chat-empty muted">对话已清空。' +
          '（Ollama 模式的历史在车端节点里，本操作只清前端与 DeepSeek 上下文）</div>';
      }
      chatPendingEl = null;
      chatSetBusy(false);
      chatDsHistory = [CHAT_DS_SYSTEM];
    });
  }
  // 后端选择 + DeepSeek 配置持久化（Key 仅 localStorage，不发往任何后端）
  (function chatRestore() {
    try {
      const saved = {
        backend: localStorage.getItem('chat_backend'),
        key: localStorage.getItem('chat_ds_key'),
        model: localStorage.getItem('chat_ds_model'),
        base: localStorage.getItem('chat_ds_base'),
      };
      if (saved.backend && chatBackendSel) chatBackendSel.value = saved.backend;
      if (saved.key && chatDsKey) chatDsKey.value = saved.key;
      if (saved.model && chatDsModel) chatDsModel.value = saved.model;
      if (saved.base && chatDsBase) chatDsBase.value = saved.base;
    } catch (_) { /* localStorage 不可用就算了 */ }
    if (chatDsCfg && chatBackendSel) chatDsCfg.hidden = chatBackendSel.value !== 'deepseek';
  })();
  if (chatBackendSel) {
    chatBackendSel.addEventListener('change', () => {
      if (chatDsCfg) chatDsCfg.hidden = chatBackendSel.value !== 'deepseek';
      try { localStorage.setItem('chat_backend', chatBackendSel.value); } catch (_) { /* ignore */ }
    });
  }
  [[chatDsKey, 'chat_ds_key'], [chatDsModel, 'chat_ds_model'], [chatDsBase, 'chat_ds_base']]
    .forEach(([el, storeKey]) => {
      if (el) el.addEventListener('change', () => {
        try { localStorage.setItem(storeKey, el.value); } catch (_) { /* ignore */ }
      });
    });

  // ---------- Navigation (top tabs + function sub-tabs) ----------
  // Switching just toggles a CSS class; every card stays in the DOM so all
  // ROS wiring (which looks elements up by id) keeps working whether or not
  // its tab is visible.
  (function initNav() {
    function initTabGroup(opts) {
      const btns = Array.from(document.querySelectorAll(opts.btnSel));
      const storeKey = 'ui_tab_' + opts.key;
      function activate(val) {
        if (!btns.some((b) => b.dataset[opts.key] === val)) return;
        btns.forEach((b) => b.classList.toggle('active', b.dataset[opts.key] === val));
        Array.from(document.querySelectorAll(opts.panelSel))
          .forEach((p) => p.classList.toggle('active', p.id === opts.prefix + val));
        // Chart.js canvases and the three.js viewer size to their container,
        // which reads as 0×0 while the panel is display:none. Nudging a resize
        // once the panel is visible makes them re-measure and fill the space.
        window.dispatchEvent(new Event('resize'));
        // 记住停留位置，刷新后原地恢复
        try { localStorage.setItem(storeKey, val); } catch (_) { /* ignore */ }
        if (opts.onActivate) opts.onActivate(val);
      }
      btns.forEach((b) => b.addEventListener('click', () => {
        activate(b.dataset[opts.key]);
        if (opts.hash) {
          try { history.replaceState(null, '', '#' + b.dataset[opts.key]); } catch (_) { /* ignore */ }
        }
      }));
      // 刷新后回到上次停留的页签（顶层 tab 的 #hash 深链在调用方随后
      // 覆盖，优先级更高；存量值无效时 activate 自己会拒绝）
      try {
        const saved = localStorage.getItem(storeKey);
        if (saved) activate(saved);
      } catch (_) { /* ignore */ }
      return activate;
    }

    // 子页/页签激活时惰性启动该页可见的 MJPEG 流（kickVisibleFnStreams：
    // 只拉还停在占位图的流，不打断已在播放的）。底盘功能模块与机械臂
    // 各自独立一组 tab（类名/data 键不同，互不干扰），行为统一。
    initTabGroup({
      btnSel: '.subtab-btn', key: 'subtab', panelSel: '.subtab-panel', prefix: 'subpanel-',
      onActivate: (val) => kickVisibleFnStreams($('subpanel-' + val)),
    });

    initTabGroup({
      btnSel: '.map-subtab-btn', key: 'mapsubtab', panelSel: '.map-subtab-panel', prefix: 'map-subpanel-',
      onActivate: (val) => kickVisibleFnStreams($('map-subpanel-' + val)),
    });

    initTabGroup({
      btnSel: '.arm-subtab-btn', key: 'armsubtab', panelSel: '.arm-subtab-panel', prefix: 'arm-subpanel-',
      onActivate: (val) => kickVisibleFnStreams($('arm-subpanel-' + val)),
    });

    const activateTop = initTabGroup({
      btnSel: '.tab-btn', key: 'tab', panelSel: '.tab-panel', prefix: 'panel-', hash: true,
      // 不对某个页签做特例：任何顶层面板激活，都把它里面可见的流启动起来
      // （隐藏子页里的流等切到对应子页时再由上面两组的 onActivate 启动）。
      onActivate: (val) => kickVisibleFnStreams($('panel-' + val)),
    });

    // Allow deep-linking to a top tab via #hash (e.g. .../#control).
    const initial = (location.hash || '').replace(/^#/, '');
    if (initial) activateTop(initial);

    // 系统总览"系统架构"卡：点击模块芯片跳到对应页签（及功能/机械臂子页）。
    document.querySelectorAll('.arch-link').forEach((el) => {
      el.addEventListener('click', () => {
        const tabBtn = document.querySelector(`.tab-btn[data-tab="${el.dataset.tab}"]`);
        if (tabBtn) tabBtn.click();
        if (el.dataset.subtab) {
          const b = document.querySelector(`.subtab-btn[data-subtab="${el.dataset.subtab}"]`);
          if (b) b.click();
        }
        if (el.dataset.mapsubtab) {
          const b = document.querySelector(`.map-subtab-btn[data-mapsubtab="${el.dataset.mapsubtab}"]`);
          if (b) b.click();
        }
        if (el.dataset.armsubtab) {
          const b = document.querySelector(`.arm-subtab-btn[data-armsubtab="${el.dataset.armsubtab}"]`);
          if (b) b.click();
        }
      });
    });
  })();

  // Auto-connect on load（按自动流程：rosbridge 还没起来就退避重试）.
  connect(true);
})();
