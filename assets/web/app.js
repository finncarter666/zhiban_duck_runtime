const $ = id => document.getElementById(id);
let statusBusy = false, commandBusy = false, commandPending = false, rollerBusy = false;
let rollerEnabled = false;
const motionRanges = {
    walk: { backward: .2, strafe: .3, yaw: 1, keyboard: [.2, .15, .5] },
    roller: { backward: .4, strafe: 0, yaw: .25, keyboard: [.4, 0, .25] }
};
for (let i = 0; i < 4; i++)
    $('head-controls').insertAdjacentHTML('beforeend', `<label>头部通道 ${i + 1} <span id="head${i}-value">0.00 rad</span><input id="head${i}" type="range" min="-1" max="1" value="0" step="0.02"></label>`);
$('head-controls').insertAdjacentHTML('beforeend', '<label>嘴部 · ID 34 <span id="mouth-value">0°</span><input id="mouth" type="range" min="0" max="30" value="0" step="1"></label>');
['head0', 'head1', 'head2', 'head3', 'mouth'].forEach(id => $(id).addEventListener('input', () => Studio.text(id + '-value', id === 'mouth' ? $(id).value + '°' : (+$(id).value).toFixed(2) + ' rad')));
async function post(path, body) {
    const r = await fetch('/api/' + path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const x = await r.json();
    if (!r.ok)
        throw new Error(x.detail || '请求失败');
    return x;
}
function showError(e) {
    Studio.messageError(e);
}
async function setMode(mode) {
    try {
        await post('mode', { mode });
        Studio.text('message', '控制模式已更新');
    }
    catch (e) {
        showError(e);
    }
}
document.querySelectorAll('[data-mode]').forEach(b => b.onclick = () => setMode(b.dataset.mode));
function showRoller(enabled) {
    const changed = rollerEnabled !== enabled;
    rollerEnabled = enabled;
    $('roller-mode').checked = enabled;
    Studio.text('roller-state', enabled ? '开启' : '关闭');
    $('roller-warning').hidden = !enabled;
    Studio.text('action-profile', enabled ? '滚轮行走' : '行走 / 其他模型');
    const limits = motionRanges[enabled ? 'roller' : 'walk'];
    Studio.text('motion-limits', `范围：vx -${limits.backward}～+0.4，vy ${limits.strafe ? '±' + limits.strafe : '0'}，ωz ±${limits.yaw}。`);
    Studio.text('left-label', enabled ? '前进 / 后退' : '前进 / 横移');
    if (changed)
        zero(false);
}
$('roller-mode').onchange = async () => {
    if (rollerBusy)
        return;
    const enabled = $('roller-mode').checked;
    showRoller(enabled);
    rollerBusy = true;
    $('roller-mode').disabled = true;
    try {
        const r = await post('roller', { enabled });
        showRoller(r.enabled);
        showActionSettings(r);
        Studio.text('message', `滚轮模式${r.enabled ? '已开启' : '已关闭'} · ${r.model}`);
    }
    catch (e) {
        showRoller(!enabled);
        showError(e);
    }
    finally {
        rollerBusy = false;
        $('roller-mode').disabled = false;
    }
};
$('pick').onclick = async () => {
    try {
        const r = await post('pick', {});
        Studio.text('message', `拾取动作已启动 · ${r.model}`);
    }
    catch (e) {
        showError(e);
    }
};
$('roulade').onclick = async () => {
    try {
        const r = await post('roulade', {});
        Studio.text('message', `翻滚动作已启动 · ${r.model}`);
    }
    catch (e) {
        showError(e);
    }
};
$('stop').onclick = () => {
    zero();
    setMode('off');
};
const keys = new Set(), sticks = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } }, resetSticks = [];
let motion = [0, 0, 0], targetMotion = [0, 0, 0];
let touchArmed = false;
function armTouch(armed) {
    touchArmed = armed;
    document.querySelectorAll('.joy').forEach(zone => zone.classList.toggle('touch-armed', armed));
    $('touch-control').setAttribute('aria-pressed', String(armed));
    Studio.text('touch-control', armed ? '取消摇杆触控' : '启用一次摇杆触控');
}
$('touch-control').onclick = () => {
    const next = !touchArmed;
    zero();
    armTouch(next);
};
const deadzone = v => Math.abs(v) <= .15 ? 0 : v;
function updateMotion() {
    const key = (a, b) => keys.has(a) - keys.has(b);
    const limits = motionRanges[rollerEnabled ? 'roller' : 'walk'];
    const forward = deadzone(sticks.left.y);
    targetMotion = [keys.has('w') || keys.has('s') ? key('w', 's') * limits.keyboard[0] : forward * (forward < 0 ? limits.backward : .4),
        keys.has('a') || keys.has('d') ? key('a', 'd') * limits.keyboard[1] : deadzone(-sticks.left.x) * limits.strafe,
        keys.has('q') || keys.has('e') ? key('q', 'e') * limits.keyboard[2] : deadzone(-sticks.right.x) * limits.yaw];
}
function showMotion() {
    ['vx', 'vy', 'wz'].forEach((id, i) => Studio.text(id + 'Value', motion[i].toFixed(2)));
}
async function sendCommand() {
    if (commandBusy) {
        commandPending = true;
        return;
    }
    commandBusy = true;
    commandPending = false;
    motion = motion.map((old, i) => .3 * old + .7 * targetMotion[i]);
    showMotion();
    try {
        await post('command', { twist: [...motion], head: [0, 1, 2, 3].map(i => +$('head' + i).value), mouth: +$('mouth').value });
    }
    catch (e) {
        showError(e);
    }
    finally {
        commandBusy = false;
        if (commandPending)
            sendCommand();
    }
}
function joystick(id, side) {
    const zone = $(id), knob = zone.firstElementChild;
    let pointer = null;
    function reset() {
        const old = pointer;
        pointer = null;
        sticks[side] = { x: 0, y: 0 };
        knob.style.transform = 'translate(0px,0px)';
        if (old !== null && zone.hasPointerCapture(old))
            zone.releasePointerCapture(old);
    }
    resetSticks.push(reset);
    function move(e) {
        const b = zone.getBoundingClientRect(), radius = (zone.clientWidth - knob.offsetWidth) / 2;
        let dx = e.clientX - b.left - b.width / 2, dy = e.clientY - b.top - b.height / 2;
        const distance = Math.hypot(dx, dy);
        if (distance > radius) {
            dx *= radius / distance;
            dy *= radius / distance;
        }
        knob.style.transform = `translate(${dx}px,${dy}px)`;
        sticks[side] = { x: dx / radius, y: -dy / radius };
        updateMotion();
    }
    zone.onpointerdown = e => {
        if (pointer !== null || e.button !== 0 || (e.pointerType !== 'mouse' && !touchArmed))
            return;
        e.preventDefault();
        pointer = e.pointerId;
        zone.setPointerCapture(pointer);
        move(e);
        sendCommand();
    };
    zone.onpointermove = e => {
        if (e.pointerId === pointer)
            move(e);
    };
    const release = e => {
        if (e.pointerId !== pointer)
            return;
        reset();
        armTouch(false);
        updateMotion();
        // 立即归零已释放的轴，另一个摇杆或键盘仍可独立操作。
        motion = motion.map((value, i) => targetMotion[i] === 0 ? 0 : value);
        sendCommand();
    };
    zone.onpointerup = zone.onpointercancel = zone.onlostpointercapture = release;
}
joystick('joy-left', 'left');
joystick('joy-right', 'right');
function zero(send = true) {
    armTouch(false);
    keys.clear();
    resetSticks.forEach(reset => reset());
    motion = [0, 0, 0];
    updateMotion();
    showMotion();
    if (send)
        sendCommand();
}
$('zero').onclick = () => zero();
// 触摸滑块先判断方向：纵向交给浏览器滚动，横向拖动才改变参数。
// 鼠标点击和键盘调节沿用浏览器原生行为。
document.querySelectorAll('input[type="range"]').forEach(input => {
    let gesture = null, touchSession = false, touchValue = input.value;
    // 原生 range 可在手势方向尚未确定时改变值；拦截这类事件，
    // 仅允许下方横向拖动产生的值进入显示与控制请求。
    ['input', 'change'].forEach(type => input.addEventListener(type, event => {
        if (touchSession && event.isTrusted) {
            input.value = touchValue;
            event.stopImmediatePropagation();
        }
    }, true));
    input.addEventListener('keydown', () => { touchSession = false; });
    const finish = event => {
        if (!gesture || event.pointerId !== gesture.id)
            return;
        const changed = gesture.active && input.value !== gesture.value;
        const cancelled = event.type !== 'pointerup';
        const original = gesture.value;
        gesture = null;
        if (cancelled) {
            input.value = touchValue = original;
            input.dispatchEvent(new Event('input', { bubbles: true }));
        }
        else if (changed) {
            input.dispatchEvent(new Event('change', { bubbles: true }));
        }
    };
    input.addEventListener('pointerdown', event => {
        if (event.pointerType === 'mouse') {
            touchSession = false;
            return;
        }
        if (!event.isPrimary)
            return;
        touchSession = true;
        touchValue = input.value;
        event.preventDefault();
        gesture = { id: event.pointerId, x: event.clientX, y: event.clientY, value: input.value, active: false, scrolling: false };
        input.setPointerCapture(event.pointerId);
    });
    input.addEventListener('pointermove', event => {
        if (!gesture || event.pointerId !== gesture.id || gesture.scrolling)
            return;
        const dx = event.clientX - gesture.x, dy = event.clientY - gesture.y;
        if (!gesture.active) {
            if (Math.abs(dy) > 8 && Math.abs(dy) >= Math.abs(dx)) {
                gesture.scrolling = true;
                return;
            }
            if (Math.abs(dx) < 10 || Math.abs(dx) <= Math.abs(dy) * 1.5)
                return;
            gesture.active = true;
            input.focus({ preventScroll: true });
        }
        const bounds = input.getBoundingClientRect();
        const step = +input.step || 1, min = +input.min, max = +input.max;
        const value = +gesture.value + dx / bounds.width * (max - min);
        input.value = Math.max(min, Math.min(max, min + Math.round((value - min) / step) * step));
        touchValue = input.value;
        input.dispatchEvent(new Event('input', { bubbles: true }));
    });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(type => input.addEventListener(type, finish));
});
['leg-alpha', 'head-alpha'].forEach(id => $(id).onchange = () => post('filter', { leg: +$('leg-alpha').value, head: +$('head-alpha').value }).catch(showError));
const actionInputs = [['action-scale', 'action_scale'], ['leg-alpha', 'action_alpha'], ['head-alpha', 'action_alpha_head']];
actionInputs.forEach(([id]) => $(id).oninput = () => Studio.text(id + '-value', (+$(id).value).toFixed(2)));
function showActionSettings(settings) {
    actionInputs.forEach(([id, key]) => {
        if (document.activeElement !== $(id)) {
            $(id).value = settings[key];
            Studio.text(id + '-value', (+$(id).value).toFixed(2));
        }
    });
}
$('action-scale').onchange = () => post('action-scale', { scale: +$('action-scale').value }).catch(showError);
const imuFilters = [['gyro-alpha', 'gyro_alpha'], ['gravity-tau', 'gravity_tau_s']];
imuFilters.forEach(([id, key]) => {
    const input = $(id), output = $(id + '-value');
    input.oninput = () => output.textContent = (+input.value).toFixed(2);
    input.onchange = () => post('imu-filter', { [key]: +input.value }).catch(showError);
});
function keyCommand() {
    updateMotion();
    sendCommand();
}
window.addEventListener('keydown', e => {
    if (e.code === 'Space') {
        e.preventDefault();
        $('stop').click();
        return;
    }
    const k = e.key.toLowerCase();
    if ('wasdqe'.includes(k) && k.length === 1) {
        keys.add(k);
        keyCommand();
    }
});
window.addEventListener('keyup', e => {
    const k = e.key.toLowerCase();
    if (keys.delete(k))
        keyCommand();
});
window.addEventListener('blur', () => zero());
document.addEventListener('visibilitychange', () => {
    if (document.hidden)
        zero();
});
window.addEventListener('pagehide', () => {
    zero(false);
    navigator.sendBeacon('/api/command', new Blob([JSON.stringify({ twist: [0, 0, 0] })], { type: 'application/json' }));
});
setInterval(() => {
    if (!document.hidden && !commandBusy)
        sendCommand();
}, 50);
setInterval(async () => {
    if (statusBusy || document.hidden)
        return;
    statusBusy = true;
    try {
        const r = await fetch('/api/status');
        if (!r.ok)
            throw new Error('状态读取失败');
        const s = await r.json();
        Studio.clearError();
        Studio.update(s);
        const modeLabel = { shadow: '仅推理', off: '已卸力', hold: '默认姿态', policy: '行走', calibrate: '标零', servo_debug: '舵机维护' };
        Studio.text('rx', s.feedback_hz.toFixed(1) + ' Hz');
        Studio.text('rl', s.inference_hz.toFixed(1) + ' Hz');
        Studio.text('servos', (s.servo_ids?.length || 0) + '/15');
        Studio.text('imu', s.imu_ok ? '正常' : '无有效反馈');
        Studio.text('mode', (modeLabel[s.mode] || s.mode) + (s.enabled ? ' · 扭矩已使能' : ' · 扭矩未使能'));
        Studio.text('message', s.last_error || `Linux 本机 I/O · ${s.feedback_age_ms == null ? '等待硬件反馈' : '反馈时延 ' + s.feedback_age_ms.toFixed(1) + ' ms'}`);
        document.querySelectorAll('[data-mode="hold"],[data-mode="policy"]').forEach(b => b.disabled = !s.control_ready);
        if (s.roller && !rollerBusy)
            showRoller(s.roller.enabled);
        if (!rollerBusy)
            showActionSettings(s.deployment);
        imuFilters.forEach(([id, key]) => {
            if (document.activeElement !== $(id)) {
                $(id).value = s.deployment[key];
                Studio.text(id + '-value', (+$(id).value).toFixed(2));
            }
        });
        const recovery = s.recovery, pick = s.pick, roulade = s.roulade;
        const upright = (s.mode === 'shadow' || s.mode === 'policy') && recovery && recovery.active && recovery.phase === 'walk';
        $('pick').disabled = !upright || (pick && pick.active) || (roulade && roulade.active);
        $('roulade').disabled = $('pick').disabled;
        if (recovery) {
            const phaseLabel = roulade && roulade.active ? '翻滚' : pick && pick.active ? '拾取' : ({ walk: '行走', default_pose: '起身 · 保持默认姿态', getup: '起身' }[recovery.phase] || recovery.phase);
            Studio.text('recovery-phase', recovery.active ? phaseLabel : '未激活');
            Studio.text('active-model', s.active_model || '—');
            Studio.text('tilt-angle', recovery.angle_deg == null ? '—' : recovery.angle_deg.toFixed(1) + '°');
            Studio.text('recovery-timer', !recovery.active ? '—' : roulade && roulade.active ? `翻滚剩余 ${roulade.remaining_s.toFixed(2)} / ${roulade.seconds.toFixed(2)} s` : pick && pick.active ? `拾取 φ ${pick.phi.toFixed(2)} / 1.00` : recovery.phase === 'default_pose' ? `保持剩余 ${recovery.hold_remaining_s.toFixed(2)} s` : recovery.phase === 'getup' ? `站立 ${recovery.stand_elapsed_s.toFixed(2)} / 1.00 s` : `倾倒 ${recovery.fall_elapsed_s.toFixed(2)} / 0.15 s`);
            if (pick && pick.active) {
                $('mouth').value = Math.round(s.mouth || 0);
                Studio.text('mouth-value', $('mouth').value + '°');
            }
        }
        for (const [id, values] of [['acc-values', s.acc], ['gyro-values', s.gyro], ['gravity-values', s.gravity]])
            Studio.text(id, values ? values.map(x => x.toFixed(3)).join(' / ') : '—');
        Studio.text('command-health', `已发送 ${s.sent} · 硬件回执 ${s.hardware_command_seq ?? '—'} · 命令时延 ${s.hardware_command_age_us < 1e9 ? (s.hardware_command_age_us / 1000).toFixed(1) + ' ms' : '—'} · 序号丢帧 ${s.sequence_gaps} · 拒绝 ${s.hardware_invalid_commands ?? 0}`);
        Studio.joints(s);
    }
    catch (e) {
        Studio.unavailable();
        showError(e);
    }
    finally {
        statusBusy = false;
    }
}, 250);
