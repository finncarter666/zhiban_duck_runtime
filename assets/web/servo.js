const $ = id => document.getElementById(id);
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
let active = false, busy = false, selectedId = null, scanTask = null, scanController = null, closing = false;
const editIds = ['kp', 'kd', 'new-id', 'reset-mid', 'set-gains', 'set-id', 'read'];
const protectionIds = ['protection-current', 'enable-protection', 'disable-protection', 'set-protection-current'];
async function api(path, body, signal) {
    const timeout = AbortSignal.timeout(4000);
    const options = { signal: signal ? AbortSignal.any([signal, timeout]) : timeout };
    if (body !== undefined)
        Object.assign(options, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const r = await fetch('/api/' + path, options);
    const x = await r.json();
    if (!r.ok)
        throw new Error(x.detail || '请求失败');
    if (path === 'status')
        Studio.update(x);
    return x;
}
function show(msg) {
    Studio.clearError();
    Studio.text('message', msg);
}
function lock(value) {
    busy = value;
    editIds.forEach(id => $(id).disabled = value || selectedId === null);
    protectionIds.forEach(id => $(id).disabled = value);
    $('scan').disabled = value;
    $('scan-results').querySelectorAll('button').forEach(b => b.disabled = value);
    $('exit').disabled = value && !scanController;
}
function selectId(id) {
    selectedId = validId(id);
    Studio.text('selected-servo', 'ID ' + selectedId);
    Studio.text('read-out', '—');
    $('scan-results').querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', String(+b.dataset.id === selectedId)));
    lock(false);
    show('已选择 ID ' + selectedId);
    send('read');
}
function validId(value) {
    const id = Number(value);
    if (String(value).trim() === '' || !Number.isInteger(id) || id < 0 || id > 253)
        throw new Error('ID 必须是 0–253 之间的整数');
    return id;
}
async function enter(signal) {
    await api('servo/enter', {}, signal);
    const deadline = Date.now() + 1500;
    while (Date.now() < deadline) {
        const s = await api('status', undefined, signal);
        if (s.mode === 'servo_debug' && s.hardware_servo_debug && s.feedback_age_ms != null && s.feedback_age_ms < 500) {
            active = true;
            return;
        }
        await sleep(100);
    }
    throw new Error('没有有效硬件反馈，舵机维护尚未就绪。');
}
function stopScan() {
    if (scanController) {
        Studio.text('scan-status', '正在停止扫描…');
        scanController.abort();
    }
}
async function exit() {
    if (closing)
        return;
    closing = true;
    stopScan();
    if (scanTask)
        await scanTask;
    lock(true);
    try {
        await api('servo/exit', {});
        active = false;
        show('已退出舵机维护，全部舵机已卸力');
    }
    catch (e) {
        Studio.messageError(e);
    }
    finally {
        lock(false);
        closing = false;
    }
}
async function send(action, extra = {}) {
    if (busy || closing || selectedId === null)
        return;
    lock(true);
    try {
        const id = selectedId;
        if (action === 'set_id')
            extra.new_id = validId(extra.new_id);
        await enter();
        show(action === 'set_id' ? `正在将 ID ${id} 改为 ${extra.new_id}…` : `ID ${id} · ${action === 'reset_mid' ? '复位中位' : action === 'set_gains' ? '写入增益' : '读取参数'}…`);
        const r = await api('servo', { action, target_id: id, ...extra });
        if (action === 'read') {
            Studio.text('read-out', `ID ${id} · 编码器 ${r.raw_pos} · KP ${r.kp} · KD ${r.kd}`);
            show(`ID ${id} · 参数读取完成`);
        }
        else if (action === 'reset_mid') {
            Studio.text('read-out', '—');
            show(`ID ${id} · 已发送中位复位命令，舵机保持卸力。`);
        }
        else if (action === 'set_id') {
            selectedId = null;
            Studio.text('selected-servo', '未选择舵机');
            Studio.text('read-out', '—');
            $('scan-results').replaceChildren();
            $('scan-progress').hidden = true;
            Studio.text('scan-status', 'ID 已修改，请重新扫描并选择新的 ID。');
            show(`ID ${id} → ${extra.new_id} · 已验证`);
        }
        else {
            show(`ID ${id} · KP / KD 已写入`);
        }
    }
    catch (e) {
        Studio.messageError(e);
    }
    finally {
        lock(false);
    }
}
async function scan() {
    scanController = new AbortController();
    const signal = scanController.signal;
    selectedId = null;
    Studio.text('selected-servo', '未选择舵机');
    Studio.text('read-out', '—');
    lock(true);
    const found = [];
    let checked = 0;
    $('scan-results').replaceChildren();
    $('scan-progress').value = 0;
    $('scan-progress').hidden = false;
    $('scan-stop').hidden = false;
    Studio.text('scan-status', '正在准备扫描…');
    try {
        const s = await api('status', undefined, signal);
        if (s.servo_scan_version !== 2)
            throw new Error('请部署支持当前扫描协议的后端版本。');
        await enter(signal);
        await api('servo', { action: 'torque_off', target_id: 10 }, signal);
        show('扫描进行中 · 全部舵机已卸力');
        for (let id = 0; id <= 253; id++) {
            signal.throwIfAborted();
            Studio.text('scan-status', `正在扫描 ID ${id} · 已检查 ${checked}/254 · 发现 ${found.length} 个`);
            let online = false;
            for (let attempt = 0; attempt < 2; attempt++) {
                await api('servo/enter', {}, signal);
                signal.throwIfAborted();
                const r = await api('servo', { action: 'ping', target_id: id }, signal);
                if (r.op !== 7 || r.target_id !== id || typeof r.ok !== 'boolean')
                    throw new Error('扫描回复格式无效');
                if (r.ok) {
                    online = true;
                    break;
                }
            }
            if (online) {
                found.push(id);
                const b = document.createElement('button');
                b.textContent = `ID ${id}`;
                b.dataset.id = id;
                b.disabled = true;
                b.setAttribute('aria-pressed', 'false');
                b.onclick = () => selectId(id);
                $('scan-results').appendChild(b);
            }
            checked++;
            $('scan-progress').value = checked;
        }
        Studio.text('scan-status', `扫描完成 · 已检查 254 个 ID · 发现 ${found.length} 个`);
        show(found.length ? '请选择已发现的 ID，读取或修改参数。' : '没有舵机回复，请检查供电、总线连接和波特率。');
    }
    catch (e) {
        Studio.text('scan-status', `${signal.aborted ? '已停止' : '扫描失败'} · 已检查 ${checked}/254 · 发现 ${found.length} 个（部分结果）`);
        show(signal.aborted ? '扫描已停止，可以选择已发现的 ID 继续操作。' : e.message === 'servo reply timeout' ? '硬件无回复，扫描已停止。' : e.message);
    }
    finally {
        $('scan-stop').hidden = true;
        scanController = null;
        lock(false);
    }
}
async function sendProtection(action, enabled) {
    if (busy || closing)
        return;
    lock(true);
    try {
        const settingCurrent = action === 'set_protection_current', body = { action };
        if (settingCurrent) {
            const input = $('protection-current').value.trim(), current = Number(input);
            if (!input || !Number.isFinite(current) || current < 0 || current > 65535 * .0065)
                throw new Error('请输入 0–425.9775 A 之间的保护电流。该范围为寄存器范围，不代表设备安全电流。');
            body.current_a = current;
        }
        else {
            body.protection_enabled = enabled;
        }
        await enter();
        show('正在向全部舵机广播保护设置…');
        const r = await api('servo', body);
        if (!r.ok || r.broadcast !== true || r.target_id !== 254 || r.op !== (settingCurrent ? 10 : 9))
            throw new Error('广播回复格式无效');
        if (settingCurrent) {
            Studio.text('protection-current-out', `广播已发送 · ${r.current_a.toFixed(4)} A`);
        }
        else {
            Studio.text('protection-out', `广播已发送 · 电压 / 过流保护${r.protection_enabled ? '开启' : '关闭'}`);
        }
        show('保护设置广播已发送，未逐个验证舵机写入结果。');
    }
    catch (e) {
        Studio.messageError(e);
    }
    finally {
        lock(false);
    }
}
$('scan').onclick = () => {
    if (busy || scanTask || closing)
        return;
    scanTask = scan().finally(() => {
        scanTask = null;
    });
};
$('scan-stop').onclick = stopScan;
$('exit').onclick = exit;
$('reset-mid').onclick = () => send('reset_mid');
$('set-gains').onclick = () => send('set_gains', { kp: +$('kp').value, kd: +$('kd').value });
$('set-id').onclick = () => send('set_id', { new_id: $('new-id').value });
$('read').onclick = () => send('read');
$('enable-protection').onclick = () => sendProtection('set_protection', true);
$('disable-protection').onclick = () => sendProtection('set_protection', false);
$('set-protection-current').onclick = () => sendProtection('set_protection_current');
window.addEventListener('pagehide', () => {
    stopScan();
    navigator.sendBeacon('/api/servo/exit', new Blob(['{}'], { type: 'application/json' }));
});
document.addEventListener('visibilitychange', () => {
    if (document.hidden && (active || scanTask))
        exit();
});
let statusBusy = false;
setInterval(async () => {
    if (document.hidden || busy || closing || statusBusy)
        return;
    statusBusy = true;
    try {
        const s = await api('status');
        if (busy || closing)
            return;
        active = s.mode === 'servo_debug';
        if (active)
            await api('servo/enter', {});
    }
    catch (e) {
        Studio.unavailable();
        Studio.messageError(e);
    }
    finally {
        statusBusy = false;
    }
}, 1000);
lock(false);
