const $ = id => document.getElementById(id);
const MOTORS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34];
let statusBusy = false, active = false, centering = null, presentMask = 0;
let held = Array(15).fill(false);
async function post(path, body = {}) {
    const r = await fetch('/api/' + path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const x = await r.json().catch(() => ({}));
    if (!r.ok)
        throw new Error(x.detail || '请求失败');
    return x;
}
function showError(e) {
    Studio.messageError(e);
}
MOTORS.forEach((id, i) => {
    $('joints').insertAdjacentHTML('beforeend', `<div class="cal-row ${i === 5 || i === 10 ? 'group-start' : ''}" id="row${i}">
    <div class="cal-meta"><b>ID ${id}</b><span id="raw${i}">等待编码器反馈</span></div>
    <div class="cal-actions"><span id="state${i}">已卸力</span><button data-center="${i}" disabled>设为中位并保持</button></div>
  </div>`);
});
function refreshRows() {
    document.querySelectorAll('[data-center]').forEach(btn => {
        const i = +btn.dataset.center;
        btn.disabled = !active || centering !== null || held[i] || !(presentMask & (1 << i));
        $(`row${i}`).classList.toggle('done', held[i]);
        Studio.text(`state${i}`, centering === i ? '正在标零…' : held[i] ? '保持 2048' : '已卸力');
    });
}
document.querySelectorAll('[data-center]').forEach(btn => {
    btn.onclick = async () => {
        const i = +btn.dataset.center;
        centering = i;
        refreshRows();
        Studio.text('message', `正在设置 ID ${MOTORS[i]} 的中位…`);
        try {
            await post('cal/center', { index: i });
            if (active) {
                held[i] = true;
                Studio.text('message', `ID ${MOTORS[i]} 中位已设置，目标 2048 与扭矩使能已读回确认。请检查实际保持情况。`);
            }
        }
        catch (e) {
            showError(e);
        }
        finally {
            centering = null;
            refreshRows();
        }
    };
});
async function enter() {
    try {
        await post('cal/enter');
        active = true;
        held.fill(false);
        Studio.text('message', '全部舵机已卸力。请手动对齐一个关节，再设为中位并保持。');
    }
    catch (e) {
        showError(e);
    }
    refreshRows();
}
async function exit() {
    active = false;
    refreshRows();
    try {
        await post('cal/exit');
        held.fill(false);
        Studio.text('message', '已退出标零，全部舵机已卸力。');
    }
    catch (e) {
        showError(e);
    }
    refreshRows();
}
$('start').onclick = enter;
$('exit').onclick = exit;
$('exit2').onclick = exit;
window.addEventListener('pagehide', () => navigator.sendBeacon('/api/cal/exit', new Blob(['{}'], { type: 'application/json' })));
document.addEventListener('visibilitychange', () => {
    if (document.hidden && active)
        exit();
});
setInterval(async () => {
    if (statusBusy || document.hidden)
        return;
    statusBusy = true;
    try {
        if (active)
            await post('cal/heartbeat').catch(() => {
            });
        const r = await fetch('/api/status');
        if (!r.ok)
            throw new Error('状态读取失败');
        const s = await r.json();
        Studio.clearError();
        Studio.update(s);
        active = s.mode === 'calibrate';
        presentMask = s.raw_mask || 0;
        held = s.cal_done || Array(15).fill(false);
        Studio.text('mode', active ? '标零进行中' : '标零已关闭');
        if (s.last_error)
            Studio.text('message', s.last_error);
        if (s.raw_pos)
            s.raw_pos.forEach((v, i) => {
                Studio.text(`raw${i}`, presentMask & (1 << i) ? '编码器 ' + v : '无舵机反馈');
            });
        refreshRows();
    }
    catch (e) {
        Studio.unavailable();
        showError(e);
        document.querySelectorAll('[data-center]').forEach(button => button.disabled = true);
    }
    finally {
        statusBusy = false;
    }
}, 250);
enter();
