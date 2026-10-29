/* 共享展示层只消费现有状态，不产生控制命令或额外的状态轮询。 */
(() => {
    const text = (id, value) => {
        const element = document.getElementById(id);
        if (element && element.textContent !== String(value))
            element.textContent = value;
    };
    const modes = {
        shadow: '仅推理', off: '已卸力', hold: '默认姿态',
        policy: '策略控制', calibrate: '标零中', servo_debug: '舵机维护'
    };
    let lastUpdate = 0;
    function connection(state, label) {
        document.getElementById('connection').dataset.state = state;
        text('connection-label', label);
    }
    function unavailable(label = '连接中断') {
        connection('offline', label);
        ['rx', 'rl', 'servos', 'imu', 'tilt-angle', 'active-model', 'recovery-timer',
            'acc-values', 'gyro-values', 'gravity-values'].forEach(id => text(id, '—'));
        text('servo-summary', '等待重新连接');
        text('feedback-age', '当前数据不可用');
        text('hero-mode', '设备状态不可用');
        text('mode', '状态未知 · 请确认设备');
        text('recovery-phase', '—');
        text('command-health', '连接中断，命令状态未知');
        document.querySelectorAll('[data-mode="hold"], [data-mode="policy"], #pick, #roulade, [data-center]')
            .forEach(button => {
            button.disabled = true;
        });
        if (document.body.dataset.page === 'overview') {
            joints(null);
        }
    }
    const jointRows = new Map();
    function joints(status) {
        const table = document.getElementById('joints');
        if (!table || document.body.dataset.page !== 'overview')
            return;
        if (status?.angles) {
            if (!jointRows.size)
                table.replaceChildren();
            status.motors.forEach((id, i) => {
                let row = jointRows.get(id);
                if (!row) {
                    row = document.createElement('tr');
                    row.innerHTML = '<td></td><td></td><td></td><td><span class="joint-status"></span></td>';
                    row.cells[0].textContent = id;
                    jointRows.set(id, row);
                    table.append(row);
                }
                const online = status.servo_ids.includes(id);
                row.classList.toggle('missing', !online);
                [status.angles[i].toFixed(1), status.targets[i].toFixed(1), online ? '在线' : '离线'].forEach((value, column) => {
                    const cell = column === 2 ? row.cells[3].firstElementChild : row.cells[column + 1];
                    if (cell.textContent !== value)
                        cell.textContent = value;
                });
            });
        } else {
            jointRows.forEach(row => {
                row.classList.add('missing');
                row.cells[1].textContent = row.cells[2].textContent = '—';
                row.cells[3].firstElementChild.textContent = '未知';
            });
        }
    }
    function update(status) {
        lastUpdate = Date.now();
        const age = status.feedback_age_ms;
        const fresh = Number.isFinite(age) && age < 150;
        connection(fresh ? 'online' : 'stale', fresh ? '设备已连接' : '服务在线 · 反馈异常');
        text('hero-mode', (modes[status.mode] || status.mode) + (status.enabled ? ' · 扭矩已使能' : ' · 扭矩未使能'));
        text('feedback-age', Number.isFinite(age) ? `反馈时延 ${age.toFixed(1)} ms` : '尚无硬件反馈');
        const missing = status.missing_joint_ids;
        text('servo-summary', missing?.length ? `缺失 ID：${missing.join(' / ')}` : status.servo_ids?.length === 15 ? '全部关节在线' : '等待关节反馈');
        document.querySelectorAll('[data-mode]').forEach(button => button.setAttribute('aria-pressed', String(button.dataset.mode === status.mode)));
        const completed = (status.cal_done || []).filter(Boolean).length;
        text('cal-completed', completed);
        const progress = document.getElementById('cal-progress');
        if (progress)
            progress.value = completed;
        const selected = document.getElementById('scan-results')?.querySelector('[aria-pressed="true"]');
        text('selected-servo', selected ? `ID ${selected.dataset.id}` : '未选择舵机');
    }
    function messageError(error) {
        text('message', error.message || String(error));
        document.querySelector('.message-bar').dataset.error = 'true';
    }
    function clearError() {
        document.querySelector('.message-bar').dataset.error = 'false';
    }
    function markNavigation() {
        if (document.body.dataset.page !== 'overview')
            return;
        const target = location.hash === '#motion' ? '/#motion' : '/';
        document.querySelectorAll('.nav-item').forEach(link => {
            const active = link.getAttribute('href') === target;
            link.classList.toggle('active', active);
            if (active)
                link.setAttribute('aria-current', target === '/' ? 'page' : 'location');
            else
                link.removeAttribute('aria-current');
        });
    }
    window.addEventListener('hashchange', markNavigation);
    markNavigation();
    document.querySelectorAll('.nav-item').forEach(link => {
        link.setAttribute('aria-label', link.textContent.trim());
    });
    // 浏览器休眠或请求悬挂时，不让最后一次状态持续显示成“在线”。
    setInterval(() => {
        if (lastUpdate && Date.now() - lastUpdate > 3500) {
            const scanButton = document.getElementById('scan-stop');
            if (scanButton && !scanButton.hidden)
                connection('stale', '扫描中 · 状态刷新暂停');
            else
                unavailable();
        }
    }, 1000);
    window.Studio = { update, unavailable, messageError, clearError, text, joints };
})();
