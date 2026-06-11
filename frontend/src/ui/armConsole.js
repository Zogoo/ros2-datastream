import { TOPICS } from '../ros/topics.js';

const el = (id) => document.getElementById(id);

/** Serial-protocol console: sends raw firmware lines to /arm/command or
 *  /base/command and logs /arm/response + /base/response. */
export class ArmConsole {
  constructor(ros) {
    this.ros = ros;
    this.log = el('console-log');

    el('console-toggle').addEventListener('click', () => {
      el('arm-console').classList.toggle('hidden');
    });
    el('console-send').addEventListener('click', () => this._send());
    el('console-input').addEventListener('keydown', (e) => {
      if (e.key === 'Enter') this._send();
    });

    ros.subscribe(TOPICS.armResponse, (m) => this._append(`ARM> ${m.data}`, m.data.startsWith('ERR') ? 'rx-err' : 'rx-ok'));
    ros.subscribe(TOPICS.baseResponse, (m) => this._append(`BASE> ${m.data}`, m.data.startsWith('ERR') ? 'rx-err' : 'rx-ok'));
  }

  _send() {
    const input = el('console-input');
    const line = input.value.trim();
    if (!line) return;
    const target = el('console-target').value;
    const topic = target === 'arm' ? TOPICS.armCommand : TOPICS.baseCommand;
    this.ros.publish(topic, { data: line });
    this._append(`< ${line}`, 'tx');
    input.value = '';
  }

  _append(text, cls) {
    const div = document.createElement('div');
    div.className = cls;
    div.textContent = text;
    this.log.appendChild(div);
    while (this.log.childNodes.length > 200) this.log.removeChild(this.log.firstChild);
    this.log.scrollTop = this.log.scrollHeight;
  }
}
