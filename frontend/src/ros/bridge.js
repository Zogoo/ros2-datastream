import ROSLIB from 'roslib';

/** Thin rosbridge wrapper: reconnecting connection lifecycle, advertise-once
 *  publishing, re-subscribing registry and a TX counter for the HUD. */
export class RosBridge {
  constructor(url) {
    this.url = url;
    this.ros = null;
    this.connected = false;
    this.publishers = new Map();
    this.subscriberSpecs = [];
    this.activeSubs = [];
    this.txCount = 0;
    this.onStatusChange = null;
  }

  connect() {
    this.ros = new ROSLIB.Ros({ url: this.url });
    this.ros.on('connection', () => {
      this.connected = true;
      this._resubscribeAll();
      this.onStatusChange?.(true);
    });
    this.ros.on('close', () => {
      const wasConnected = this.connected;
      this.connected = false;
      this.publishers.clear();
      this.activeSubs = [];
      if (wasConnected) this.onStatusChange?.(false);
      setTimeout(() => this.connect(), 2000);
    });
    this.ros.on('error', () => {});
  }

  publish(topic, message) {
    if (!this.connected) return;
    let pub = this.publishers.get(topic.name);
    if (!pub) {
      pub = new ROSLIB.Topic({
        ros: this.ros,
        name: topic.name,
        messageType: topic.type,
        queue_size: 1,
      });
      pub.advertise();
      this.publishers.set(topic.name, pub);
    }
    pub.publish(new ROSLIB.Message(message));
    this.txCount += 1;
  }

  subscribe(topic, callback) {
    this.subscriberSpecs.push({ topic, callback });
    if (this.connected) this._activate(topic, callback);
  }

  subscribeJson(topic, callback) {
    this.subscribe(topic, (msg) => {
      try {
        callback(JSON.parse(msg.data));
      } catch {
        /* malformed payloads are dropped */
      }
    });
  }

  _resubscribeAll() {
    for (const { topic, callback } of this.subscriberSpecs) this._activate(topic, callback);
  }

  _activate(topic, callback) {
    const sub = new ROSLIB.Topic({
      ros: this.ros,
      name: topic.name,
      messageType: topic.type,
    });
    sub.subscribe(callback);
    this.activeSubs.push(sub);
  }
}
