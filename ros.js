var ros = ros || {};

ros.Connection = function(url) {
  this.handlers = {};

  this.nextCallback = 0;
  this.callbacks = {};

  this.socket = new WebSocket(url);
  var ths = this;
  this.socket.onmessage = function(e) {
    var call = JSON.parse(e.data);

    if (call.callback !== undefined) {
      var handler = ths.callbacks[call.callback];
      delete ths.callbacks[call.callback];
      handler[0](call.msg, handler[1]);
    } else {
      for (var i in ths.handlers[call.receiver]) {
        var handler = ths.handlers[call.receiver][i];
        handler[0](call.msg, handler[1]);
      }
    }
  }
  this.socket.onopen = function(e) {
    if (ths.onopen !== undefined) {
      ths.onopen(e);
    }
  }
  this.socket.onclose = function(e) {
    if (ths.onclose !== undefined) {
      ths.onclose(e);
    }
  }
}

ros.Connection.prototype.callService = function(service, msg, callback, data) {
  this.callbacks[this.nextCallback] = [callback, data];
  this.socket.send(JSON.stringify({receiver:service,callback:this.nextCallback++,msg:msg}));
}

ros.Connection.prototype.publish = function(topic, typeStr, msg) {
  typeStr.replace(/^\//,'');
  this.socket.send(JSON.stringify({receiver:topic,msg:msg,type:typeStr}));
}

ros.Connection.prototype.addHandler = function(topic, func, data) {
  if (!(topic in this.handlers)) {
    this.handlers[topic] = new Array();
  }
  this.handlers[topic].push([func, data]);
}
