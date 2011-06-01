var ros = ros || {};

ros.Connection = function(url) {
  this.handlers = new Array();

  this.nextCallback = 0;
  this.callbacks = {};

  this.socket = new WebSocket(url);
  this.onmessage = null;
  var ths = this;
  this.socket.onmessage = function(e) {
    if(ths.onmessage) {
      try {
        ths.onmessage(e);
      } catch(err) {
        ros_debug(err);
      }
    }

    var call = JSON.parse(e.data);

    if (call.callback !== undefined) {
      var handler = ths.callbacks[call.callback];
      delete ths.callbacks[call.callback];
      handler(call.msg);
    } else {
      for (var i in ths.handlers[call.receiver]) {
        var handler = ths.handlers[call.receiver][i]
        handler(call.msg);
      }
    }
  }
}

ros.Connection.prototype.callService = function(service, msg, callback) {
  this.callbacks[this.nextCallback] = callback;
  this.socket.send(JSON.stringify({receiver:service,callback:this.nextCallback++,msg:msg}));
}

ros.Connection.prototype.publish = function(topic, typeStr, msg) {
  typeStr.replace(/^\//,'');
  this.socket.send(JSON.stringify({receiver:topic,msg:msg,type:typeStr}));
}

ros.Connection.prototype.addHandler = function(topic, func) {
  if (!(topic in this.handlers)) {
    this.handlers[topic] = new Array();
  }
  this.handlers[topic].push(func);
}

ros.Connection.prototype.setOnError = function(func) {
  this.socket.onerror = func;
}

ros.Connection.prototype.setOnClose = function(func) {
  this.socket.onclose = func;
}

ros.Connection.prototype.setOnOpen = function(func) {
  this.socket.onopen = func;
}

ros.Connection.prototype.setOnMessage = function(func) {
  this.onmessage = func;
}
