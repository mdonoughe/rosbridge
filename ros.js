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
      // service callback
      var handler = ths.callbacks[call.callback];
      delete ths.callbacks[call.callback];
      var context = handler.context || ths;
      var args = handler.arguments || [];
      if (call.msg !== undefined) {
        if (handler.success !== undefined) {
          handler.success.apply(context, [call.msg].concat(args));
        }
      } else {
        if (handler.error !== undefined) {
          handler.error.apply(context, [call.error].concat(args));
        }
      }
      if (handler.complete !== undefined) {
        handler.complete.apply(context, [call].concat(args));
      }
    } else {
      // topic callback
      for (var i in ths.handlers[call.receiver]) {
        var handler = ths.handlers[call.receiver][i];
        var context = handler[1].context || ths;
        var args = handler[1].arguments || [];
        handler[0].apply(context, [call.msg].concat(args));
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

// service = service to call.
// args = {
// data: data to send to the service. Defaults to {}.
// context: context used for the callback(value of this). Defaults to the connection.
// success: function(response, ...) to call on success.
// error: function(error data, ...) to call on error.
// complete: function(raw data, ...) to call after success or error. This receives the raw response from the server which contains either msg or error depending on the result.
// arguments: an array-like object containing extra arguments to be sent to the callbacks.
// };
ros.Connection.prototype.callService = function(service, args) {
  this.callbacks[this.nextCallback] = args;
  // note that the callback field sent to the server is the index of the callback in the array, not the actual callback data
  this.socket.send(JSON.stringify({receiver:service, callback:this.nextCallback++, msg:args.data || {}}));
}

ros.Connection.prototype.publish = function(topic, typeStr, msg) {
  typeStr.replace(/^\//,'');
  this.socket.send(JSON.stringify({receiver:topic,msg:msg,type:typeStr}));
}

// topic = topic to listen for. note that you must call /rosjs/subscribe first or the event will never fire.
// func = function(msg, ...) to call when a message is received.
// args = {
// context: context used when calling the callback. Defaults to the connection.
// arguments: an array-like object containing extra arguments sent to the callback.
// };
ros.Connection.prototype.addHandler = function(topic, func, args) {
  if (!(topic in this.handlers)) {
    this.handlers[topic] = new Array();
  }
  this.handlers[topic].push([func, args]);
}
