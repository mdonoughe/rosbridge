This is a very modified version of
[rosbridge](http://www.ros.org/wiki/rosbridge) from brown_remotelab.

See the [wiki](https://github.com/mdonoughe/rosbridge/wiki) for documentation.

## Changes

I've changed services to have callbacks like this:

```javascript
{receiver: '/test_service', callback: 0, msg: {data: 'request'}}
{callback: 0, msg: {data: 'response'}}
```

so that you can have multiple requests in flight to the same service at the
same time with different callbacks. This doesn't change the appearance of the
JavaScript API, but it *does* change its behavior to work like you'd expect.
As a result, the following code now prints 1 and 2 rather than 2 and 2.

```javascript
connection.callService('/rosjs/subscribe', {
  data: ['/test1', 0],
  success: function(rsp) {
    console.log('1');
  }
});
connection.callService('/rosjs/subscribe', {
  data: ['/test2', 0],
  success: function(rsp) {
    console.log('2');
  }
});
```

It is also possible for an error to be returned from a service call. The
following code prints "Topic does not exist" unless /unpublishedTopic is
actually published.

```javascript
connection.callService('/rosjs/subscribe', {
  data: ['/unpublishedTopic', 0],
  success: function(rsp) {
    console.log('success');
  },
  error: function(rsp) {
    console.log(rsp);
  }
});
```

The JavaScript API has changed. The event handlers are now just fields without
setters, and there is no event handler for onerror because that's not part of
the WebSocket spec.

You can also pass an extra argument to addHandler and callService, which will
be passed as a second argument to your callback function.

```javascript
// print 'hello' to the console every time a message comes in on /test
connection.addHandler('/test', function(msg, data) {
  console.write(data);
}, {arguments: ['hello']});
```

I was having an issue with terribleWSS using 100% CPU on my test machine, so I
replaced it with txWebSocket and Twisted. txWebSocket is included in the bin
directory, but Twisted must be installed manually. python-twisted from Debian
works fine.

Note: the included txWebSocket is a slightly modified version from
https://github.com/mdonoughe/txWebSocket .

Setting wspath changes the path to the websocket. By default, the path is '/',
so you'd connect with 'ws://localhost:9090/'.

You'll probably want to change wspath if you set docroot, which causes a
folder to be exposed at 'http://localhost:9090/'.

You can use jsfile(default is 'ros.js') to make the ros.js file visible at
'http://localhost:9090/ros.js', or set it to '' to make it disappear.

Parameters are now referenced as ~param instead of /brown/rosbridge/param.
This means that you can easily set params in roslaunch like you'd expect.

## Example

I'm using the following configuration with roslaunch to host a web application
at http://0.0.0.0:9090/:

```xml
<node pkg="rosbridge" type="rosbridge.py" name="bridge">
  <param name="wspath" value="/ws"/>
  <param name="docroot" value="$(find aslam_input)/docroot"/>
</node>
```

A file index.html in the aslam_input/docroot includes /ros.js(automatically
provided by rosbridge, not actually present in the docroot folder) and another
script which uses `new ros.Connection('ws://' + location.host + '/ws');` to
connect to the websocket without hardcoding an address.
