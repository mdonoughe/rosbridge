This is similar to [rosbridge](http://www.ros.org/wiki/rosbridge) from
brown_remotelab.

I've changed services to have callbacks like this:

    {receiver: '/test_service', callback: 0, msg: {data: 'request'}}
    {callback: 0, msg: {data: 'response'}}

so that you can have multiple requests in flight to the same service at the
same time with different callbacks. This doesn't change the appearance of the
JavaScript API, but it *does* change its behavior to work like you'd expect.
As a result, the following code now prints 1 and 2 rather than 2 and 2.

    connection.callService('/rosjs/subscribe', ['/test',0], function(rsp) {
      console.log('1');
    });
    connection.callService('/rosjs/subscribe', ['/test',0], function(rsp) {
      console.log('2');
    });

The JavaScript API has changed slightly. The event handlers are now just
fields without setters, and there is no event handler for onerror because
that's not part of the WebSocket spec.

You can also pass an extra argument to addHandler and callService, which will
be passed as a second argument to your callback function.

    // print 'hello' to the console every time a message comes in on /test
    connection.addHandler('/test', function(msg, data) {
      console.write(data);
    }, 'hello');

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
