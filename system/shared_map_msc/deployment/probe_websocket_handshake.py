import socket,time,base64,os,json
started=time.monotonic()
with socket.create_connection(('127.0.0.1',8765),timeout=3) as sock:
 key=base64.b64encode(os.urandom(16)).decode()
 request=('GET / HTTP/1.1\r\nHost: 127.0.0.1:8765\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
          'Sec-WebSocket-Key: '+key+'\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: foxglove.sdk.v1\r\n\r\n')
 sock.sendall(request.encode());sock.settimeout(3)
 try:data=sock.recv(4096);print(json.dumps({'seconds':time.monotonic()-started,'response':data.decode(errors='replace')[:1500]}))
 except socket.timeout:print(json.dumps({'seconds':time.monotonic()-started,'error':'TCP connected but websocket handshake response timed out'}))
