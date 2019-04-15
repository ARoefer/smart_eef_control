#!/usr/bin/python
import BaseHTTPServer, SimpleHTTPServer
import ssl

httpd = BaseHTTPServer.HTTPServer(('0.0.0.0', 8000), SimpleHTTPServer.SimpleHTTPRequestHandler)
httpd.socket = ssl.wrap_socket(httpd.socket, certfile='./certs_and_key.pem', ssl_version=ssl.PROTOCOL_TLSv1, server_side=True)
httpd.serve_forever()