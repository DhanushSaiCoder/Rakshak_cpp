# stream_server.py

from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import ssl
import time
import base64
import cv2
import numpy as np
import multiprocessing as mp
from multiprocessing import shared_memory
import threading 
# from modular_main import SharedFrame
import sys
from share_frame import SharedFrame
from log_util import setup_combined_logger
import logging
import queue
logger = setup_combined_logger(__name__)
# logger = logging.getLogger(module_name)

USERNAME = "admin"
PASSWORD = "1234"
PORT = 8443

cam_dict=None
stop_flag=False

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class MJPEGStreamer:
    def __init__(self, read_func, wfile, stop_flag, quality=70):
        self.read_func = read_func
        self.wfile = wfile
        self.stop_flag = stop_flag
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        self.latest_frame = None
        self.jpg_queue = queue.Queue(maxsize=1)
        self.encode_thread = threading.Thread(target=self._encoder_loop, daemon=True)

    def _encoder_loop(self):
        while not self.stop_flag:
            frame = self.read_func()
            if frame is None:
                continue
            # start = time.time()
            success, jpg = cv2.imencode('.jpg', frame, self.encode_param)
            # end = time.time()
            # print(f'Elapsed time: {end - start}')
            if success:
                try:
                    self.jpg_queue.get_nowait()  # discard old
                except queue.Empty:
                    pass
                self.jpg_queue.put(jpg.tobytes())

    def start(self):
        self.encode_thread.start()
        while not self.stop_flag:
            try:
                jpg_bytes = self.jpg_queue.get(timeout=1)
            except queue.Empty:
                continue

            try:
                self.wfile.write(b"--jpgboundary\r\n")
                self.wfile.write(b"Content-type: image/jpeg\r\n")
                self.wfile.write(f"Content-length: {len(jpg_bytes)}\r\n\r\n".encode())
                self.wfile.write(jpg_bytes)
                self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                print("[Stream] Client disconnected")
                break

class MultiCamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Disable request logging
        return
    def do_AUTHHEAD(self):
        self.send_response(401)
        self.send_header('WWW-Authenticate', 'Basic realm=\"SecureCamStream\"')
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        

    def is_authenticated(self):
        auth_header = self.headers.get('Authorization')
        if not auth_header:
            self.do_AUTHHEAD()
            return False
        try:
            encoded = auth_header.split(" ")[1]
            decoded = base64.b64decode(encoded).decode('utf-8')
            user, pwd = decoded.split(':')
            return user == USERNAME and pwd == PASSWORD
        except:
            self.do_AUTHHEAD()
            return False

    def do_GET(self):
        global cam_dict
        global stop_flag

        # if not self.is_authenticated():
        #     return
        
        path = self.path.strip("/")
        # print(f'=============User Request Path={path}=================')
        # if not path.startswith("cam") or not path[3:].isdigit():
        #     self.send_error(404)
        #     return

        # cam_id = int(path[3:])
        cam_id = path        
        if cam_id not in cam_dict:
            self.send_error(404, f"No such camera: {cam_id} in {cam_dict.keys()}")
            return

        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        # print(f'Cur Cam={cur_cam}')
        static_reader = SharedFrame(name=cam_dict["static_cam"])
        top_reader = SharedFrame(name=cam_dict["top_cam"]) 
        read_dict={"static_cam":static_reader,"top_cam":top_reader}
        # streamer = MJPEGStreamer(
        #     read_func=lambda: read_dict[cam_id].read(),
        #     wfile=self.wfile,
        #     stop_flag=stop_flag,
        #     quality=50
        # )
        # streamer.start()
        try:
            while not stop_flag:

                frame=read_dict[cam_id].read()
                if frame is not None:
                    _, jpg = cv2.imencode('.jpg', frame)
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.wfile.write(b"Content-type: image/jpeg\r\n")
                    self.wfile.write(f"Content-length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
            
            # time.sleep(0.05)
        except Exception as e:
            pass
            # print(f"[DISCONNECT] Cam{cam_id}: {e}")    


def run_https_server(shared_static_name, shared_top_name, shared_state):
    global stop_flag
    global cam_dict
    cam_dict={"static_cam":shared_static_name, "top_cam":shared_top_name}
    # print(f'Received Global Dict={cam_dict.keys()}')
    server = ThreadedHTTPServer(('0.0.0.0', PORT), MultiCamHandler)
    # context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    # context.load_cert_chain(certfile='cert.pem', keyfile='key.pem')
    # server.socket = context.wrap_socket(server.socket, server_side=True)
    print(f"[INFO] HTTPS Streaming running on https://<ip>:{PORT}/static_cam, top_cam, ...")
    # server.serve_forever()  
    
    t1=threading.Thread(target=server.serve_forever,daemon=True)
    t1.start()
    while not shared_state.stop_flag.value:
        # print('HTTP Running')
        continue
    stop_flag=True
    print('HTTP Stopped')
    server.server_close()
    # server.shutdown()