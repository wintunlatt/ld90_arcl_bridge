import socket, threading, time, math

class ClientSim(threading.Thread):
    def __init__(self, conn, addr):
        super().__init__(daemon=True)
        self.conn = conn
        self.addr = addr
        self.running = True
        self.auth = False
        # Sim state
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.vx = 0.0; self.wz = 0.0
        self.last = time.time()
        self.lock = threading.Lock()

    def run(self):
        try:
            self.conn.sendall(b"Welcome to ARCL Simulation\r\n")
        except:
            return

        # Start a periodic POSE broadcaster
        threading.Thread(target=self.pose_loop, daemon=True).start()

        # Read lines
        file = self.conn.makefile('rb')
        while self.running:
            line = file.readline()
            if not line:
                break
            text = line.decode('utf-8', errors='ignore').strip()
            if not text:
                continue
            up = text.upper()
            if up.startswith("CONNECT "):
                # accept any password in sim
                self.auth = True
                self._send("OK")
            elif up.startswith("GETSTATUS"):
                self._send("STATUS OK")
            elif up.startswith("STOP"):
                with self.lock:
                    self.vx = 0.0; self.wz = 0.0
                self._send("OK")
            elif up.startswith("DRIVE "):
                try:
                    parts = text.split()
                    vx = float(parts[1]); wz = float(parts[2])
                    with self.lock:
                        self.vx = vx; self.wz = wz
                    self._send("OK")
                except:
                    self._send("ERR")
            else:
                self._send("OK")
        self.running = False
        try:
            self.conn.close()
        except:
            pass

    def pose_loop(self):
        while self.running:
            now = time.time()
            dt = now - self.last
            self.last = now
            with self.lock:
                # simple unicycle model
                self.x += self.vx * math.cos(self.yaw) * dt
                self.y += self.vx * math.sin(self.yaw) * dt
                self.yaw += self.wz * dt
                # normalize yaw
                if self.yaw > math.pi: self.yaw -= 2*math.pi
                if self.yaw < -math.pi: self.yaw += 2*math.pi
                x,y,yaw = self.x, self.y, self.yaw
            self._send(f"POSE {x:.3f} {y:.3f} {yaw:.6f}")
            time.sleep(0.1)

    def _send(self, s):
        try:
            self.conn.sendall((s + "\r\n").encode('utf-8'))
        except:
            self.running = False

def main():
    host = "127.0.0.1"
    port = 7260
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(5)
    print(f"[FAKE ARCL] Listening on {host}:{port}")
    try:
        while True:
            conn, addr = srv.accept()
            print(f"[FAKE ARCL] Client connected: {addr}")
            ClientSim(conn, addr).start()
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()

if __name__ == "__main__":
    main()
