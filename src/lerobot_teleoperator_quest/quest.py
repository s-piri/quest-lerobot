"""Real LeRobot input plugin. Does not open or command a robot."""

import hmac
import json
import secrets
import socket
import ssl
from threading import Lock, Thread, current_thread
from time import monotonic

from lerobot.teleoperators.teleoperator import Teleoperator
from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve

from .config import QuestConfig
from .contracts import Telemetry
from .input import InputBuffer, input_action
from .so101 import FOLLOWER_KEYS, from_follower_observation


class Quest(Teleoperator):
    config_class = QuestConfig
    name = "quest"

    def __init__(self, config: QuestConfig):
        super().__init__(config)
        self.config = config
        self._buffer = InputBuffer(timeout=config.timeout_s)
        self._server = None
        self._thread = None
        self._connection = None
        self._transports = {}
        self._accepting = False
        self._connection_lock = Lock()
        self._feedback_lock = Lock()
        self._feedback: Telemetry | None = None
        self._token = secrets.token_urlsafe(32)

    @property
    def access_token(self) -> str:
        """Ephemeral credential; intentionally absent from dataclass config/logs."""
        return self._token

    @property
    def action_features(self) -> dict:
        return {key: type(value) for key, value in input_action(None, 0).items()}

    @property
    def feedback_features(self) -> dict:
        return dict.fromkeys(FOLLOWER_KEYS, float)

    @property
    def is_connected(self) -> bool:
        return self._server is not None

    @property
    def is_calibrated(self) -> bool:
        # This is input-device calibration, never the follower's motor calibration.
        return True

    def calibrate(self) -> None:
        """Input device requires no servo calibration; does not write files."""

    def configure(self) -> None:
        """No motor configuration is owned by this input device."""

    @property
    def url(self) -> str:
        if not self.is_connected:
            raise RuntimeError("Quest input is disconnected")
        port = self._server.socket.getsockname()[1]
        host = self.config.host
        if ":" in host:
            host = f"[{host}]"
        scheme = "wss" if self.config.cert else "ws"
        return f"{scheme}://{host}:{port}"

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise RuntimeError("Quest input is already connected")
        context = None
        if self.config.cert is not None:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(self.config.cert, self.config.key)
        self._token = secrets.token_urlsafe(32)
        self._server = serve(
            self._handle,
            self.config.host,
            self.config.port,
            ssl=context,
            origins=self.config.origins,
            max_size=4096,
            max_queue=1,
            close_timeout=1,
            open_timeout=3,
        )
        # Server.handler is the documented accepted-socket handler. Wrap it
        # before starting acceptance so even TLS/HTTP handshakes are tracked.
        handler = self._server.handler
        self._server.handler = lambda sock, addr: self._track_transport(handler, sock, addr)
        with self._feedback_lock:
            self._feedback = None
        with self._connection_lock:
            self._accepting = True
        self._thread = Thread(target=self._server.serve_forever, name="quest-input", daemon=True)
        self._thread.start()

    def _track_transport(self, handler, sock, addr):
        with self._connection_lock:
            accepting = self._accepting
            if accepting:
                self._transports[sock] = current_thread()
        try:
            if accepting:
                handler(sock, addr)
        finally:
            sock.close()
            with self._connection_lock:
                self._transports.pop(sock, None)

    def _handle(self, connection):
        session = None
        try:
            with self._connection_lock:
                accepting = self._accepting
            if not accepting:
                connection.close(1001, "input shutting down")
                return
            auth = json.loads(connection.recv(timeout=3))
            if (
                not isinstance(auth, dict)
                or set(auth) != {"type", "token"}
                or auth["type"] != "auth"
                or not isinstance(auth["token"], str)
                or not hmac.compare_digest(auth["token"], self._token)
            ):
                connection.close(1008, "authentication required")
                return
            with self._connection_lock:
                busy = not self._accepting or self._connection is not None
                if not busy:
                    session = self._buffer.open_session()
                    self._connection = connection
            if busy:
                connection.close(1013, "input already has an owner")
                return
            connection.send(json.dumps({"type": "ready", "version": 1, "session": session}))
            for message in connection:
                payload = json.loads(message)
                self._buffer.submit(session, payload)
                connection.send(json.dumps({"type": "accepted", "seq": payload["seq"]}))
        except (ValueError, TypeError, TimeoutError):
            # Invalidate before the closing handshake: get_action must not
            # expose the last armed sample while a peer delays that handshake.
            if session is not None:
                self._buffer.close_session(session)
            connection.close(1008, "invalid input")
        except ConnectionClosed:
            pass
        finally:
            if session is not None:
                self._buffer.close_session(session)
            with self._connection_lock:
                if self._connection is connection:
                    self._connection = None

    def get_action(self) -> dict:
        if not self.is_connected:
            raise RuntimeError("Quest input is disconnected")
        return input_action(*self._buffer.snapshot())

    def send_feedback(self, feedback: dict) -> None:
        if not self.is_connected:
            raise RuntimeError("Quest input is disconnected")
        snapshot = Telemetry(from_follower_observation(feedback), None, None, monotonic())
        with self._feedback_lock:
            self._feedback = snapshot

    @property
    def feedback(self) -> dict | None:
        with self._feedback_lock:
            return None if self._feedback is None else self._feedback.to_dict()

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        with self._connection_lock:
            self._accepting = False
        session = self._buffer.session
        self._buffer.close_session(session)
        # Stop accepting before closing all accepted raw transports, including
        # pending TLS/HTTP/auth handshakes. No sequential peer close handshakes.
        self._server.shutdown()
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            raise RuntimeError("Quest input listener failed to stop")
        with self._connection_lock:
            transports = tuple(self._transports.items())
        for sock, _ in transports:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass  # A peer or completed handshake may already have closed it.
            sock.close()
        deadline = monotonic() + 5
        for _, worker in transports:
            worker.join(timeout=max(0, deadline - monotonic()))
            if worker.is_alive():
                raise RuntimeError("Quest input connection failed to stop")
        self._server = None
        self._thread = None
