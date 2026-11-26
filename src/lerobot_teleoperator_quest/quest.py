"""Real LeRobot input plugin. Does not open or command a robot."""

import hmac
import json
import secrets
import socket
import ssl
from threading import Lock, Thread, current_thread
from time import monotonic
from urllib.parse import urlencode

from lerobot.teleoperators.teleoperator import Teleoperator
from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve

from .config import QuestConfig
from .contracts import Telemetry
from .input import HANDS, InputBuffer, frame_action
from .so101 import FOLLOWER_KEYS, from_follower_observation


class Quest(Teleoperator):
    config_class = QuestConfig
    name = "quest"

    def __init__(self, config: QuestConfig):
        super().__init__(config)
        self.config = config
        self._buffer = InputBuffer(timeout=config.timeout_s, bimanual=config.bimanual)
        self._server = None
        self._thread = None
        self._connection = None
        self._transports = {}
        self._accepting = False
        self._connection_lock = Lock()
        self._feedback_lock = Lock()
        self._feedback: dict[str, Telemetry] | None = None
        self._state: str | None = None  # the newest undelivered state message
        self._token = secrets.token_urlsafe(32)

    @property
    def access_token(self) -> str:
        """Ephemeral credential; intentionally absent from dataclass config/logs."""
        return self._token

    @property
    def action_features(self) -> dict:
        empty = dict.fromkeys(self._arms)
        return {key: type(value) for key, value in frame_action(empty, 0).items()}

    @property
    def _arms(self) -> tuple[str, ...]:
        return HANDS if self.config.bimanual else ("",)

    @staticmethod
    def _follower_prefix(arm: str) -> str:
        # The official bimanual SO follower prefixes each arm's keys this way.
        return f"{arm}_" if arm else ""

    @property
    def feedback_features(self) -> dict:
        return {
            self._follower_prefix(arm) + key: float for arm in self._arms for key in FOLLOWER_KEYS
        }

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

    def pairing_link(self, page: str, host: str | None = None) -> str:
        """The client page's address with this run's listener and token filled in.

        Open it on the headset instead of typing the token there. Everything
        after "#" stays in the browser and is never sent to the page's server.
        `host` replaces a wildcard or loopback bind address with the name the
        headset should use. Calling this reveals the token: treat the link as one.
        """
        if not isinstance(page, str) or not page or "#" in page:
            raise ValueError("page must be the client page's address, without a fragment")
        url = self.url
        if host is not None:
            if not isinstance(host, str) or not host:
                raise ValueError("host must be a non-empty name or address")
            scheme, _, rest = url.partition("://")
            url = f"{scheme}://{host}:{rest.rsplit(':', 1)[1]}"
        fields = {"url": url, "token": self._token}
        if self.config.bimanual:
            fields["mode"] = "both"
        return f"{page}#{urlencode(fields)}"

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
            self._state = None
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
            version = 2 if self.config.bimanual else 1
            connection.send(json.dumps({"type": "ready", "version": version, "session": session}))
            for message in connection:
                payload = json.loads(message)
                # A ping is not input and refreshes nothing: it only gives the
                # listener a moment to deliver state while no sample is flowing.
                if payload != {"type": "ping"}:
                    self._buffer.submit(session, payload)
                    connection.send(json.dumps({"type": "accepted", "seq": payload["seq"]}))
                with self._feedback_lock:
                    state, self._state = self._state, None
                if state is not None:
                    connection.send(state)
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
        return frame_action(*self._buffer.frame())

    def send_feedback(self, feedback: dict) -> None:
        if not self.is_connected:
            raise RuntimeError("Quest input is disconnected")
        if not isinstance(feedback, dict):
            raise ValueError("feedback must be a follower observation")
        now = monotonic()
        snapshot = {}
        for arm in self._arms:
            prefix = self._follower_prefix(arm)
            own = {key: feedback[prefix + key] for key in FOLLOWER_KEYS if prefix + key in feedback}
            snapshot[arm] = Telemetry(from_follower_observation(own), None, None, now)
        with self._feedback_lock:
            self._feedback = snapshot

    def send_state(self, state: dict) -> None:
        """Queue what the headset should draw; see `QuestControlStep.headset_state`.

        It never blocks on the headset: the listener delivers only the newest
        state, after acknowledging the client's next sample.
        """
        if not self.is_connected:
            raise RuntimeError("Quest input is disconnected")
        if not isinstance(state, dict) or state.get("type") != "state":
            raise ValueError('state must be a dictionary with "type": "state"')
        try:
            text = json.dumps(state, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise ValueError("state must be finite JSON") from exc
        if len(text) > 8192:
            raise ValueError("state is too large to send every frame")
        with self._feedback_lock:
            self._state = text

    @property
    def feedback(self) -> dict | None:
        """Measured positions only: one record, or one per arm when bimanual."""
        with self._feedback_lock:
            if self._feedback is None:
                return None
            records = {arm: record.to_dict() for arm, record in self._feedback.items()}
        return records if self.config.bimanual else records[""]

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
