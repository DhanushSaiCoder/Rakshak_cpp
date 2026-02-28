import os, json, time, random, logging, threading
from typing import Callable, List, Optional
import pika
import Globals as g # Loading of all Global variables as g
from log_util import setup_combined_logger
import uuid
from datetime import datetime, timezone, timedelta
log=setup_combined_logger(__name__)

class BaseMQ_New:
    def __init__(
        self,
        exchange: str = g.RABBIT_EXCHANGE,
        exchange_type: str = g.RABBIT_EXCHANGE_TYPE,
        prefetch: int = g.PREFETCH,
        queue: str = None,
        # creds (override by env or pass explicitly)
        host: Optional[str] = None,
        port: Optional[int] = None,
        user: Optional[str] = None,
        password: Optional[str] = None,
        vhost: Optional[str] = None,
        connection_name: str = g.CONNECTION_NAME,
        system: str = g.SYSTEM_ID,
        base: str = g.BASE, 
    ):
        self.exchange = exchange
        self.exchange_type = exchange_type
        self.queue = queue
        self.prefetch = prefetch

        self.host = host or g.RABBIT_HOST
        self.port = int(port or g.RABBIT_PORT)
        self.user = user or g.RABBIT_USER
        self.password = password or g.RABBIT_PASS
        self.vhost = vhost or g.RABBIT_VHOST
        self.connection_name = connection_name

        self.system = system
        self.base = base

        self.connection: Optional[pika.BlockingConnection] = None
        self.pub_ch: Optional[pika.adapters.blocking_connection.BlockingChannel] = None
        self.sub_ch: Optional[pika.adapters.blocking_connection.BlockingChannel] = None

        self._connect()

    # ---------- internals ----------

    def _params(self) -> pika.ConnectionParameters:
        creds = pika.PlainCredentials(self.user, self.password)
        return pika.ConnectionParameters(
            host=self.host,
            port=self.port,
            virtual_host=self.vhost,
            credentials=creds,
            heartbeat=g.HEARTBEAT,
            blocked_connection_timeout=g.BLOCKED_CONNECTION_TIMEOUT,
            socket_timeout=g.SOCKET_TIMEOUT,
            connection_attempts=g.CONNECTION_ATTEMPTS,
            retry_delay=g.RETRY_DELAY,
            client_properties={
                "connection_name": self.connection_name,
                "product": "basemq",
                "version": "1.0",
            },
        )

    def _connect(self):
        backoff = 1
        while True:
            try:
                if self.connection and self.connection.is_open:
                    try: self.connection.close()
                    except Exception: pass

                self.connection = pika.BlockingConnection(self._params())

                # Publisher channel
                self.pub_ch = self.connection.channel()
                # self.pub_ch.confirm_delivery() # enable confirm_delivery, RabbitMQ switches the channel into confirm mode.
                # self.pub_ch.add_on_return_callback(self._on_returned) # works only when manidatory in basic publish is true 

                # Subscriber channel
                self.sub_ch = self.connection.channel()
                self._declare_topology(self.sub_ch)
                self.sub_ch.basic_qos(prefetch_count=self.prefetch)

                log.info(f"[BaseMQ] Connected to {self.host}:{self.port} vhost={self.vhost}")
                return
            except Exception as e:
                log.error(f"[BaseMQ] connect failed: {e}; retrying in {backoff}s")
                time.sleep(backoff + random.random())
                backoff = min(backoff * 2, g.RECONNECTION_TIME_LIMIT)

    def _declare_topology(self, ch):
        # Durable topic exchange
        ch.exchange_declare(self.exchange, self.exchange_type, durable=True)
        # Durable queue (classic durable for easiest local testing)
        ch.queue_declare(
            self.queue,
            durable=False,        # transient queue, not stored to disk
            auto_delete=True,     # delete when consumer disconnects
            exclusive=False,      # or True if only one consumer should ever attach
            arguments={
                "x-message-ttl": 2000,       # messages older than 2s are dropped
                "x-max-length": 1,           # only keep the latest
                "x-overflow": "drop-head"    # drop oldest when full
            }
        )
        # Note: if your broker supports quorum, replace above with:
        # ch.queue_declare(self.queue, durable=True, arguments={"x-queue-type":"quorum"})

    # This is binded with the on returned so this will prints the dropped message 
    def _on_returned(self, ch, method, props, body):
        log.error(f"[BaseMQ] UNROUTABLE publish: rk={method.routing_key} "
                  f"code={method.reply_code} {method.reply_text} body={body[:200]}")

    # ---------- public API ----------

    def bind(self, binding_keys: List[str]):
        """Bind the durable queue to one or more routing-key patterns."""
        for key in binding_keys:
            self.sub_ch.queue_bind(queue=self.queue, exchange=self.exchange, routing_key=key)
        # log.info(f"[BaseMQ] Bound {self.queue} to {binding_keys}")

    def publish(self, topic: str, msg_type: str, payload: dict, schema_ver: str = "1.0"):
        """Reliable publish (persistent + confirms)."""
        routing_key = f"{self.base}.{self.system}.{topic}"
        # body = json.dumps(message, separators=(",", ":"))
            
        message = {
            "schema_ver": schema_ver,
            "msg_type": msg_type,
            "ts": datetime.now(timezone.utc).astimezone(timezone(timedelta(hours=5, minutes=30))).isoformat(timespec="milliseconds"),
            "message_id": str(uuid.uuid4()),
            "system": self.system,
            "payload": payload
        }
        # if msg_type == "models_list":
        #     print(f'message -> {message}')
        body = json.dumps(message)
        props = pika.BasicProperties(
            content_type="application/json",
            delivery_mode=1,  # persistent
        )
        try:
            if self.connection and self.connection.is_open:
                self.connection.process_data_events(time_limit=0)

            self.pub_ch.basic_publish(
                exchange=self.exchange,
                routing_key=routing_key,
                body=body,
                properties=props,
                mandatory=False,
            )

            if self.connection and self.connection.is_open:
                self.connection.process_data_events(time_limit=0)

        except Exception as e:
            log.error(f"[BaseMQ] publish error {e}; reconnect→retry once")
            self._connect()
            self.pub_ch.basic_publish(
                exchange=self.exchange,
                routing_key=routing_key,
                body=body,
                properties=props,
                mandatory=True,
            )

    def subscribe(self, on_message: Callable[[str, dict], None]):
        """
        Blocking consume loop. The queue must already be bound via bind().
        Callback: on_message(routing_key, message_dict)
        """
        def _on_msg(ch, method, props, body: bytes):
            try:
                msg = json.loads(body)
            except Exception:
                log.error("[BaseMQ] bad JSON → ack & skip")
                ch.basic_ack(method.delivery_tag)
                return

            try:
                on_message(method.routing_key, msg)
                ch.basic_ack(method.delivery_tag)
            except Exception as e:
                log.error(f"[BaseMQ] handler error: {e}")
                ch.basic_ack(method.delivery_tag)

        while True:
            try:
                if self.sub_ch is None or self.sub_ch.is_closed:
                    self._connect()
                self.sub_ch.basic_consume(queue=self.queue, on_message_callback=_on_msg, auto_ack=False)
                # log.info(f"[BaseMQ] Consuming on queue='{self.queue}' (Ctrl+C to stop)")
                self.sub_ch.start_consuming()
            except (pika.exceptions.StreamLostError, pika.exceptions.AMQPConnectionError) as e:
                log.warning(f"[BaseMQ] connection lost: {e}; reconnecting…")
                self._connect()
            except KeyboardInterrupt:
                try: self.sub_ch.stop_consuming()
                except Exception: pass
                try: self.connection.close()
                except Exception: pass
                # log.info("[BaseMQ] Stopped.")
                break
            except Exception as e:
                log.error(f"[BaseMQ] fatal: {e}")
                time.sleep(2)
