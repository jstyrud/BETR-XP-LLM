import logging
import ssl
import time
from dataclasses import dataclass
from threading import Thread
from typing import List

import urllib3
import websocket
import xmltodict

from rws_py.rest_client import RESTClient

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

ACCEPT_HEADER = {"Accept": "application/xhtml+xml;v=2.0"}


@dataclass
class SubscriptionInfo:
    """Data class of info needed for a subscription"""

    uri: str
    """ The URI of the resource to subscribe to"""
    title: str
    """ The resource title used to parse the subscription message"""
    callback: callable
    """The callback that will be called when the resource changes."""
    priority: int = 1
    """The priority of the subscription. 0:Low, 1:Medium, 2:High:
        '0' for Low priority    (Valid for all resources)
        '1' for Medium priority (Valid for all resources)
        '2' for High priority   (Valid for only 'IOSIGNALS' and 'RAPID Persistent variable value \
            resources) """


class RWSSubscriber:
    ws_thread: Thread = None
    closed: bool = False
    subd_resources: List = []
    group_id: str = ""
    ws = None
    timeout = 3

    def __init__(
        self,
        client: RESTClient,
        logger_: logging.Logger,
    ) -> None:
        self.logger = logger_
        self.client = client
        self.host_url = f"{client.conf.protocol}://{client.conf.host_ip}:{client.conf.port}"
        self.host_ws_url = (
            f"{client.conf.ws_protocol}://{client.conf.host_ip}:{client.conf.ws_port}"
        )
        self.ws_sup_protocol = client.conf.ws_sup_protocol

    def subscribe(self, sub_info: SubscriptionInfo):
        # res_uri, res_response_str, on_change_callback, priority=1):
        self.logger.debug(f"Subscribing to {sub_info.uri}")
        n_res = len(self.subd_resources)
        # data = {"resources": "1", "1": res_uri, "1-p": "1"}
        data = {
            "resources": n_res + 1,
            (n_res + 1): sub_info.uri,
            f"{n_res+1}-p": str(sub_info.priority),
        }
        if n_res == 0:
            # first subscription.
            resp = self.client.session.post(
                self.host_url + "/subscription",
                data,
                timeout=self.timeout,
                verify=False,
                headers=self.client.POST_HEADER,
            )
        else:
            resp = self.client.session.put(
                self.host_url + "/subscription/" + self.group_id,
                data,
                timeout=self.timeout,
                verify=False,
                headers=self.client.POST_HEADER,
            )

        if resp.status_code == 201:
            location = resp.headers["Location"]
            self.group_id = location[location.find("/poll") + 6 :]
            self.logger.debug(f"Subscribed to {sub_info.uri} with group id:{self.group_id}.")
        elif resp.status_code == 200:
            self.logger.debug(f"Subscribed to {sub_info.uri} with group id:{self.group_id}")
        else:
            self.logger.error(f"subscription response: {resp.reason}")
            return

        self.subd_resources.append(sub_info)
        if self.ws_thread is None:
            self.ws_thread = Thread(
                target=self.createWebSocket,
                args=(self.host_ws_url + "/poll/" + self.group_id, self.client.headers),
            )
            self.ws_thread.daemon = True
            self.ws_thread.start()
            time.sleep(0.1)

    def close(self):
        self.ws.close()
        if self.ws_thread:
            self.ws_thread.join()
        self.closed = True
        self.ws_thread = None

    def unsubscribe(self, group_id, resource_uri) -> bool:
        self.logger.debug(f"Unsubscribing from {resource_uri}")
        resp = self.client.session.delete(
            self.host_url + f"/subscription/{group_id}/{resource_uri}",
            timeout=self.timeout,
            verify=False,
            headers=ACCEPT_HEADER,
        )
        if resp.status_code == 200:
            self.logger.debug(f"Unsubscribed from {resource_uri}")
            return True
        else:
            self.logger.error(f"Unsubscribing failed. : {resp.status_code} Resp:{ resp.reason}")
            return False

    def unsubscribe_all(self):
        for i in reversed(range(len(self.subd_resources))):
            if self.unsubscribe(group_id=self.group_id, resource_uri=self.subd_resources[i].uri):
                del self.subd_resources[i]

    def delete_group(self):
        self.logger.debug(f"Deleting  group-id {self.group_id}")
        try:
            if self.group_id:
                resp = self.client.session.delete(
                    self.host_url + "/subscription/" + self.group_id,
                    timeout=self.timeout,
                    verify=False,
                    headers=ACCEPT_HEADER,
                )
                if resp.status_code == 200:
                    self.logger.debug(f"Subscription deleted for group_id {self.group_id}.")
                    self.subd_resources = []

                else:
                    raise ValueError(f"status_code: {resp.status_code} Resp:{ resp.reason}")
            else:
                self.logger.debug("No subscription to delete.")

        except Exception as err:
            self.logger.error(f"subscription delete failed. Exception: {err}")

    def __del__(self) -> None:
        if not self.closed:
            self.close()

    def on_message(self, ws, message):
        resp_dict = xmltodict.parse(message)["html"]["body"]["div"]["ul"]["li"]

        def extract_val(element, values):
            if "@title" not in element:
                return None
            if "span" not in element:
                return None
            title = element["@title"]
            values_span = element["span"]
            if isinstance(values_span, list):
                for val in values_span:
                    values[val["@class"]] = val["#text"]
            elif isinstance(values_span, dict):
                values[values_span["@class"]] = values_span["#text"]
            else:
                # self.logger.error("No value found in subscription message.")
                return None
            return title

        values_ = {}
        if isinstance(resp_dict, list):
            for element in resp_dict:
                title = extract_val(element, values_)
        else:
            title = extract_val(resp_dict, values_)

        for res in self.subd_resources:
            if res.title == title:
                res.callback(values_)

        # for res in self.subd_resources:
        #     key_str = f'class="{res[1]}">'
        #     p_start = message.find(key_str)
        #     if p_start != -1:
        #         p_start += len(key_str)
        #         p_end = p_start + message[p_start:].find("</span>")
        #         val = message[p_start:p_end]
        #         self.logger.debug(f"{res[0]} changed to {val}")
        #         # Call the callback
        #         res[2](val)

    def on_error(self, ws, error):
        self.logger.error(f"Subscriber websocket error: {error}")

    def on_close(self, ws, ss, dd):
        self.logger.debug("Subscriber websocket closed.")
        self.unsubscribe_all()
        # self.delete_group()

    def on_open(self, ws):
        self.logger.debug("Subscriber websocket opened.")

    def createWebSocket(self, url, headers):

        self.logger.debug(f"Creating subscriber web socket to {url}")
        # headers["Sec-WebSocket-Protocol"] = "rws_subscription"

        self.ws = websocket.WebSocketApp(
            url,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
            subprotocols=[self.ws_sup_protocol],
            header=headers,
        )
        self.ws.on_open = self.on_open
        # websocket.enableTrace(True)
        self.ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
