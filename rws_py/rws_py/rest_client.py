from dataclasses import dataclass, field
from typing import Dict, Optional

import requests
import xmltodict
from requests.auth import HTTPBasicAuth, HTTPDigestAuth

from rws_py.rws_config import RWSConfig

import logging
logger = logging.getLogger(__name__)

@dataclass
class RESTResponse:
    status_code: int = -1
    reason: str = ""
    data: Dict = field(default_factory=dict)
 

def get_encoding(msg: str) -> Optional[str]:
    if hasattr(msg, "encoding") and msg.encoding is not None:
        return msg.encoding
    elif hasattr(msg, "apparent_encoding") and msg.apparent_encoding is not None:
        return msg.apparent_encoding
    return None


class RESTClient:
    conf: RWSConfig
    timeout: float = 2.0
    session: requests.sessions.Session = None
    headers = {}
    POST_HEADER = {"content-type": "application/x-www-form-urlencoded;v=2.0"}
    PUT_HEADER = {"content-type": "text/plain;v=2.0"}
    ACCEPT_HEADER_XML = {"Accept": "application/xhtml+xml;v=2.0"}
    ACCEPT_HEADER_JSON = {"Accept": "application/hal+json;v=2.0"}

    def __init__(self, config: RWSConfig = RWSConfig(6)) -> None:
        self.conf = config
        if self.conf.auth == "digest":
            self.auth = HTTPDigestAuth(self.conf.user, self.conf.password)
        elif self.conf.auth == "basic":
            self.auth = HTTPBasicAuth(self.conf.user, self.conf.password)
        else:
            logger.warning("Unknown auth. Setting digest as default.")
            self.auth = HTTPDigestAuth(self.conf.user, self.conf.password)

        self.conf.host_url = f"{self.conf.protocol}://{self.conf.host_ip}:{self.conf.port}"
        self.conf.host_ws_url = f"{self.conf.ws_protocol}://{self.conf.host_ip}:{self.conf.ws_port}"

    def login(self):
        self.session = requests.session()
        try:
            resp = self.session.get(
                self.conf.host_url + "/rw",
                auth=self.auth,
                headers=self.ACCEPT_HEADER_XML,
                timeout=self.timeout,
                verify=False,
            )

            if resp.status_code == 200:
                logger.debug("Login successful")
                abbcx = resp.cookies["ABBCX"]
                session = resp.cookies["-http-session-"]
                self.headers["Cookie"] = "-http-session-=" + session + "; ABBCX=" + abbcx
            else:
                logger.error(f"Status code:{resp.status_code}\nResponse:{resp.reason}")
            return True
        except Exception as err:
            logger.error(f"Login failed. Check rw_version, ip, port, and ethernet cable. \n{err}")
            return False

    def logout(self):
        resp = self.call_api("/logout", "GET")
        if resp.status_code == 200 or resp.status_code == 204:
            logger.info("Logout successful.")
        else:
            logger.error(f"Logout failed. Reason: {resp.reason}")

        self.session.close()

    def call_api(
        self,
        url: str,
        method: str,
        headers: Optional[str] = None,
        params: Optional[Dict] = None,
    ) -> RESTResponse:
        response = RESTResponse()
        headers = self.ACCEPT_HEADER_XML
        if params is None:
            params = {}
        if method == "GET":
            headers = self.ACCEPT_HEADER_JSON
            resp = self.session.get(
                self.conf.host_url + url,
                auth=self.auth,
                headers=headers,
                timeout=self.timeout,
                verify=False,
                params={"json": 1},
            )
            response.status_code = resp.status_code
            response.reason = resp.reason

            if resp.status_code == 200 or resp.status_code == 204:
                if "Content-Type" in resp.headers:
                    if resp.headers["Content-Type"] in [
                        "application/hal+json;v=2.0",
                        "application/json",
                    ]:
                        try:
                            response.data = resp.json()
                        except ValueError:
                            try:
                                encoding = get_encoding(resp)
                                if encoding is not None:
                                    response.data = xmltodict.parse(resp.content.decode(encoding))
                                else:
                                    response.data = resp.content
                            except Exception:
                                return response
                    elif resp.headers["Content-Type"] in [
                        "text/plain",
                        "application/octet-stream",
                    ]:
                        encoding = get_encoding(resp)
                        if encoding is not None:
                            response.data = resp.content.decode(encoding)
                        else:
                            response.data = resp.content

            return response

        elif method == "POST":
            headers["Content-Type"] = "application/x-www-form-urlencoded;v=2.0"
            resp = self.session.post(
                self.conf.host_url + url,
                headers=headers,
                timeout=self.timeout,
                verify=False,
                data=params,
            )
            return resp

        elif method == "PUT":
            headers["Content-Type"] = "text/plain;v=2.0"
            resp = self.session.put(
                self.conf.host_url + url,
                headers=headers,
                timeout=self.timeout,
                verify=False,
                data=params,
            )
            return resp
        elif method == "DELETE":
            return self.session.delete(
                self.conf.host_url + url,
                headers=headers,
                timeout=self.timeout,
                verify=False,
            )
