#Python 2/3 compatibility with 3 style code
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import ctypes
import threading
try:
    import queue
except ImportError:
    import Queue as queue

class MessageQueue:
    "Queue up received messages and invoke notification to run callback"
    def __init__(self, cbptr, notify_handle):
        CBType = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p)
        self._cb_notify = CBType(cbptr.value)
        self._notify_handle = notify_handle

        self._queue = queue.Queue()

    def storemsg(self, msg):
        self._queue.put(msg)
        self._cb_notify(self._notify_handle)

    def size(self):
        return self._queue.qsize()

    def get(self):
        return self._queue.get()

class ServiceCallback:
    def __init__(self, cbptr, notify_handle):
        CBType = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p)
        self._cb_notify = CBType(cbptr.value)
        self._notify_handle = notify_handle

        self._response = None
        self._hasresponse = threading.Condition()

    def srv_cb(self, srvreq):
        "Store received service request and block until Julia callback completes"
        self._hasresponse.acquire()
        self._request = srvreq
        self._cb_notify(self._notify_handle)

        #wait for the julia callback
        self._hasresponse.wait()
        self._hasresponse.release()
        return self._response

    def get_request(self):
        return self._request

    def set_response(self, resp):
        self._response = resp
        self._hasresponse.acquire()
        self._hasresponse.notify()
        self._hasresponse.release()
