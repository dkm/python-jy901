import json
import numpy as np

from .frames import *

class JSONEncoder(json.JSONEncoder):

    def default(self, obj):
        if isinstance(obj, AccelFrame):
            print("boom")
            return None
        return json.JSONEncoder.default(self, obj)
