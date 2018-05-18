import json
import numpy as np

from .frames import *
import numpy as np

class JSONEncoder(json.JSONEncoder):

    def default(self, obj):
        # if hasattr(obj, '__json__'):
        #     return obj.__json__()
        # return json.JSONEncoder.default(self, obj)
        if isinstance(obj, (AccelFrame, AngleFrame, MagneticFrame, AtmosphericFrame, AngularVelocityFrame)):
            return obj.__dict__
        elif isinstance(obj, np.ndarray):
            return list(obj)
        elif isinstance(obj, bytearray):
            return [int(x) for x in obj]
        return json.JSONEncoder.default(self, obj)
