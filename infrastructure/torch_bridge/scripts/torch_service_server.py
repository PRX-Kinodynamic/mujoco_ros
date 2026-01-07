import glob
import traceback

import os
import rospy
# import torch_bridge.TorchQuery
import numpy as np
from torch_bridge.srv import TorchQuery, TorchQueryResponse
class TorchServer(object):

    def __init__(self):

        self.service_name = rospy.get_param("~service_name", "")

        s = rospy.Service(self.service_name, TorchQuery, self.service_callback)

    def service_callback(self, req):
        response = TorchQueryResponse();
        response.outputs = 1
        response.output_dimensions = [3]
        response.result = [0,1,2] # Dummy output

        if req.compute_jacobians:
            # Some dummy jacobians
            dres_dx = [0,1,2,3,4,5,6,7,8]
            dres_du = [0,1,2,3,4,5]
            response.jacobians = dres_dx + dres_du
        return response

        # print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
        # return AddTwoIntsResponse(req.a + req.b)
if __name__ == '__main__':
    rospy.init_node("TorchServer")
    ros_node = TorchServer()
    rospy.spin()