#!/usr/bin/env python3

import numpy as np

import rospy

from bayesopt_ros.srv import EvaluateQuery, EvaluateQueryResponse

def evaluate_query(request: EvaluateQuery):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    query = np.array(request.query)
    res = np.sum(query)
    print("query:", query, " | outcome:", res)
    return EvaluateQueryResponse(res)

if __name__ == "__main__":
    rospy.init_node('test_srv_executor')
    s = rospy.Service('eval_service', EvaluateQuery, evaluate_query)
    print("Test srv executor ready")
    rospy.spin()
