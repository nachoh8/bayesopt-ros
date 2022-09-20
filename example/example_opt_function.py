#!/usr/bin/env python3

import numpy as np

import rospy

from bayesopt_ros.srv import EvaluateQuery, EvaluateQueryResponse

def testfunc(Xin):
    total = 5.0
    for value in Xin:
        total = total + (value - 0.33) * (value - 0.33)

    return total

def evaluate_query(request: EvaluateQuery):
    ''' 
    Callback function used by the service server to process
    requests from clients.
    '''
    query = np.array(request.query)
    res = testfunc(query)
    print("Query:", query, " | Outcome:", res)
    return EvaluateQueryResponse(res)

if __name__ == "__main__":
    rospy.init_node('example_opt_function')
    s = rospy.Service('example_evaluation_service', EvaluateQuery, evaluate_query)
    print("Evaluation service ready!!!")
    rospy.spin()
