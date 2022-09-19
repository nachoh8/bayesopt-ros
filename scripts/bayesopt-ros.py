#!/usr/bin/env python3

import argparse
import json
import numpy as np

import rospy
from bayesopt_ros.srv import EvaluateQuery, EvaluateQueryResponse

from bayesoptmodule import BayesOptContinuous


class BayesOptROS(BayesOptContinuous):
    def __init__(self, params: dict):
        self.create_node(params['ros_params'])
        print("----------------------------")
        self.create_optimizer(params)
        print("----------------------------")
    
    def create_optimizer(self, params: dict):
        """
        Create BayesOpt Continuous optimizer from params
        """
        print("Creating BayesOpt Optimizer...")

        SEARCH_SPACE_KEY = 'search_space_params'
        OPTIMIZER_KEY = 'optimizer_params'

        lb = np.array(params[SEARCH_SPACE_KEY]['lower_bound'], dtype=np.float64)
        ub = np.array(params[SEARCH_SPACE_KEY]['upper_bound'], dtype=np.float64)
        assert(lb.shape[0] == ub.shape[0])

        if OPTIMIZER_KEY in params:
            opt_params = params[OPTIMIZER_KEY]
        else:
            opt_params = {}

        super().__init__(lb.shape[0])
        self.lb = lb
        self.ub = ub
        self.params = opt_params

    def create_node(self, params: dict):
        """
        Create the ROS Node and Wait for Service
        """
        rospy.init_node('bayesopt')
        
        srv_name = params['evaluation_service']
        print("Waiting for Service: " + str(srv_name))
        rospy.wait_for_service(srv_name)
        self.eval_query_srv = rospy.ServiceProxy(srv_name, EvaluateQuery, persistent=True)
        
    def evaluateSample(self, x_in: np.ndarray):
        try:
            res = self.eval_query_srv(x_in.tolist())
            outcome = res.outcome
        except rospy.ServiceException as err:
            print("Service call failed: %s"%err)
        
        print("Outcome: " + str(outcome))

        return outcome
    
    def run(self):
        print("Optimization start")
        print("----------------------------")
        
        y, x, err = optimizer.optimize()
        print(y, x, err)
        
        self.eval_query_srv.close()

        # TODO: write result to service, topic, file,...?


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='BayesOpt ROS wrapper')
    parser.add_argument("-fparams", type=str, help="optimizer params", metavar='<params_file>', required=True)

    args = parser.parse_args()

    f = open(args.fparams, 'r')
    params: dict = json.load(f)

    optimizer = BayesOptROS(params)
    optimizer.run()
    

    """try:
        main()
    except rospy.ROSInterruptException as err:
        print("Error: ")"""
