#!/usr/bin/env python3

import argparse
import json
from datetime import datetime
import numpy as np

import rospy
from bayesopt_ros.srv import EvaluateQuery
from bayesopt_ros.msg import OptimizationResult

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
        self.res_file = params['res_file'] if 'res_file' in params else ''
        res_topic_enabled = params['res_topic_enabled']

        print("Waiting for Evaluation Service: " + str(srv_name))
        
        rospy.wait_for_service(srv_name)
        self.eval_query_srv = rospy.ServiceProxy(srv_name, EvaluateQuery, persistent=True)

        if res_topic_enabled:
            self.res_topic = rospy.Publisher('bayesopt/result', OptimizationResult)
        else:
            self.res_topic = None
        
    def evaluateSample(self, x_in: np.ndarray):
        res = self.eval_query_srv(x_in.tolist())
        outcome = res.outcome

        return outcome
    
    def run(self):
        print("Optimization start")
        print("----------------------------")
        
        try:
            res, q, _ = self.optimize()
        except rospy.ROSInterruptException as ros_err:
            print("Error - ROS interruption: %s"%ros_err)
            exit(-1)
        except rospy.ServiceException as srv_err:
            print("Error - Service call failed: %s"%srv_err)
            exit(-2)

        print("----------------------------")
        print("Optimization finished")
        print("Best query:", q)
        print("Outcome:", res)
        
        self.eval_query_srv.close()

        if self.res_topic is not None:
            print("----------------------------")
            print("Publishing result to " + self.res_topic.name + " topic")
            
            msg = OptimizationResult()
            msg.date = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
            msg.best_query = q.tolist()
            msg.outcome = res
            
            self.res_topic.publish(msg)

        if self.res_file != "":
            print("----------------------------")
            print("Saving result to " + self.res_file)

            data = {"date": datetime.now().strftime("%d/%m/%Y %H:%M:%S"), "query": q.tolist(), "outcome": res}
            json_str = json.dumps(data, indent=4)
            with open(self.res_file, 'w') as outfile:
                outfile.write(json_str)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='BayesOpt ROS wrapper')
    parser.add_argument("-fparams", type=str, help="optimizer params", metavar='<params_file>', required=True)

    args = parser.parse_args()

    f = open(args.fparams, 'r')
    params: dict = json.load(f)

    optimizer = BayesOptROS(params)
    optimizer.run()
