# BayesOpt ROS Wrapper

A simple ROS wrapper for the Bayesian optimization library [BayesOpt](https://github.com/rmcantin/bayesoptpro)

Tested in ROS Melodic with Python 3.6.9.

## Requirements

- Python 3
- rospy
- numpy
- BayesOpt (with Python interface)

## How to use

The wrapper is launched with script _bayesopt-ros.py_, which takes as input a configuration file described in section [Configuration](#configuration).

```bash
python3 bayesopt-ros.py -fparams <params_file>
```

This script creates a node ("bayesopt") and will wait for the _evaluation_ service, specified in the configuration file, until it is available.
The _evaluation_ service defines the function to be optimized and must be created by the user, will receive as request the query to be evaluated and will return as response a scalar.
The service specification is defined in _EvaluateQuery.srv_.

The final result of the optimization process will be displayed on the screen.
It is also possible to save the result in a specified file or enable the topic "/bayesopt/result" to read the result in an _OptimizationResult.msg_ message.

A simple example of the use of this wrapper can be found in the _example_ folder.

## Configuration

A json file that defines the paramters as follows:

- **ros_params:** ROS node parameters
  - **evaluation_service:** String, name of the evaluation service
  - **res_file:** String, file to save the results \[OPTIONAL\].
  - **res_topic_enabled:** Bool, _true_ to enable the result topic
- **search_space_params:** defines the limits of the search space of the function to be optimized.
  - **lower_bound:** Array of doubles, lower bound of each dimension.
  - **upper_bound:** Array of doubles, upper bound of each dimension.
- **optimizer_params:** optimization parameters \[OPTIONAL\]. If not specified, it takes the default values. For a description of the parameters, see the [BayesOpt guide](http://rmcantin.github.io/bayesopt/html/usemanual.html).
