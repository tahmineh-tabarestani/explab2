My Moveit Node
-----------------------

Implements a node to control the arm of robot using moveit

Services:
 /move_arm ---> SetBool (Request, Response)

Description:

There are seven boxes in the environment, each box conteins one id and each id identify one room with some information.
Defined various joints configuration to get correct position in order to find marker_id designed on 
the boxes. Once the ``move_arm`` service is called with arg *True*, ``reach(req, resp)`` function
tries to find the plan to the corresponding joints configuration and executes it.

There is a function for planning and executing it to each joints configuration. It uses the provided methods
from ``moveit`` package for loading robot model, its kinematic and joints group. 
