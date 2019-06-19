// Agent sample_agent in project rosaria_agents

/* Initial beliefs and rules */

/* Initial goals */

!start.

/* Plans */

+!start : true
	<- 	enable_motors;
		.wait(motors_state("True"));
		!move.


+!move : motors_state("True")
	<- 	cmd_vel(10,30,4,0,0,1);
		.wait(2000);
		cmd_vel(50,10,10,5,0,1);
		.wait(2000);
		cmd_vel(3,30,30,5,7,9);
		.wait(2000);
		disable_motors.

-!move <- .print("Can't move").
