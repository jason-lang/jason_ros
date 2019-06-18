// Agent sample_agent in project rosaria_agents

/* Initial beliefs and rules */

/* Initial goals */

!start.

/* Plans */

+!start : true
	<- 	cmd_vel(10,30,4,0,0,1);
		.wait(1000);
		cmd_vel(50,10,10,5,0,1);
		.wait(1000);
		cmd_vel(3,30,30,5,7,9). 
