// Agent sample_agent in project turtlesim_agent

/* Initial beliefs and rules */

/* Initial goals */

!start.

/* Plans */

+!start : true <-
	.print("I`m a turtle. Let`s go around!");
	cmd_vel(2.0, 0.0, 0.0, 0.0, 0.0, 1.8);
	.wait(2000);
	cmd_vel(2.0, 1.0, 0.0, 0.0, 0.0, 1.5);
	.wait(2000);
	cmd_vel(2.0, 1.0, 1.0, 3.0, 0.0, 1.5);
	.wait(2000);
	.print("Finished!");
	.
