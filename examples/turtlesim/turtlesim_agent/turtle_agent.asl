// Agent sample_agent in project turtlesim_agent

/* Initial beliefs and rules */

/* Initial goals */

!start.

/* Plans */

+!start : true <-
	.print("I like red pen!")
	set_pen(255,0,0);
	.print("I`m a turtle. Let`s go around!");
	cmd_vel(2.0, 0.0, 0.0, 0.0, 0.0, 1.8);
	.wait(2000);
	cmd_vel(5.0, 1.0, 0.0, 2.0, 0.0, 1.5);
	.wait(2000);
	.print("I'm feeling artistic, let's go with white pen!")
	set_pen(255,255,255);
	cmd_vel(1.0, 5.0, 2.0, 3.0, 2.0, 1.5);
	.wait(2000);
	cmd_vel(1.0, 0.0, 4.0, 1.0, 1.0, 0.5);
	.wait(2000);
	.print("Finished!");
	.
