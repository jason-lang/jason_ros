// Agent sample_agent in project multiple_ags

/* Initial beliefs and rules */

/* Initial goals */

!start.

/* Plans */

+!start : true <-
	teste("oi, sou agente 2");
	.print("hello world.");
	.send("bob", tell, oi("oi")).
