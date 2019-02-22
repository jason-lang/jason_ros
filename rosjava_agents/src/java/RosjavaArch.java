import jason.asSyntax.*;
import jason.architecture.*;
import jason.asSemantics.*;

import java.util.List;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.Map;
import java.util.HashMap;

public class RosjavaArch extends AgArch{
	RosJasonNode rosNode;
	Map<String, ActionExec> actionsWaiting = new HashMap<String,ActionExec>();

	@Override
    public void reasoningCycleStarting() {
    	List<String> actions_status = rosNode.retrieveStatus();
		for(String status : actions_status){
			actionsStatus(status);
		}
    	super.reasoningCycleStarting();
    }

	@Override
	public List<Literal> perceive() {
	  List<Literal> per = new ArrayList<Literal>();
	  Literal perception = rosNode.getPerception();
	  if(perception!=null){
		per.add(perception);
	  }
	  return per;
	}

	@Override
    public void act(ActionExec action) {
        String action_string = actionToString(action);
        rosNode.publishAction(action_string);

        actionsWaiting.put(getPredicate(action_string), action); //TODO: Needs to be improved
    }

	public String actionToString(ActionExec action){
	 String s = action.getActionTerm().getFunctor();
	 List<Term> terms = action.getActionTerm().getTerms();
	   if (action.getActionTerm().hasTerm()){
		 s = s + "(";
		 for(Term term : terms){
		   if(term.isString()){
			 s = s + ((StringTerm) term).getString() + ",";
		   } else if(term.isNumeric()){
			 try{
			   s = s + Double.toString(((NumberTerm) term).solve()) + ",";
			 } catch(Exception e) {e.printStackTrace();}
		   }
		 }
		 s = s.substring(0, s.length()-1) + ")"; //take last ',' out and close with ')'
	   }
	 return s;
   }

   public void actionsStatus(String data){
	 String action_status = getPredicate(data);
	 String action_name = getArgs(data);

	 if (action_status!=null && action_name!=null) {
	   // System.out.println("Action: " + action_name + " Status: " + action_status);
	   ActionExec action = actionsWaiting.get(action_name);

	   if(action != null){
		 if(action_status.equals("done")){
		   action.setResult(true);
		   actionExecuted(action);
		 }else if(action_status.equals("fail")){
		   action.setResult(false);
		   actionExecuted(action);
		 }else{
		   System.out.println("Status" + action_status + "unknown. Setting action "+action_name + " as failed!");
		   action.setResult(false);
		   actionExecuted(action);
		 }
		 actionsWaiting.remove(action_name);
	   }else{
		 System.out.println("Action "+action_name+" not found.");
	   }
	 }else {
	   System.out.println("Couldn't confirm action status");
	 }

   }

   public String getPredicate(String data){
	 Pattern p = Pattern.compile("[^(]*");
	 Matcher m = p.matcher(data);
	 String predicate = null;
	 if(m.find()){
	   predicate = m.group(0);
	 }
	 return predicate;
   }

   public String getArgs(String data){
	 Pattern p = Pattern.compile("\\((.*?)\\)");
	 Matcher m = p.matcher(data);
	 String args = null;
	 if(m.find()){
	   args = m.group(1);
	 }
	 return args;
   }
}
