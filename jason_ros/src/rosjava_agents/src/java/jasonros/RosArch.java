package jasonros;

import static jason.asSyntax.ASSyntax.*;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.ros.exception.RosRuntimeException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.namespace.GraphName;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import jason.RevisionFailedException;
import jason.architecture.*;
import jason.asSemantics.*;
import jason.asSyntax.*;
import jason.asSyntax.Trigger.TEOperator;
import jason.asSyntax.Trigger.TEType;
import jason.asSyntax.parser.ParseException;
import jason.asSyntax.parser.TokenMgrError;
import jason_ros_msgs.ActionStatus;
import jason_ros_msgs.Message;
import jason_ros_msgs.Perception;

public class RosArch extends AgArch {
    jasonros.RosJasonNode rosNode;
    Map<Integer, ActionExec> actionsWaiting = new HashMap<Integer, ActionExec>();

    @Override
    public void init() throws Exception {
        CommandLineLoader loader =
                new CommandLineLoader(Lists.newArrayList("jasonros.RosJasonNode"));
        String nodeClassName = loader.getNodeClassName();
        System.out.println("Loading node class: " + loader.getNodeClassName());
        NodeConfiguration nodeConfiguration = loader.build();
        nodeConfiguration.setNodeName(GraphName.of("jason/agent/" + this.getAgName()));
        NodeMain nodeMain = null;

        try {
            nodeMain = (jasonros.RosJasonNode) loader.loadClass(nodeClassName);
        } catch (ClassNotFoundException e) {
            throw new RosRuntimeException("Unable to locate node: " + nodeClassName, e);
        } catch (InstantiationException e) {
            throw new RosRuntimeException("Unable to instantiate node: " + nodeClassName, e);
        } catch (IllegalAccessException e) {
            throw new RosRuntimeException("Unable to instantiate node: " + nodeClassName, e);
        }

        Preconditions.checkState(nodeMain != null);
        NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(nodeMain, nodeConfiguration);

        rosNode = (jasonros.RosJasonNode) nodeMain;
        while (!rosNode.Connected()) Thread.sleep(1000);
        rosNode.setNameParameter(this.getAgName());
    }

    @Override
    public void reasoningCycleStarting() {
        List<jason_ros_msgs.ActionStatus> actions_status = rosNode.retrieveStatus();
        for (jason_ros_msgs.ActionStatus status : actions_status) {
            actionsStatus(status);
        }
        super.reasoningCycleStarting();
    }

    @Override
    public List<Literal> perceive() {
        jason_ros_msgs.Perception perception = rosNode.getPerception();
        while (perception != null) {
            String agent_name = perception.getAgentName();
            if(agent_name.equals(this.getAgName()) || agent_name.isEmpty()){
              TransitionSystem ts = getTS();
              Literal bel = createLiteral(perception.getPerceptionName());
              bel.addAnnot(ts.getAg().getBB().TPercept);

              for (String param : perception.getParameters()) {
                bel.addTerm(parsePerception(param));
              }

              boolean bufUpdate = perception.getUpdate();
              if (bufUpdate) {
                Iterator<Literal> ibb =
                ts.getAg().getBB().getCandidateBeliefs(new PredicateIndicator(
                perception.getPerceptionName(), perception.getParameters().size()));
                boolean addBelief = true;
                while (ibb != null && ibb.hasNext()) {
                  Literal l = ibb.next();
                  if (l.equals(bel)) {
                    addBelief = false;
                  } else {
                    ibb.remove(); // remove l as perception from BB

                    // only produce -bel event if the agent has plans for the event
                    Trigger te = new Trigger(TEOperator.del, TEType.belief, l);
                    if (ts.getC().hasListener() || ts.getAg().getPL().hasCandidatePlan(te)) {
                      l = ASSyntax.createLiteral(l.getFunctor(), l.getTermsArray());
                      l.addAnnot(ts.getAg().getBB().TPercept);
                      te.setLiteral(l);
                      ts.getC().addEvent(new Event(te, Intention.EmptyInt));
                    }
                  }
                }
                if (addBelief) {
                  try {
                    ts.getAg().addBel(bel);
                  } catch (RevisionFailedException e) {
                    System.out.println("Error adding new belief");
                  }
                }
              } else {
                try {
                  getTS().getAg().addBel(bel);
                } catch (RevisionFailedException e) {
                  System.out.println("Error adding new belief");
                }
              }
            }
            perception = rosNode.getPerception();
        }
        return null;
    }

    private boolean isVector(String str){
      return (str.charAt(0)=='[' && str.charAt(str.length()-1) == ']');
    }

    private Term parsePerception(String perception){
      if(isVector(perception)){
        return string2ListTerm(perception);
      }else{
        return parseString2TermOrString(perception);
      }
    }

    private Term parseString2TermOrString(String str){
      Term t;
      try {
        t = parseTerm(str);
        if (t.isVar()) {
          t = new StringTermImpl(str);
        }
      } catch (ParseException | TokenMgrError e) {
        t = new StringTermImpl(str);
      }
      return t;
    }

    private ListTerm string2ListTerm(String vector){
      // List of terms that will be returned
      ListTerm term_list = new ListTermImpl();

      // remove first and last brackets
      int len = vector.length();
      String inner_elem_raw = vector.substring(1, len-1);

      // get all the vector elements. (split string by commas outside brackets)
      ArrayList<String> inner_elems = split_str_comma_brackets(inner_elem_raw);

      // for each element of vector check if it is another vector
      for (int i=0; i<inner_elems.size(); i++) {
        //if element is vector call function recursively and append to term_list
        if(isVector(inner_elems.get(i))){
          term_list.append(string2ListTerm(inner_elems.get(i)));
        }else{ //else append directly to term_list
          term_list.append(parseString2TermOrString(inner_elems.get(i)));
        }
      }
      return term_list;
    }

    private ArrayList<String> split_str_comma_brackets(String str){
      int open_brackets = 0;
      int last_comma_index = -1;
      ArrayList<String> splitted_str = new ArrayList<String>();
      for (int i = 0; i<str.length(); i++) {
        if (str.charAt(i) == '['){
          open_brackets += 1;
        }else if(str.charAt(i) == ']'){
          open_brackets -= 1;
        }else if(str.charAt(i) == ',' && open_brackets == 0){
          String sub_string = str.substring(last_comma_index+1, i);
          splitted_str.add(sub_string.trim());
          last_comma_index = i;
        }
      }
      String sub_string = str.substring(last_comma_index+1);
      splitted_str.add(sub_string.trim());
      return splitted_str;
    }

    @Override
    public void act(ActionExec action) {
        int seq = rosNode.publishAction(action);

        actionsWaiting.put(seq, action);
    }

    public void actionsStatus(jason_ros_msgs.ActionStatus data) {
        String agent_name = data.getAgentName();
        if(agent_name.equals(this.getAgName())){
          int id = data.getId();
          boolean result = data.getResult();

          ActionExec action = actionsWaiting.get(id);

          if (action != null) {
            action.setResult(result);
            actionExecuted(action);
            actionsWaiting.remove(id);
          } else {
            System.out.println("Action not found.");
          }
        }
    }

    @Override
    public void sendMsg(jason.asSemantics.Message m) {
        rosNode.publishMessage(m);
    }

    @Override
    public void broadcast(jason.asSemantics.Message m) {
        rosNode.publishMessage(m);
    }

    @Override
    public void checkMail() {
        jason_ros_msgs.Message rosmsg = rosNode.getMessage();
        Circumstance C = getTS().getC();
        while (rosmsg != null) {
            try {
                jason.asSemantics.Message im = jason.asSemantics.Message.parseMsg(rosmsg.getData());
                C.addMsg(im);
            } catch (ParseException e) {
                System.out.println("No message added.");
            }

            rosmsg = rosNode.getMessage();
        }
    }
}
