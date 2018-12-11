import jason.asSyntax.*;
import jason.architecture.*;
import jason.asSemantics.*;
import java.util.*;

import com.fasterxml.jackson.databind.JsonNode;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;
import ros.SubscriptionRequestMsg;

public class rosbridgeArch extends AgArch {

    RosBridge bridge = new RosBridge();
    Literal perception = null;
    Publisher pub;

    @Override
    public void init(){
      bridge.connect("ws://master:9090", true);
      bridge.subscribe(SubscriptionRequestMsg.generate("/jason/percepts")
                .setType("std_msgs/String")
                .setThrottleRate(1)
                .setQueueLength(1),
        new RosListenDelegate() {
              public void receive(JsonNode data, String stringRep) {
                      MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(PrimitiveMsg.class);
                      PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                      perception = Literal.parseLiteral(msg.data);
              }
        }
      );
      pub = new Publisher("/jason/actions", "std_msgs/String", bridge);
    }

    @Override
    public List<Literal> perceive() {
      List<Literal> per = new ArrayList<Literal>();
      if(perception!=null){
        per.add(perception);
      }
      return per;
    }

    @Override
    public void act(ActionExec action) {
        String action_string = actionToString(action);
        pub.publish(new PrimitiveMsg<String>(action_string));

        action.setResult(true);
        actionExecuted(action);
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
}
