import jason.asSyntax.*;
import jason.architecture.*;
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
    }

    @Override
    public List<Literal> perceive() {
      List<Literal> per = new ArrayList<Literal>();
      if(perception!=null){
        per.add(perception);
      }
      return per;
    }
}
