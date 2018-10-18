import jason.asSyntax.*;
import jason.architecture.*;
import java.util.*;

import com.fasterxml.jackson.databind.JsonNode;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;

public class rosbridgeArch extends AgArch {

    @Override
    public List<Literal> perceive() {

      // gets the default perception
      // List<Literal> per = (List<Literal>) super.perceive();
      List<Literal> per = new ArrayList<Literal>();
      Literal lit = Literal.parseLiteral("p(a)");

      per.add(lit);
      System.out.println(per);
      return per;
    }
}
