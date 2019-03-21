import jason.asSyntax.*;
import jason.architecture.*;
import jason.asSemantics.*;
import java.util.List;
import java.util.ArrayList;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;

public class RosJasonNode extends AbstractNodeMain {
  Publisher<std_msgs.String> actionPub;
  // Literal perception = null;
  List<Literal> perception = new List<Literal>();
  // Map<String, Literal> perception = new HashMap<String, Literal>();

  List<String> actions_status = new ArrayList<String>();
  boolean connected;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("jason/agent");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    actionPub =
        connectedNode.newPublisher("/jason/actions", std_msgs.String._TYPE);

    Subscriber<std_msgs.String> perceptsSub =
        connectedNode.newSubscriber("/jason/percepts", std_msgs.String._TYPE);

    perceptsSub.addMessageListener(new MessageListener<std_msgs.String>() {
      @Override
      public void onNewMessage(std_msgs.String message) {
         Literal new_perception = Literal.parseLiteral(message.getData());
         perception.add(new_perception);
        // perception.put(getPredicate(new_perception), new_perception);
      }
    });

		Subscriber<std_msgs.String> actionsStatusSub =
		connectedNode.newSubscriber("/jason/actions_status", std_msgs.String._TYPE);

    actionsStatusSub.addMessageListener(new MessageListener<std_msgs.String>() {
      @Override
      public void onNewMessage(std_msgs.String message) {
        actions_status.add(message.getData());
      }
    });
    this.connected = true;
  }

  public  List<Literal> getPerception() { return perception; }

  public void publishAction(String action) {
    std_msgs.String str = actionPub.newMessage();
    str.setData(action);
    actionPub.publish(str);
  }

  public List<String> retrieveStatus() {
    List<String> aux = new ArrayList<String>(actions_status);
    actions_status.clear();
    return aux;
  }

  public boolean Connected() { return this.connected; }
}
