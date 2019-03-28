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

import jason_msgs.Action;

public class RosJasonNode extends AbstractNodeMain {
  Publisher<jason_msgs.Action> actionPub;
  // Literal perception = null;
  List<Literal> perception = new ArrayList<Literal>();
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
        connectedNode.newPublisher("/jason/actions", jason_msgs.Action._TYPE);

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

  public int publishAction(ActionExec action) {
    jason_msgs.Action act = actionPub.newMessage();
    act.setActionName(action.getActionTerm().getFunctor());

    if (action.getActionTerm().hasTerm()) {
        List<String> params = new ArrayList<String>();
        for (Term term : action.getActionTerm().getTerms()) {
            if(term.isString()){
                params.add(((StringTerm)term).getString());
            }else{
                params.add(String.valueOf(term));
            }
        }
        act.setParameters(params);
    }
    actionPub.publish(act);
    std_msgs.Header header = act.getHeader();
    return header.getSeq();
  }

  public List<String> retrieveStatus() {
    List<String> aux = new ArrayList<String>(actions_status);
    actions_status.clear();
    return aux;
  }

  public boolean Connected() { return this.connected; }
}
