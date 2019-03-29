import jason.asSyntax.*;
import jason.architecture.*;
import jason.asSemantics.*;
import java.util.List;
import java.util.ArrayList;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.NodeConfiguration;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;

import jason_msgs.Action;
import jason_msgs.ActionStatus;

public class RosJasonNode extends AbstractNodeMain {
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    Publisher<jason_msgs.Action> actionPub;
    // Literal perception = null;
    List<Literal> perception = new ArrayList<Literal>();
    // Map<String, Literal> perception = new HashMap<String, Literal>();

    List<jason_msgs.ActionStatus> actions_status = new ArrayList<jason_msgs.ActionStatus>();
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

		Subscriber<jason_msgs.ActionStatus> actionsStatusSub =
		connectedNode.newSubscriber("/jason/actions_status", jason_msgs.ActionStatus._TYPE);

    actionsStatusSub.addMessageListener(new MessageListener<jason_msgs.ActionStatus>() {
      @Override
      public void onNewMessage(jason_msgs.ActionStatus message) {
        actions_status.add(message);
      }
    });
    this.connected = true;
  }

  public  List<Literal> getPerception() { return perception; }

  public int publishAction(ActionExec action) {
    jason_msgs.Action act = actionPub.newMessage();

    std_msgs.Header header = nodeConfiguration.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
    act.setHeader(header);

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
    return header.getSeq();
  }

  public List<jason_msgs.ActionStatus> retrieveStatus() {
    List<jason_msgs.ActionStatus> aux = new ArrayList<jason_msgs.ActionStatus>(actions_status);
    actions_status.clear();
    return aux;
  }

  public boolean Connected() { return this.connected; }
}
