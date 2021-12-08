package jasonros;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import jason.architecture.*;
import jason.asSemantics.*;
import jason.asSyntax.*;
import jason_ros_msgs.Action;
import jason_ros_msgs.ActionStatus;
import jason_ros_msgs.Message;
import jason_ros_msgs.Perception;

public class RosJasonNode extends AbstractNodeMain {
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    ParameterTree parameterTree;
    // String namespace = "";
    Publisher<jason_ros_msgs.Action> actionPub;
    Publisher<jason_ros_msgs.Message> msgPub;

    ConcurrentLinkedQueue<jason_ros_msgs.Perception> perceptionQueue =
            new ConcurrentLinkedQueue<jason_ros_msgs.Perception>();
    ConcurrentLinkedQueue<jason_ros_msgs.Message> messageQueue =
            new ConcurrentLinkedQueue<jason_ros_msgs.Message>();

    List<jason_ros_msgs.ActionStatus> actions_status = new ArrayList<jason_ros_msgs.ActionStatus>();
    boolean connected;
    String agent_name = "";

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("jason/agent");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        parameterTree = connectedNode.getParameterTree();

        actionPub = connectedNode.newPublisher("jason/actions", jason_ros_msgs.Action._TYPE);

        msgPub = connectedNode.newPublisher("jason/send_msg", jason_ros_msgs.Message._TYPE);

        Subscriber<jason_ros_msgs.Perception> perceptsSub =
                connectedNode.newSubscriber("jason/percepts", jason_ros_msgs.Perception._TYPE);

        perceptsSub.addMessageListener(new MessageListener<jason_ros_msgs.Perception>() {
            @Override
            public void onNewMessage(jason_ros_msgs.Perception message) {
                perceptionQueue.offer(message);
            }
        });

        Subscriber<jason_ros_msgs.ActionStatus> actionsStatusSub =
                connectedNode.newSubscriber("jason/actions_status", jason_ros_msgs.ActionStatus._TYPE);

        actionsStatusSub.addMessageListener(new MessageListener<jason_ros_msgs.ActionStatus>() {
            @Override
            public void onNewMessage(jason_ros_msgs.ActionStatus message) {
                actions_status.add(message);
            }
        });

        Subscriber<jason_ros_msgs.Message> msgSub =
                connectedNode.newSubscriber("jason/receive_msg", jason_ros_msgs.Message._TYPE);

        msgSub.addMessageListener(new MessageListener<jason_ros_msgs.Message>() {
            @Override
            public void onNewMessage(jason_ros_msgs.Message message) {
                messageQueue.offer(message);
            }
        });

        this.connected = true;
    }

    public jason_ros_msgs.Perception getPerception() {
        return perceptionQueue.poll();
    }
    public jason_ros_msgs.Message getMessage() {
        return messageQueue.poll();
    }

    public int publishAction(ActionExec action) {
        jason_ros_msgs.Action act = actionPub.newMessage();

        std_msgs.Header header =
                nodeConfiguration.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
        act.setHeader(header);

        act.setAgentName(this.agent_name);
        act.setActionName(action.getActionTerm().getFunctor());

        if (action.getActionTerm().hasTerm()) {
            List<String> params = new ArrayList<String>();
            for (Term term : action.getActionTerm().getTerms()) {
                if (term.isString()) {
                    params.add(((StringTerm) term).getString());
                } else {
                    params.add(String.valueOf(term));
                }
            }
            act.setParameters(params);
        }

        actionPub.publish(act);
        return header.getSeq();
    }

    public void publishMessage(jason.asSemantics.Message m) {
        jason_ros_msgs.Message msg = msgPub.newMessage();

        std_msgs.Header header =
                nodeConfiguration.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
        msg.setHeader(header);

        msg.setData(m.toString());

        msgPub.publish(msg);
    }

    public List<jason_ros_msgs.ActionStatus> retrieveStatus() {
        List<jason_ros_msgs.ActionStatus> aux = new ArrayList<jason_ros_msgs.ActionStatus>(actions_status);
        actions_status.clear();
        return aux;
    }

    public boolean Connected() {
        return this.connected;
    }

    public void setNameParameter(String agent_name){
      this.agent_name = agent_name;
      parameterTree.set("jason/agent_name", agent_name);
    }
}
