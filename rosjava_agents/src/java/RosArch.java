import jason.asSyntax.*;
import jason.architecture.*;
import jason.asSemantics.*;

import java.util.List;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.Map;
import java.util.HashMap;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.ros.exception.RosRuntimeException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import jason_msgs.ActionStatus;

public class RosArch extends AgArch {
  RosJasonNode rosNode;
  Map<Integer, ActionExec> actionsWaiting = new HashMap<Integer, ActionExec>();

  @Override
  public void init() throws Exception {
    CommandLineLoader loader =
        new CommandLineLoader(Lists.newArrayList("RosJasonNode"));
    String nodeClassName = loader.getNodeClassName();
    System.out.println("Loading node class: " + loader.getNodeClassName());
    NodeConfiguration nodeConfiguration = loader.build();
    NodeMain nodeMain = null;

    try {
      nodeMain = (RosJasonNode)loader.loadClass(nodeClassName);
    } catch (ClassNotFoundException e) {
      throw new RosRuntimeException("Unable to locate node: " + nodeClassName,
                                    e);
    } catch (InstantiationException e) {
      throw new RosRuntimeException(
          "Unable to instantiate node: " + nodeClassName, e);
    } catch (IllegalAccessException e) {
      throw new RosRuntimeException(
          "Unable to instantiate node: " + nodeClassName, e);
    }

    Preconditions.checkState(nodeMain != null);
    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
    nodeMainExecutor.execute(nodeMain, nodeConfiguration);

    rosNode = (RosJasonNode)nodeMain;
    while (!rosNode.Connected())
      Thread.sleep(1000);
  }

  @Override
  public void reasoningCycleStarting() {
    List<jason_msgs.ActionStatus> actions_status = rosNode.retrieveStatus();
    for (jason_msgs.ActionStatus status : actions_status) {
      actionsStatus(status);
    }
    super.reasoningCycleStarting();
  }

  @Override
  public List<Literal> perceive() {
    List<Literal> per = new ArrayList<Literal>();
    // Map<String, Literal> perception = rosNode.getPerception();
    List<Literal> perception = rosNode.getPerception();
    if (!perception.isEmpty()) {
      per.addAll(perception);
    }
    return per;
  }

  @Override
  public void act(ActionExec action) {
    int seq = rosNode.publishAction(action);

    System.out.println(seq);
    // action.setResult(true);
    // actionExecuted(action);

    actionsWaiting.put(seq, action);
  }

  public void actionsStatus(jason_msgs.ActionStatus data) {
    int id = data.getId();
    boolean result = data.getResult();

    ActionExec action = actionsWaiting.get(id);

    if (action != null) {
        if (result) {
          action.setResult(true);
          actionExecuted(action);
        } else {
          action.setResult(false);
          actionExecuted(action);
        }
        actionsWaiting.remove(id);
    } else {
        System.out.println("Action not found.");
    }
  }

}
