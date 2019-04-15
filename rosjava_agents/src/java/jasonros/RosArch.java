package jasonros;

import jason.asSyntax.*;
import jason.architecture.*;
import jason.asSemantics.*;
import static jason.asSyntax.ASSyntax.*;
import jason.asSyntax.parser.ParseException;
import jason.RevisionFailedException;
import jason.asSyntax.Trigger.TEOperator;
import jason.asSyntax.Trigger.TEType;


import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import java.util.HashMap;
import java.util.Iterator;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.ros.exception.RosRuntimeException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import jason_msgs.ActionStatus;
import jason_msgs.Perception;

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
    NodeMain nodeMain = null;

    try {
      nodeMain = (jasonros.RosJasonNode)loader.loadClass(nodeClassName);
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

    rosNode = (jasonros.RosJasonNode)nodeMain;
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
  public List<Literal> perceive(){
    jason_msgs.Perception perception = rosNode.getPerception();
    while (perception!=null) {
        TransitionSystem ts = getTS();
        Literal bel = createLiteral(perception.getPerceptionName());
        bel.addAnnot(ts.getAg().getBB().TPercept);

        for(String param: perception.getParameters()){
            try{
                Term p = parseTerm(param);
                if(p.isVar()){
                    p = new StringTermImpl(param);
                }
                bel.addTerm(p);
            }catch(ParseException e){
                bel.addTerm(new StringTermImpl(param));
            }
        }

        boolean bufUpdate = perception.getUpdate();
        if(bufUpdate){
            Iterator<Literal> ibb = ts.getAg().getBB().getCandidateBeliefs(new PredicateIndicator(perception.getPerceptionName(),perception.getParameters().size()));
            boolean addBelief = true;
            while (ibb != null && ibb.hasNext()) {
                Literal l = ibb.next();
                if(l.equals(bel)){
                    addBelief = false;
                }else{
                   // try{
                   //     getTS().getAg().delBel(l);
                   // }catch(RevisionFailedException e){
                   //     System.out.println("Error adding new belief");
                   // }
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
            if(addBelief){
                try{
                    ts.getAg().addBel(bel);
                }catch(RevisionFailedException e){
                    System.out.println("Error adding new belief");
                }
            }
        }else{
            try{
                getTS().getAg().addBel(bel);
            }catch(RevisionFailedException e){
                System.out.println("Error adding new belief");
            }
        }
        perception = rosNode.getPerception();
    }
    return null;
  }

  @Override
  public void act(ActionExec action) {
    int seq = rosNode.publishAction(action);

    actionsWaiting.put(seq, action);
  }

  public void actionsStatus(jason_msgs.ActionStatus data) {
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
