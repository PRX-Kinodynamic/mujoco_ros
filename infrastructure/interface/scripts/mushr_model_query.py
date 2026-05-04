#!/usr/bin/env python
import rospy
import numpy as np
import yaml
from std_msgs.msg import String

"""

MODEL (THIS FILE)     <------------------------>     EVALUATOR       <------> FILE

Start 
0. Init msg             -----------------------> Init  
  * topic_ns: namespace for msgs
  * data_set: Which data set to use
0.5 Receive Awk         <----------------------- Awk init
1. Ask for data         -----------------------> Batch size
2. Receive data         <----------------------- Respond with:
                                                  * Ids
                                                  * X0
                                                  * Ui's for each time step
2. Compute 
   trajectories         -----------------------> Evaluate   --------> Write to file
   Respond with:
     * Ids
     * Xi for each time step of the input
                                                (Go to 1)
                                                  (...)
                                                  (...)
N. Finish               -----------------------> Close file --------> close
END 
"""


class MushrModelQuery(object):
  def __init__(self):
    self.init_publisher = rospy.Publisher("/ModelEvaluation/initialize", String, queue_size=1, latch=True)
    # self.error_subscriber = rospy.Publisher("/ModelEvaluation/initialize", std_msgs.msg.String, queue_size=1, latch=True)
    self.error_subscriber = rospy.Subscriber("/ModelEvaluation/error", String, self.error_callback)

    self.evaluator_available = False
    self.evaluator_finished = False
    self.input_data = {}

    self.timer = rospy.Timer(rospy.Duration(1.0), self.run)

    topic_ns = '/mushr/evaluation'
    self.init_query(topic_ns)


  def init_query(self, topic_ns):
    init_msg = String();
    data = {'topic_ns': topic_ns, 'data_set': 'test'}
    ydata = yaml.dump(data)
    init_msg.data = str(ydata)

    request_topic = topic_ns + "/request" ;
    response_topic = topic_ns + "/response" ;
    evaluate_topic = topic_ns + "/evaluate" ;
    self.evaluate_publisher = rospy.Publisher(evaluate_topic, String, queue_size=1, latch=True)
    self.request_publisher = rospy.Publisher(request_topic, String, queue_size=1, latch=True)
    self.response_subscriber = rospy.Subscriber(response_topic, String, self.response_callback)
    self.init_publisher.publish(init_msg);

  def response_callback(self, msg):
    y_input = yaml.safe_load(msg.data)
    if y_input["header"] == "initialized":
      self.evaluator_available = True;
      print("Evaluator is available")
    elif y_input["header"] == "data":

      for idx in y_input:
        # print("idx", idx)
        if idx == "header":
          continue;
        if idx == "finished":
          self.evaluator_finished = bool(y_input["finished"])
          continue
        traj_id = int(idx)
        x0_ctrl = (y_input[traj_id]['x0'], y_input[traj_id]['controls'])
        self.input_data[traj_id] = x0_ctrl;
    else:
      print("Unknown type received", msg.data)

  def error_callback(self, msg):
    print("[Error] ", msg.data)

  def run(self, event):

    if not self.evaluator_available:
      return;

    if self.evaluator_finished:
      return;

    if len(self.input_data) == 0:
      data = {'batch_size': 10}
      ydata = yaml.dump(data)
      request_msg = String();
      request_msg.data = str(ydata)
      print(request_msg)
      self.request_publisher.publish(request_msg);
      return 

    all_trajs = {}
    # print("keys:", self.input_data.keys())
    for key in self.input_data.keys():
      # print("key", key)
      x0 = self.input_data[key][0]
      ui = self.input_data[key][1]


      output_traj = {}
      # Expects 11 states: (0.0, 0.1, ..., 1.0)
      # Here should be x_{t+1} = f(x_t, u_t)
      output_traj['trajectory'] = {'id':key}
      for u in ui: 
        output_traj['trajectory'][u] = x0
      output_traj['trajectory']['1.0'] = x0 

      all_trajs[key] = output_traj
      # all_trajs['trajectory'] = output_traj
    self.input_data = {}

    output_data = String()
    output_data.data = str(yaml.dump(all_trajs))
    self.evaluate_publisher.publish(output_data);


if __name__ == '__main__':
  try:
    rospy.init_node("MushrModelQuery")
    mushr_model_query = MushrModelQuery()


    rospy.spin();
    # keyboard_control_service.keyboard_terminal()
  except rospy.ROSInterruptException:
    pass