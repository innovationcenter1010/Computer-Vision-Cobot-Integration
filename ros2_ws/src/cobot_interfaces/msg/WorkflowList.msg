# Latched on /task/workflows by task_manager: every registered workflow in
# dashboard order, with its current readiness. Republished when readiness
# changes and after each run.
std_msgs/Header header
WorkflowInfo[] workflows
string default_workflow   # what /task/start (Trigger) and an empty id run
