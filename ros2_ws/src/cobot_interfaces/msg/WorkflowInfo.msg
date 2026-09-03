# One registered task_manager workflow (see cobot_tasks/workflows/).
string id                 # StartWorkflow.workflow / TaskStatus.active_task
string label              # human name for the dashboard
string description
bool ready                # check_ready passed when the list was published
string not_ready_reason   # why not, when ready == false
uint8 requires            # prerequisite capability bitmask (SystemState.CAP_*)
string requires_name      # "see" / "move" / "operate" for the dashboard badge
