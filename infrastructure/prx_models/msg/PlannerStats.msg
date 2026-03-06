float32 planned_duration
int32 iteration_count 
int32 total_nodes  
float32 cost_current_solution
float32 time_current_solution
int32 iters_current_solution

## Dirt-Specific
int32 random_f_rejected       # [Random] New nodes not added due to f(new_node) > f(current_sln)
int32 random_prunning         # [Random] New nodes not added due to dirt-pruning (using dir_radius)
int32 random_collision_check  # [Random] New nodes not added due to being in collision
int32 random_accepted         # [Random] New nodes accepted to be added to the tree (could be removed later)
int32 random_bnb              # [Random] Nodes deleted in a BNB operation when a new solution is found

int32 blossom_f_rejected       # [Blossom] New nodes not added due to f(new_node) > f(current_sln)
int32 blossom_prunning         # [Blossom] New nodes not added due to dirt-pruning (using dir_radius)
int32 blossom_collision_check  # [Blossom] New nodes not added due to being in collision
int32 blossom_accepted         # [Blossom] New nodes accepted to be added to the tree (could be removed later)
int32 blossom_bnb              # [Blossom] Nodes deleted in a BNB operation when a new solution is found

int32 solution_type

