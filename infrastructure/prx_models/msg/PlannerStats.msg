float32 total_planning_time
int64 total_iterations
int64 total_nodes

# Solution-specific stats
int8 solution_found

# Current solution stats
float32 current_solution_cost
float32 current_solution_time
int64 current_solution_iterations

# First solution stats
float32 first_solution_cost
float32 first_solution_time
int64 first_solution_iterations

## Dirt-Specific
float32 best_f_value 

int32 random_g_rejected       # [Random] New nodes not added due to g(new_node) > g(current_sln)
int32 random_prunning         # [Random] New nodes not added due to dirt-pruning (using dir_radius)
int32 random_collision_check  # [Random] New nodes not added due to being in collision
int32 random_accepted         # [Random] New nodes accepted to be added to the tree (could be removed later)
int32 random_bnb              # [Random] Nodes deleted in a BNB operation when a new solution is found

int32 blossom_g_rejected       # [Blossom] New nodes not added due to f(new_node) > f(current_sln)
int32 blossom_prunning         # [Blossom] New nodes not added due to dirt-pruning (using dir_radius)
int32 blossom_collision_check  # [Blossom] New nodes not added due to being in collision
int32 blossom_accepted         # [Blossom] New nodes accepted to be added to the tree (could be removed later)
int32 blossom_bnb              # [Blossom] Nodes deleted in a BNB operation when a new solution is found

int32 solution_type

