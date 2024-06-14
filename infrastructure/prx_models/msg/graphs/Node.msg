uint64 index				 							# id of the current node
uint64 parent 										# parent of the current node. Only used for Tree
uint64 parent_edge								# Edge id between the parent and this node. Only used for Tree
uint64[] children 								# Tree: Children of the current node. Graph: Edges out of this node
uint32 status 										# Extra field for status of the node
ml4kp_bridge/SpacePoint point 		# State associated to the current node 
float64 cost 											# Migration rule: bags/nodemsg_migration1.bmr 