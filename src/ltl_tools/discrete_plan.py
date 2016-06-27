# -*- coding: utf-8 -*-

from product import ProdAut_Run
from collections import defaultdict
from networkx import dijkstra_predecessor_and_distance
from ts import distance, reach_waypoint
import time 


#from gurobipy import *


#===========================================
#optimal initial synthesis
#===========================================
def dijkstra_plan_networkX(product, beta=10,start_set=None,pose=None,segment='lasso'):
	# requires a full construct of product automaton
	# start_set can be used to specify other initial states
	# 
	start = time.time()
	runs = {}
	loop = {}
	cycle = {}
	if start_set == None:
		init_set = product.graph['initial']
	else:
		init_set = start_set
	########################################
	# minimal circles
	for prod_target in product.graph['accept']:
		loop_pre, loop_dist = dijkstra_predecessor_and_distance(product, prod_target)
		for target_pred in product.predecessors_iter(prod_target):
			if target_pred in loop_dist:
				cycle[target_pred] = product.edge[target_pred][prod_target]["weight"] + loop_dist[target_pred]
		if cycle:
			opti_pred = min(cycle, key = cycle.get)
			suffix = compute_path_from_pre(loop_pre, opti_pred) + [prod_target,]
			loop[prod_target] = (cycle[opti_pred], suffix)
	########################################
	# shortest line
	for prod_init in init_set:
		line = {}
		line_pre, line_dist = dijkstra_predecessor_and_distance(product, prod_init)
		for target in loop.iterkeys():
			if target in line_dist:
				if pose:
					line_dist[target] += 0.1*distance(pose, prod_init[0][0])
				line[target] = line_dist[target]+beta*loop[target][0]
		if line:
			opti_targ = min(line, key = line.get)
			prefix = compute_path_from_pre(line_pre, opti_targ)
			precost = line_dist[opti_targ]
			runs[(prod_init, opti_targ)] = (prefix, precost, loop[opti_targ][1], loop[opti_targ][0])
	########################################
	# best combination
	if runs:
		if segment == 'lasso':
			prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1] + beta*p[3])
			run = ProdAut_Run(product, prefix, precost, suffix, sufcost, precost+beta*sufcost)
			#print '\n==================\n'
			print 'dijkstra_plan_networkX done within %.2fs: precost %.2f, sufcost %.2f' %(time.time()-start, precost, sufcost)
			return run, time.time()-start
			#print '\n==================\n'
		elif segment == 'prefix':
			prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1])
			run = ProdAut_Run(product, prefix, precost, suffix, sufcost, precost+beta*sufcost)
			#print '\n==================\n'
			print 'dijkstra_plan_networkX done within %.2fs: precost %.2f, sufcost %.2f' %(time.time()-start, precost, sufcost)
			return run, time.time()-start
			#print '\n==================\n'			
	else:
		print 'no accepting run found in optimal planning!'
		return None, None


def dijkstra_plan_optimal(product, beta=10, start_set=None, pose=None, time_limit=None, segment='lasso'):
	start = time.time()
	#print 'dijkstra plan optimal started!'
	runs = {}
	accept_set = product.graph['accept']
	if start_set == None:
		init_set = product.graph['initial']
	else:
		init_set = start_set
	#print 'number of accepting states %d' %(len(accept_set))
	#print 'number of initial states %d' %(len(init_set))
	loop_dict = {}
	for init_prod_node in init_set:
		for (prefix, precost) in dijkstra_targets(product, init_prod_node, accept_set):
			# for reachable accepting state
			# print 'accept node reached %s' %(str(prefix[-1]))
			if pose:
				precost += distance(pose, init_prod_node[0][0])
			if prefix[-1] in loop_dict:
				suffix, sufcost = loop_dict[prefix[-1]]
			else:
				suffix, sufcost = dijkstra_loop(product, prefix[-1])
				print suffix, sufcost
				loop_dict[prefix[-1]] = (suffix, sufcost)
			if suffix:
				runs[(prefix[0], prefix[-1])] = (prefix, precost, suffix, sufcost)
				#print 'find run from %s to %s and back' %(str(init_prod_node), str(prefix[-1]))
			if (time_limit) and (time.time()-start > time_limit):  # time limit has reached
				break
	if runs:
		if segment == 'lasso':
			prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1] + beta*p[3])
			run = ProdAut_Run(product, prefix, precost, suffix, sufcost, precost+beta*sufcost)
			#print '\n==================\n'
			print 'optimal_dijkstra_olf done within %.2fs: precost %.2f, sufcost %.2f' %(time.time()-start, precost, sufcost)
			return run, time.time()-start
		elif segment == 'prefix':
			prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1])
			run = ProdAut_Run(product, prefix, precost, suffix, sufcost, precost+beta*sufcost)
			#print '\n==================\n'
			print 'optimal_dijkstra_olf done within %.2fs: precost %.2f, sufcost %.2f' %(time.time()-start, precost, sufcost)
			return run, time.time()-start
	else:
		print 'no accepting run found in optimal planning!'
		return None, None


def dijkstra_targets(product, prod_source, prod_targets):
	# for product graph only, shortest path from source to a set of targets
	tovisit = set()
	visited = set()
	dist  = defaultdict(lambda: float('inf'))
	pre_node = {}
	dist[prod_source] = 0
	tovisit.add(prod_source)
	feasible_targets = set()
	for prod_accep in prod_targets:
		accept_pre_set = product.accept_predecessors(prod_accep)
		if accept_pre_set:
			feasible_targets.add(prod_accep)
	while (tovisit and feasible_targets):
		f_prod_node = min(tovisit, key=lambda n: dist[n])
		tovisit.remove(f_prod_node)
		visited.add(f_prod_node)
		d = dist[f_prod_node]
		for (t_prod_node, cost) in product.fly_successors_iter(f_prod_node):
		 	nd = d + cost 
		 	if nd < dist[t_prod_node]:
		 		dist[t_prod_node] = nd
		 		pre_node[t_prod_node] = [f_prod_node]
		 	if t_prod_node not in visited:
		 		tovisit.add(t_prod_node)
	 	if f_prod_node in feasible_targets:
	 	#print 'found path to buchi_target %s' %str(buchi_target)
	 		feasible_targets.remove(f_prod_node)
	 		yield compute_path_from_pre(pre_node, f_prod_node), dist[f_prod_node]


def dijkstra_loop(product, prod_accep):
	#print 'accept node check %s' %(str(prod_accep))
	paths = {}
	costs = {}
	accept_pre_set = product.accept_predecessors(prod_accep)
	for (tail, cost) in dijkstra_targets(product, prod_accep, accept_pre_set):
		if tail:
			accep_pre = tail[-1]
			#paths[accep_pre] = tail
			paths[accep_pre] = tail + [prod_accep,]
			costs[accep_pre] = cost + product.edge[accep_pre][prod_accep]['weight']
	if costs:
		min_pre = min(costs.keys(), key=lambda p: costs[p])
		min_loop =  paths[min_pre]
		return min_loop, costs[min_pre]
	else:
		return None, None


def compute_path_from_pre(pre, target):
	#print 'pre: %s with size %i' %(pre, len(pre))
	n = target
	path = [n]
	while n in pre:
		#print 'before append'
		#print 'now at node %s' %str(n)
		pn_list = pre[n]
		#print 'its pre_list %s' %str(pn_list)
		if not pn_list:
			break
		pn = pn_list[0]
		#print '[0] of pn_list %s' %str(pn)
		path.append(pn)
		#print 'path: %s' %path
		n = pn
	path.reverse()
	return path

#===========================================
# improve the current plan, given trace history
#===========================================
def prod_states_given_history(product, trace):
	if trace:
		S1 = set([(trace[0],p) for p in product.graph['buchi'].graph['initial']])
		for p in trace[1:]:
			S2 = set()
			for f_node in S1:
				for t_node in product.successors_iter(f_node):
					if t_node[0]==p:
						S2.add(t_node)
			S1 = S2.copy()
		S2 = set()
		for f_node in S1:
			S2 = S2.union(set([t_node for t_node in product.successors_iter(f_node)]))
		return S2
	else:
		return set([(p,q) for p in product.graph['ts'].nodes_iter() for q in product.graph['buchi'].graph['initial']])
	
		

def improve_plan_given_history(product, trace, pose=None, segment='lasso'):
	new_initial_set = prod_states_given_history(product, trace)
	if new_initial_set:
		#new_run, time=dijkstra_plan_optimal(product, 10, new_initial_set)
		new_run, time = dijkstra_plan_networkX(product, 0.1, new_initial_set, pose, segment)
		print 'Find better plans'
		return new_run
	else:
		print 'No better plans'
		return None

#===========================================
#local revision, in case of system update
#===========================================
def validate_and_revise_after_ts_change(run, product, sense_info, com_info):
	new_prefix = None
	new_suffix = None
	start = time.time()
	changed_regs = product.graph['ts'].graph['region'].update_after_region_change(sense_info, com_info)
	if changed_regs:
		for (index, prod_edge) in enumerate(run.pre_prod_edges):
			(f_ts_node, f_buchi_node) = prod_edge[0]
			(t_ts_node, t_buchi_node) = prod_edge[1] 
			succ_prod = set()
			for prod_node_to, weight in product.graph['ts'].fly_successors_iter(f_ts_node):
				succ_prod.add(prod_node_to)
			if t_ts_node not in succ_prod:
					print 'Oops, the current plan prefix contains invalid edges, need revision!'
					new_prefix = dijkstra_revise_once(product, run.prefix, index)
					break
		for (index, prod_edge) in enumerate(run.suf_prod_edges):
			(f_ts_node, f_buchi_node) = prod_edge[0]
			(t_ts_node, t_buchi_node) = prod_edge[1] 
			succ_prod = set()
			for prod_node_to, weight in product.graph['ts'].fly_successors_iter(f_ts_node):
				succ_prod.add(prod_node_to)
			if t_ts_node not in succ_prod:
					print 'Oops, the current plan suffix contains invalid edges, need revision!'
					new_prefix = dijkstra_revise_once(product, run.suffix, index)
					break
		if new_prefix or new_suffix:
			if new_prefix:
				run.prefix = new_prefix
			if new_suffix:
				run.suffix = new_suffix
			run.prod_run_to_prod_edges(product)
			run.output(product)
			print 'validate_and_revise_after_ts_change done in %.2fs' %(time.time()-start)
		else:
			print 'local revision failed'
			return False


def dijkstra_revise(product, run_segment, broken_edge_index):
	suf_segment = run_segment[(broken_edge_index+1):-1]
	for (bridge, cost) in dijkstra_targets(product, run_segment[broken_edge_index-1], suf_segment):
		run_segment_reversed = run_segment
		run_segment_reversed.reverse()
		index = run_segment_reversed.index(bridge[-1])
		index = len(run_segment)-index-1
		new_run_segment = run_segment[0:(broken_edge_index-1)] + bridge + run_segment[(index+1):-1]
		return new_run_segment


def dijkstra_revise_once(product, run_segment, broken_edge_index):
	for (bridge, cost) in dijkstra_targets(product, run_segment[broken_edge_index-1], set([run_segment[-1]])):
		new_run_segment = run_segment[0:(broken_edge_index-1)] + bridge
		return new_run_segment

#===========================================
# shortest path in ts
#===========================================

def shortest_path_ts(ts, f_ts_node, t_ts_node):
	if (f_ts_node not in ts) or (t_ts_node not in ts):
		print 'either nodes not in ts'
		return None, None
	else:
		path_pre, path_dist = dijkstra_predecessor_and_distance(ts, f_ts_node)
		if t_ts_node not in path_dist:
			print 'destination not reached'
		else:
			path = compute_path_from_pre(path_pre, t_ts_node)
			cost = path_dist[t_ts_node]
		return (path, cost)

def mip(request, Reply):
	# show the layout
	action_list = list(request.keys())
	agent_list = list(Reply.keys())
	print '************************'
	print 'action_d', str(action_list)
	for agent in agent_list:
		reply = Reply[agent]
		print  'agent %s: reply' %str(agent+1), reply
	print '************************'
	for action in action_list:
		if all(Reply[a][action][0]==False for a in agent_list):
			return None, None
			break
	try:
		# use gurobipy solver for mip
		# check http://www.gurobi.com/documentation/5.6/quick-start-guide/py_example_mip1_py
		M = len(Reply.keys())   #agents
		N = len(request.keys()) #action_d
		assign = defaultdict(lambda: [0,]*N)
		m = Model("assignment")
		# create variables
		for i in xrange(0,M):
			for j in xrange(0,N):
				assign[i][j] = m.addVar(vtype=GRB.BINARY, 
					name="b[%s][%s]"%(str(i),str(j)))
		m.update()
		# set objective
		obj = 0
		for i in xrange(0,M):
			for j in xrange(0,N):
				obj += (assign[i][j]*Reply[agent_list[i]][action_list[j]][0]*
						abs(Reply[agent_list[i]][action_list[j]][1]-request[action_list[j]]))
		m.setObjective(obj, GRB.MINIMIZE)
		# add constraints
		for i in xrange(0,M):
			constr = 0
			for j in xrange(0,N):
				constr += assign[i][j]*Reply[agent_list[i]][action_list[j]][0]
			m.addConstr(constr<=1,'row%s' %str(i))
		for j in xrange(0,N):
			constr = 0
			for i in xrange(0,M):
				constr += assign[i][j]*Reply[agent_list[i]][action_list[j]][0]
			m.addConstr(constr==1,'col%s' %str(j))
		# solve 
		m.optimize()
		for v in m.getVars():
			print v.varName, v.x
		print 'Obj:', m.objVal
		# send confirmation
		time_list1 =[assign[i][j].x*Reply[agent_list[i]][action_list[j]][0]*Reply[agent_list[i]][action_list[j]][1] 
					for i in xrange(0,M) for j in xrange(0,N)]
		time_list2 =[request[action_list[j]] for j in xrange(0,N)]
		time = max(time_list1+time_list2)
		#time = request[action_list[0]]-m.objVal
		Confirm = dict()
		for i in xrange(0,M):
			confirm = dict()
			for j in xrange(0,N):
				if assign[i][j].x:
					confirm[action_list[j]]= (assign[i][j].x, time)
				else:
					confirm[action_list[j]]= (assign[i][j].x, 0)
			Confirm[agent_list[i]]=confirm
		return Confirm, time 
	except GurobiError:
		print 'Error reported'	
		return None, None

def EnumerateAll(request, Reply):
	# show the layout
	# in case gurobi does not work
	action_list = list(request.keys())
	agent_list = list(Reply.keys())
	print '************************'
	print 'action_d', str(action_list)
	for agent in agent_list:
		reply = Reply[agent]
		print  'agent %s:' %agent, [reply[key] for key in action_list]
		print '\n'
	print '************************'
	M = len(Reply.keys())   #agents
	N = len(request.keys()) #action_d
	Confirm = dict()
	confrim = dict()
	if N == 0:
		print 'None action required!'
		return None, None
	elif N ==1:
		action = action_list[0]
		time = request[action]
		feasilbe_agent = [a for a in agent_list if Reply[a][action][0]]
		choosen_agent = min(feasilbe_agent, 
			key=lambda a: abs(Reply[a][action][1]-time) )
		appro_time = max(Reply[choosen_agent][action][1], time)
		confirm[action_list[0]] = (True, appro_time)
		Confirm[choosen_agent] = confirm
		# confirm
		confirm.clear()
		confirm[action] = (False, 0)
		for agent in agent_list:
			if agent != choosen_agent:
				Confirm[agent] = confirm.copy()
		return Confirm, appro_time
	elif N ==2:
		# action one 
		action_one = action_list[0]
		time_one = request[action_one]
		feasilbe_agent_one = [a for a in agent_list if Reply[a][action_one][0]]
		choosen_agent_one = min(feasilbe_agent_one, 
			key=lambda a: abs(Reply[a][action_one][1]-time_one))
		# action two 
		action_two = action_list[1]
		time_two = request[action_two]
		feasilbe_agent_two = [a for a in agent_list 
				if Reply[a][action_one][0] and (a != choosen_agent_one)]
		choosen_agent_two = min(feasilbe_agent_two, 
			key=lambda a: abs(Reply[a][action_two][1]-time_two))
		appro_time = max([Reply[choosen_agent_one][action_one][1],
				Reply[choosen_agent_two][action_two][1], time_one, time_two])
		for agent in agent_list:
			confirm.clear()
			if agent == choosen_agent_one:
				confirm[action_one] = (True, appro_time)
				confirm[action_two] = (False, 0)
			elif agent == choosen_agent_two:
				confirm[action_one] = (False, 0)
				confirm[action_two] = (True, appro_time)
			else:
				confirm[action_one] = (False, 0)
				confirm[action_two] = (False, 0)
			Confirm[agent] = confirm
		return Confirm, appro_time
	elif N ==3:
		# action one 
		action_one = action_list[0]
		time_one = request[action_one]
		feasilbe_agent_one = [a for a in agent_list if Reply[a][action_one][0]]
		choosen_agent_one = min(feasilbe_agent_one, 
			key=lambda a: abs(Reply[a][action_one][1]-time_one))
		# action two 
		action_two = action_list[1]
		time_two = request[action_two]
		feasilbe_agent_two = [a for a in agent_list 
				if Reply[a][action_two][0] and (a != choosen_agent_one)]
		choosen_agent_two = min(feasilbe_agent_two, 
			key=lambda a: abs(Reply[a][action_two][1]-time_two))
		# action three
		action_three = action_list[2]
		time_three = request[action_three]
		feasilbe_agent_three = [a for a in agent_list 
				if Reply[a][action_three][0] and 
				(a not in [choosen_agent_one, choosen_agent_two])]
		choosen_agent_three = min(feasilbe_agent_three, 
			key=lambda a: abs(Reply[a][action_three][1]-time_three))
		appro_time = max([Reply[choosen_agent_one][action_one][1],
				Reply[choosen_agent_two][action_two][1], 
				Reply[choosen_agent_three][action_three][1], 
				time_one, time_two, time_three])
		for agent in agent_list:
			confirm.clear()
			if agent == choosen_agent_one:
				confirm[action_one] = (True, appro_time)
				confirm[action_two] = (False, 0)
				confirm[action_three] =(False, 0)
			elif agent == choosen_agent_two:
				confirm[action_one] = (False, 0)
				confirm[action_two] = (True, appro_time)
				confirm[action_three] =(False, 0)
			elif agent == choosen_agent_three:
				confirm[action_one] = (False, 0)
				confirm[action_two] = (False, 0)
				confirm[action_three] = (True, appro_time)
			else:
				confirm[action_one] = (False, 0)
				confirm[action_two] = (False, 0)
				confirm[action_three] =(False, 0)
			Confirm[agent] = confirm
		return Confirm, appro_time
	elif N>=4:
		print 'we can not handle so many...'
		return None, None











	
		
