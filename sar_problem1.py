# Recreation of sar_level1 problem 0 

import sys

sys.path.append('../')

import sar_domain

path_to_gym = "../../pddlgym"

the_domain = sar_domain.domain

rigid = sar_domain.empty_rigid()
initial = sar_domain.empty_initial()

rigid.types['location'] = ['f0-0f', 'f0-1f', 'f0-2f', 'f0-3f', 'f0-4f', 'f0-5f',
                           'f1-0f', 'f1-1f', 'f1-2f', 'f1-3f', 'f1-4f', 'f1-5f',
                           'f2-0f', 'f2-1f', 'f2-2f', 'f2-3f', 'f2-4f', 'f2-5f',
                           'f3-0f', 'f3-1f', 'f3-2f', 'f3-3f', 'f3-4f', 'f3-5f', 
                           'f4-0f', 'f4-1f', 'f4-2f', 'f4-3f', 'f4-4f', 'f4-5f', 
                           'f5-0f', 'f5-1f', 'f5-2f', 'f5-3f', 'f5-4f', 'f5-5f']

rigid.types['hospital'] = ['hospital0']
rigid.types['person'] = ['person0']
rigid.types['robot'] = ['robot0']
rigid.types['wall'] = ['wall2-2', 'wall2-3', 'wall2-4', 'wall4-0', 'wall4-2', 
                       'wall5-4']
rigid.types['chicken'] = ['chicken0', 'chicken1', 'chicken2', 'chicken3',
                          'chicken4']

clear_locs = ['f0-0f', 'f0-1f', 'f0-2f', 'f0-3f', 'f0-4f', 'f0-5f', 'f1-0f',
              'f1-2f', 'f1-3f', 'f1-4f', 'f1-5f', 'f2-0f', 'f2-1f', 'f2-5f',
              'f3-0f', 'f3-1f', 'f3-2f', 'f3-3f', 'f3-4f', 'f3-5f', 'f4-1f', 
              'f4-3f', 'f4-4f', 'f4-5f', 'f5-0f', 'f5-1f', 'f5-2f', 'f5-3f',
              'f5-5f']

for loc in rigid.types['location']:
    initial.clear[loc] = loc in clear_locs

connections = [('f0-0f', 'f0-1f', 'right'), ('f0-0f', 'f1-0f', 'down'),
               ('f0-1f', 'f0-0f', 'left'), ('f0-1f', 'f0-2f', 'right'),
               ('f0-1f', 'f1-1f', 'down'), ('f0-2f', 'f0-1f', 'left'),
               ('f0-2f', 'f0-3f', 'right'), ('f0-2f', 'f1-2f', 'down'),
               ('f0-3f', 'f0-2f', 'left'), ('f0-3f', 'f0-4f', 'right'),
               ('f0-3f', 'f1-3f', 'down'), ('f0-4f', 'f0-3f', 'left'),
               ('f0-4f', 'f0-5f', 'right'), ('f0-4f', 'f1-4f', 'down'),
               ('f0-5f', 'f0-4f', 'left'), ('f0-5f', 'f1-5f', 'down'),
               ('f1-0f', 'f0-0f', 'up'), ('f1-0f', 'f1-1f', 'right'),
               ('f1-0f', 'f2-0f', 'down'), ('f1-1f', 'f0-1f', 'up'),
               ('f1-1f', 'f1-0f', 'left'), ('f1-1f', 'f1-2f', 'right'),
               ('f1-1f', 'f2-1f', 'down'), ('f1-2f', 'f0-2f', 'up'), 
               ('f1-2f', 'f1-1f', 'left'), ('f1-2f', 'f1-3f', 'right'), 
			   ('f1-2f', 'f2-2f', 'down'), ('f1-3f', 'f0-3f', 'up'), 
			   ('f1-3f', 'f1-2f', 'left'), ('f1-3f', 'f1-4f', 'right'), 
			   ('f1-3f', 'f2-3f', 'down'), ('f1-4f', 'f0-4f', 'up'), 
			   ('f1-4f', 'f1-3f', 'left'), ('f1-4f', 'f1-5f', 'right'), 
			   ('f1-4f', 'f2-4f', 'down'), ('f1-5f', 'f0-5f', 'up'), 
			   ('f1-5f', 'f1-4f', 'left'), ('f1-5f', 'f2-5f', 'down'), 
			   ('f2-0f', 'f1-0f', 'up'), ('f2-0f', 'f2-1f', 'right'), 
			   ('f2-0f', 'f3-0f', 'down'), ('f2-1f', 'f1-1f', 'up'), 
			   ('f2-1f', 'f2-0f', 'left'), ('f2-1f', 'f2-2f', 'right'), 
			   ('f2-1f', 'f3-1f', 'down'), ('f2-2f', 'f1-2f', 'up'), 
			   ('f2-2f', 'f2-1f', 'left'), ('f2-2f', 'f2-3f', 'right'), 
			   ('f2-2f', 'f3-2f', 'down'), ('f2-3f', 'f1-3f', 'up'), 
			   ('f2-3f', 'f2-2f', 'left'), ('f2-3f', 'f2-4f', 'right'), 
			   ('f2-3f', 'f3-3f', 'down'), ('f2-4f', 'f1-4f', 'up'), 
			   ('f2-4f', 'f2-3f', 'left'), ('f2-4f', 'f2-5f', 'right'), 
			   ('f2-4f', 'f3-4f', 'down'), ('f2-5f', 'f1-5f', 'up'), 
			   ('f2-5f', 'f2-4f', 'left'), ('f2-5f', 'f3-5f', 'down'), 
			   ('f3-0f', 'f2-0f', 'up'), ('f3-0f', 'f3-1f', 'right'), 
			   ('f3-0f', 'f4-0f', 'down'), ('f3-1f', 'f2-1f', 'up'), 
			   ('f3-1f', 'f3-0f', 'left'), ('f3-1f', 'f3-2f', 'right'), 
			   ('f3-1f', 'f4-1f', 'down'), ('f3-2f', 'f2-2f', 'up'), 
			   ('f3-2f', 'f3-1f', 'left'), ('f3-2f', 'f3-3f', 'right'), 
			   ('f3-2f', 'f4-2f', 'down'), ('f3-3f', 'f2-3f', 'up'), 
			   ('f3-3f', 'f3-2f', 'left'), ('f3-3f', 'f3-4f', 'right'), 
			   ('f3-3f', 'f4-3f', 'down'), ('f3-4f', 'f2-4f', 'up'), 
			   ('f3-4f', 'f3-3f', 'left'), ('f3-4f', 'f3-5f', 'right'), 
			   ('f3-4f', 'f4-4f', 'down'), ('f3-5f', 'f2-5f', 'up'), 
			   ('f3-5f', 'f3-4f', 'left'), ('f3-5f', 'f4-5f', 'down'), 
			   ('f4-0f', 'f3-0f', 'up'), ('f4-0f', 'f4-1f', 'right'), 
			   ('f4-0f', 'f5-0f', 'down'), ('f4-1f', 'f3-1f', 'up'), 
			   ('f4-1f', 'f4-0f', 'left'), ('f4-1f', 'f4-2f', 'right'), 
			   ('f4-1f', 'f5-1f', 'down'), ('f4-2f', 'f3-2f', 'up'), 
			   ('f4-2f', 'f4-1f', 'left'), ('f4-2f', 'f4-3f', 'right'), 
			   ('f4-2f', 'f5-2f', 'down'), ('f4-3f', 'f3-3f', 'up'), 
			   ('f4-3f', 'f4-2f', 'left'), ('f4-3f', 'f4-4f', 'right'), 
			   ('f4-3f', 'f5-3f', 'down'), ('f4-4f', 'f3-4f', 'up'), 
			   ('f4-4f', 'f4-3f', 'left'), ('f4-4f', 'f4-5f', 'right'), 
			   ('f4-4f', 'f5-4f', 'down'), ('f4-5f', 'f3-5f', 'up'), 
			   ('f4-5f', 'f4-4f', 'left'), ('f4-5f', 'f5-5f', 'down'), 
			   ('f5-0f', 'f4-0f', 'up'), ('f5-0f', 'f5-1f', 'right'), 
			   ('f5-1f', 'f4-1f', 'up'), ('f5-1f', 'f5-0f', 'left'), 
			   ('f5-1f', 'f5-2f', 'right'), ('f5-2f', 'f4-2f', 'up'), 
			   ('f5-2f', 'f5-1f', 'left'), ('f5-2f', 'f5-3f', 'right'), 
			   ('f5-3f', 'f4-3f', 'up'), ('f5-3f', 'f5-2f', 'left'), 
			   ('f5-3f', 'f5-4f', 'right'), ('f5-4f', 'f4-4f', 'up'), 
			   ('f5-4f', 'f5-3f', 'left'), ('f5-4f', 'f5-5f', 'right'), 
			   ('f5-5f', 'f4-5f', 'up'), ('f5-5f', 'f5-4f', 'left')]

for start, dest, dir in connections:
    if not start in rigid.relations['connected'].keys():
        rigid.relations['connected'][start] = {}
    
    rigid.relations['connected'][start][dir] = dest

walls = [('wall2-2','f2-2f'), ('wall2-3','f2-3f'), ('wall2-4','f2-4f'), 
         ('wall4-0','f4-0f'), ('wall4-2','f4-2f'), ('wall5-4','f5-4f')]

for wall, loc in walls:
    rigid.relations['wall-at'][wall] = loc

initial.robot_at['robot0'] = 'f4-5f'
initial.person_at['person0'] = 'f5-2f'
initial.carrying['robot0'] = None
rigid.relations['chicken-at']['chicken0'] = 'f0-3f'
rigid.relations['chicken-at']['chicken1'] = 'f3-3f'
rigid.relations['chicken-at']['chicken2'] = 'f1-3f'
rigid.relations['chicken-at']['chicken3'] = 'f5-2f'
rigid.relations['chicken-at']['chicken4'] = 'f0-4f'

goal = [('person_at', 'person0', 'f5-5f')]

def main():
    sar_domain.rigid = rigid
    sar_domain.run_lookahead(initial, goal, 0, render=True)    

if __name__ == "__main__":
    main()
        