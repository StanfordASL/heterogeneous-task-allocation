import random
#import time
import argparse
import sys
import csv
import subprocess
#from threading import Timer

if __name__ == "__main__":

    dimension = [50]
    fleets = [32] #fleets = [2, 4, 8, 16, 32, 64, 128]
    time_horizon = [32] #time_horizon = [2, 4, 8, 16, 32, 64, 128]
    agents = 500
    intruders = 3
    repetitions = 1
    global_timeout = 600 #timeout in seconds

    cmd_milp = ['python3', 'milp_loader.py', '-p', '../../data/']
    cmd_cpp = ['../../cpp/build/mainbin', '../../data/']

    with open('../../data/'+'results.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['dimension', 'fleets', 'time_horizon', 'time_milp', 'time_cpp', 'val_milp', 'val_cpp'])

    for d in dimension:
        for f in fleets:
            outer_milp_timeout = True
            outer_cpp_timeout = False
            
            for t in time_horizon:
                t_milp = [ ]
                v_milp = [ ]
                t_cpp = [ ]
                v_cpp = [ ]

                milp_timeout = outer_milp_timeout
                cpp_timeout = outer_cpp_timeout
                for r in range(repetitions):
                    print(d,f,t,r)
                    
                    # generate data
                        
                    if not (milp_timeout and cpp_timeout):
                        cmd_data = ['python3', 'generate_input.py', '-d', str(d), '-f', str(f), '-t', str(t), '-a', str(agents), '-i', str(intruders), '-s', str(r), '-path', '../../data/']
                        subprocess.run(cmd_data)

                    if r == 0:
                        to = global_timeout
                    else:
                        to = None

                    # run milp solver
                    if not milp_timeout:
                        try:
                            print('running milp')
                            milp_result = subprocess.run(cmd_milp, timeout=to, capture_output=True)
                            output_split = milp_result.stdout.decode('utf-8').split('\n')

                            t_milp.append(float(output_split[-3].split(' ')[-1]))
                            v_milp.append(float(output_split[-2]))
                            
                        except subprocess.TimeoutExpired:
                            print('milp timeout')
                            milp_timeout = True
                            outer_milp_timeout = True

                    # run c++ solver
                    if not cpp_timeout:
                        try:
                            print('running cpp')
                            cpp_result = subprocess.run(cmd_cpp, timeout=to, capture_output=True)
                            output_split = cpp_result.stdout.decode('utf-8').split('\n')

                            t_cpp.append(float(output_split[-3].split(':')[-1]))
                            v_cpp.append(float(output_split[-2].split(':')[-1]))
                            
                        except subprocess.TimeoutExpired:
                            print('cpp timeout')
                            cpp_timeout = True
                            outer_cpp_timeout = True

                #print([t_milp,v_milp,t_cpp,v_cpp])

                # collect results milp
                final_t_milp = 'DNF'
                final_v_milp = 'DNF'
                
                if not milp_timeout:
                    final_t_milp = sum(t_milp) / repetitions
                    final_v_milp = sum(v_milp) / repetitions

                # collect results cpp
                final_t_cpp = 'DNF'
                final_v_cpp = 'DNF'
                
                if not cpp_timeout:
                    final_t_cpp = sum(t_cpp) / repetitions
                    final_v_cpp = sum(v_cpp) / repetitions

                with open('../../data/'+'results.csv', 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile, delimiter=',',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    writer.writerow([d, f, t, final_t_milp, final_t_cpp, final_v_milp, final_v_cpp])
