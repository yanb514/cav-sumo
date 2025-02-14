import re
import json
import os
import subprocess

with open('../config.json', 'r') as config_file:
    config = json.load(config_file)

computer_name = os.environ.get('HOSTNAME', 'Unknown')

if "CSI" in computer_name:
    SUMO_EXE = config['SUMO_PATH']["CSI"]
elif "VMS" in computer_name:
    SUMO_EXE = config['SUMO_PATH']["VMS"]
else: # run on SOL
    SUMO_EXE = config['SUMO_PATH']["SOL"]

def update_flows(penetration_rate, input_file="onramp_template.rou.xml", output_file="onramp.rou.xml"):
    with open(input_file, "r") as f:
        content = f.read()

    # Define replacement function with penetration rate
    def replace_flow(match):
        flow_id = match.group(1)
        total_veh = float(match.group(2))
        route = match.group(3)
        
        veh_cav = round(total_veh * penetration_rate)
        veh_hdv = round(total_veh - veh_cav)
        
        return (
            f'<flow id="{flow_id}_hdv" type="hdv" begin="0" end="300" vehsPerHour="{veh_hdv}" '
            f'route="{route}" departLane="random" departSpeed="desired"/>\n'
            f'    <flow id="{flow_id}_cav" type="cav" begin="0" end="300" vehsPerHour="{veh_cav}" '
            f'route="{route}" departLane="random" departSpeed="desired"/>'
        )

    # Use regex to find and replace flow definitions
    modified_content = re.sub(
        r'<flow id="(mainlane|ramp_0)" type="hdv" .*? vehsPerHour="(\d+)" route="(\w+)".*?/>',
        replace_flow,
        content,
        flags=re.DOTALL
    )

    with open(output_file, "w") as f:
        f.write(modified_content)



def run_sumo(sim_config, tripinfo_output=None, fcd_output=None):
    """Run a SUMO simulation with the given configuration."""
    # command = ['sumo', '-c', sim_config, '--tripinfo-output', tripinfo_output, '--fcd-output', fcd_output]

    command = [SUMO_EXE, '-c', sim_config, 
               '--no-step-log',  '--xml-validation', 'never', 
               '--lateral-resolution', '0.5']
    if tripinfo_output is not None:
        command.extend(['--tripinfo-output', tripinfo_output])
        
    if fcd_output is not None:
        command.extend([ '--fcd-output', fcd_output])
        
    subprocess.run(command, check=True)

if __name__ == "__main__":

    
    update_flows(penetration_rate=0.3)
    run_sumo("onramp.sumocfg", tripinfo_output=None, fcd_output="onramp_fcd")
    