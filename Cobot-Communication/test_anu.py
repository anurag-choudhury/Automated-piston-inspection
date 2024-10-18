import subprocess

# Replace the placeholder with the actual command
script = "bash -c 'source ./../devel/setup.bash && rosrun demo demo_leave_listen_node'"

# Execute the command using subprocess
subprocess.run(script, shell=True)
