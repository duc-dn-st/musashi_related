#!/usr/bin/expect -f

# Get a Bash shell
spawn -noecho bash

# Wait for a prompt
expect "$ "

# Type something
send "bash start_controller_mode.sh"

# Hand over control to the user
interact

exit
