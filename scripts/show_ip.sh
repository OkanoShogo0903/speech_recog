ip -4 a | grep -oP '(?<=inet\s)\d+(\.\d+){3}'
