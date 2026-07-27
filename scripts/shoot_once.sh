#!/usr/bin/env bash
set -e

python3 -c 'import socket,struct; p=b"{\"request\":\"send_to_agent\",\"app_api_level\":130,\"params\":{\"agent_id\":\"com.boosterobotics.default\"},\"agent_req\":{\"event\":\"on_component_click\",\"component_id\":\"shoot\",\"state\":0},\"app_platform\":\"iOS\"}"; print("payload length:",len(p)); s=socket.create_connection(("127.0.0.1",6868),3); s.sendall(struct.pack("<I",len(p))+p); print("sent"); s.close()'
