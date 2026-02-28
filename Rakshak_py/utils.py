import os,subprocess
 
import socket
import asyncio
import urllib.request
import re
import base64
import requests
import urllib3
 
import json

JETSON_IP=None
def get_lan_ip():  
    cmd = 'ifconfig wlan0 | head -2|tail -1|tr -s \" \"|cut -d \" \" -f3'   
    L = [S.strip('\n') for S in os.popen(cmd).readlines()]
    output=L[0]    
    return output
 
def get_eth_ip():  
    cmd = 'ifconfig eth0 | head -2|tail -1|tr -s \" \"|cut -d \" \" -f3'   
    L = [S.strip('\n') for S in os.popen(cmd).readlines()]
    output=L[0]    
    return output
 
def execute_cmd(cmd):
    #print(f'cmd={cmd}')
    L = [S.strip('\n') for S in os.popen(cmd).readlines()]
    #print(L)
    output=L[0]    
    return output

def getCamPorts():
    cam_port_dict={}
    try:
        
        cmd=f'./get_cam_ports.sh'
        result=subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        # Access the output
        json_output = result.stdout
        # Convert to Python dict
        cam_port_dict = json.loads(json_output)
        # print(f'type={type(cam_port_dict)}')
        error = result.stderr
        
        # Print or parse
        # print("Command Output:\n", cam_port_dict)
        return cam_port_dict
    except Exception as e:
        print(f"[ERROR] Failed to set permission on {dev}: {e}")
    return cam_port_dict

if __name__=='__main__':
    camera_dict = getCamPorts()
    # print(f'test---------->{camera_dict["USB3Neo_07703"]}')
    print(f'test---------->{camera_dict}')
 

# top_cam_serial -> USB3Neo_02286 down -> USB3Neo_09247