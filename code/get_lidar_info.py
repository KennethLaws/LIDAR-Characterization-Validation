#! /usr/bin/env python

import json
import sys
import socket
import urllib2

lidar_ip_address = '192.168.1.201'

request_url = 'http://%s/cgi/info.json' % lidar_ip_address
lidar_request = urllib2.Request(url = request_url)
response = urllib2.urlopen(lidar_request, None, 2)
lidar_response = json.loads(response.read())

lidar_hex_version = hex(int(lidar_response['appid']['bot']['version']))
lidar_serial = lidar_response['serial']
lidar_version = lidar_hex_version[2] + '.' + lidar_hex_version[3:5] + '.' + lidar_hex_version[5:7]
lidar_model = lidar_response['model']
print(lidar_serial)
print(lidar_version)
print(lidar_model)

