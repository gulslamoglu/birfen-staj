#!/usr/bin/env python

import rospy
from my_string_concat_service.srv import ConcatenateStrings, ConcatenateStringsResponse

def concatenate_strings_server(req):
    result = req.string1 + req.string2
    print(f"Received request to concatenate: '{req.string1}' and '{req.string2}'")
    print(f"Sending response: '{result}'")
    return ConcatenateStringsResponse(result)

def main():
    rospy.init_node('concatenate_strings_server')
    service = rospy.Service('concatenate_strings', ConcatenateStrings, concatenate_strings_server)
    print("Ready to concatenate strings.")
    rospy.spin()

if __name__ == '__main__':
    main()
