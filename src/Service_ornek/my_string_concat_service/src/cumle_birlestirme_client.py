#!/usr/bin/env python

import rospy
from my_string_concat_service.srv import ConcatenateStrings

def concatenate_strings_client(string1, string2):
    rospy.wait_for_service('concatenate_strings')
    try:
        concatenate_strings = rospy.ServiceProxy('concatenate_strings', ConcatenateStrings)
        response = concatenate_strings(string1, string2)
        return response.result
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return None

def main():
    rospy.init_node('concatenate_strings_client')
    string1 = "Hello, "
    string2 = "world!"
    result = concatenate_strings_client(string1, string2)
    if result is not None:
        print(f"Concatenated string: '{result}'")

if __name__ == '__main__':
    main()
