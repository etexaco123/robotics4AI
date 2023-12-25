#!/usr/bin/python
import rospy
import rospkg
import os

def replace_class(filename, dir, isSubBehaviour = False):
    rospack = rospkg.RosPack()

    if not os.path.exists(rospack.get_path("behaviours") + '/scripts/behaviours/' + dir):
        os.makedirs(rospack.get_path("behaviours") + '/scripts/behaviours/' + dir)
    # Check if file already exists
    if os.path.isfile(rospack.get_path("behaviours") + '/scripts/behaviours/' + dir + '/' + filename + ".py"):
        print filename + ' already exists.'
        return

    print 'Creating new behaviour file: ', filename
    
    if not isSubBehaviour:
        with open(rospack.get_path("behaviours") + '/scripts/template.py', 'r') as f:
            filedata = f.read()
    else:
        with open(rospack.get_path("behaviours") + '/scripts/template_subbehaviour.py', 'r') as f:
            filedata = f.read()
    filedata = filedata.replace("TEMP", filename)


    with open(rospack.get_path("behaviours") +'/scripts/behaviours/'  + dir + '/' + filename + ".py", "w") as f:
        f.write(filedata)

    with open(rospack.get_path("behaviours") + "/scripts/behaviours/" + dir + "/__init__.py", "w") as f:
	f.write(" ")


if __name__ == "__main__":
    rospy.init_node("create_behaviours")
    
    if not rospy.has_param("main_behaviour"):
        print 'No param found'
        exit(0)
    
    filename = rospy.get_param("main_behaviour")
    filename = str(filename)
    dir = filename
    replace_class(filename, dir)

    sub_behaviours = rospy.get_param("sub_behaviours", [])

    for s in sub_behaviours:
        filename = str(s)
        replace_class(filename, dir, True)
