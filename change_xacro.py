import xml.etree.cElementTree as ET
from robot_conf import mass, height, foot_length, shank_length, thigh_length, trunk_length
xacro_path = "./urdf/xacro/"
rt_xacro = xacro_path+"rt_model_fk.xacro"
lt_xacro = xacro_path+"lt_model_fk.xacro"
ce_xacro = xacro_path+"ce_model_fk.xacro"

tree_list = [ET.parse(ce_xacro), ET.parse(lt_xacro), ET.parse(rt_xacro)]
xacro_list = [ce_xacro, lt_xacro, rt_xacro]


namespaces = {"xacro":"http://www.ros.org/wiki/xacro"}
ET.register_namespace("xacro","http://www.ros.org/wiki/xacro")

for i in range(len(tree_list)):
    tree = tree_list[i]

    for p in tree.findall("xacro:property",namespaces=namespaces):
        name = p.attrib["name"]
        if name == "mass":
            p.set("value","{}".format(mass))
        elif name == "height":
            p.set("value","{}".format(height))
        elif name == "foot_length":
            p.set("value","{}".format(foot_length))
        elif name == "shank_length":
            p.set("value","{}".format(shank_length))
        elif name == "thigh_length":
            p.set("value","{}".format(thigh_length))
        elif name == "trunk_length":
            p.set("value","{}".format(trunk_length))

    tree.write(xacro_list[i],"UTF-8")