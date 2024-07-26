#!/usr/bin/env python
import yaml
import argparse
import math
import rospkg
import os

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def get_fname(fname):
    # Remove ".yaml" from the end of the filename
    return fname[:-5]

yaml_prefix = os.environ['DIRTMP_PATH'] + "/resources/input_files/environments/"
xml_prefix = rospkg.RosPack().get_path('prx_models') + '/models/obstacles/'
print(xml_prefix)

if __name__ == "__main__":
    argparse = argparse.ArgumentParser()
    argparse.add_argument('-f', '--file', help='YAML file to convert to XML', required=True)
    args = argparse.parse_args()

    with open(yaml_prefix + args.file, 'r') as stream:
        try:
            env_params = yaml.load(stream, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    xml_string = "<mujoco model = \"obstacles\">\n<compiler angle=\"radian\" />\n<worldbody>\n"
    
    try:
        for geom in env_params['environment']['geometries']:
            collision_geometry = geom['collision_geometry']
            name = geom['name']
            config = geom['config']
            if collision_geometry['type'] == 'box':
                print("Box geometry found with name ", name)
                dims = collision_geometry['dims']
                dims = [d/2.0 for d in dims]
                pos = config['position']
                quat = config['orientation']
                # Convert quaternion to euler angles
                euler = euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])
                xml_string += "<body name=\"obstacle_"+name+"\">\n"
                xml_string += "\t<geom name=\"obstacle_"+name+"\" type=\"box\" pos=\""+str(pos[0])+" "+str(pos[1])+" "+str(pos[2])+ \
                    "\" euler=\""+str(euler[0])+" "+str(euler[1])+" "+str(euler[1])+ \
                    "\" size=\""+str(dims[0])+" "+str(dims[1])+" "+str(dims[2])+"\" rgba=\"1.0 0.0 0.0 1\"/>\n"
                xml_string += "</body>\n"
            else:
                raise NotImplementedError("Only box geometries are supported at the moment")

        xml_string += "</worldbody>\n</mujoco>"

        with open(xml_prefix + get_fname(args.file) + ".xml", 'w') as f:
            f.write(xml_string)
        
    except KeyError:
        print("No geometries found in YAML file")
        exit(1)