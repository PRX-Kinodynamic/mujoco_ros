#!/usr/bin/env python
import yaml
import argparse
import math
import rospkg
import os

def init_yaml(filename):
    print(filename)
    with open(filename, 'r') as file:
        return yaml.safe_load(file)
    return None

def save_yaml(filename, yaml_params):
    with open(filename, 'w') as file:
        yaml.dump(yaml_params, file)

def set_environment_params(plant_file, experiments_file, experiment_id):
    
    experiment = None
    for e in experiments_file["experiments"]:
        if str(e["id"]) == experiment_id:
            experiment = e

    if plant_file["name"] == "fg_ltv_sde":
        plant_file["start_state"][0] = experiment["start"]["position"][0]
        plant_file["start_state"][1] = experiment["start"]["position"][1]

        plant_file["goal"]["state"][0] = experiment["goal"]["position"][0]
        plant_file["goal"]["state"][1] = experiment["goal"]["position"][1]

        plant_file["state_space"]["lower_bound"][0] = experiments_file["bounds"]["lower"][0]
        plant_file["state_space"]["lower_bound"][1] = experiments_file["bounds"]["lower"][1]

        plant_file["state_space"]["upper_bound"][0] = experiments_file["bounds"]["upper"][0]
        plant_file["state_space"]["upper_bound"][1] = experiments_file["bounds"]["upper"][1]

    if plant_file["name"] == "mushrFG":
        plant_file["start_state"][0] = experiment["start"]["position"][0]
        plant_file["start_state"][1] = experiment["start"]["position"][1]
        plant_file["start_state"][2] = experiment["start"]["orientation"]

        plant_file["goal"]["state"][0] = experiment["goal"]["position"][0]
        plant_file["goal"]["state"][1] = experiment["goal"]["position"][1]
        plant_file["goal"]["state"][2] = experiment["goal"]["orientation"]
        
        plant_file["state_space"]["lower_bound"][0] = experiments_file["bounds"]["lower"][0]
        plant_file["state_space"]["lower_bound"][1] = experiments_file["bounds"]["lower"][1]

        plant_file["state_space"]["upper_bound"][0] = experiments_file["bounds"]["upper"][0]
        plant_file["state_space"]["upper_bound"][1] = experiments_file["bounds"]["upper"][1]

    return plant_file

if __name__ == "__main__":
    argparse = argparse.ArgumentParser()
    argparse.add_argument('-p', '--plant', help='YAML file to convert to XML', required=True)
    argparse.add_argument('-e', '--experiments', help='YAML file to convert to XML', required=True)
    argparse.add_argument('-i', '--id', help='YAML file to convert to XML', required=True)
    argparse.add_argument('-o', '--out', help='YAML file to convert to XML', required=True)
    args = argparse.parse_args()

    plant_yaml = init_yaml(args.plant)
    experiments_yaml = init_yaml(args.experiments)
    
    out_yaml = set_environment_params(plant_yaml, experiments_yaml, args.id)

    save_yaml(args.out, out_yaml)
