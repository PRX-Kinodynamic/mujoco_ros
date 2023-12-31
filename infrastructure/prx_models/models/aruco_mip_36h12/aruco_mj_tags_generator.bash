#!/bin/bash

aruco_path=${1}
imgs_dir="${aruco_path}/imgs"
imgs_tar="${imgs_dir}.tar.gz"
tags_directory="${aruco_path}/tags"

function generate_aruco_mj_xml () {
	tag_number=$1
	tag_name="aruco_mip_36h12_${tag_number}"
  filename="${tags_directory}/${tag_name}.xml"
  rm -f ${filename}
	cat << EOT >> ${filename}
<mujoco model="${tag_name}">
  <asset>
    <texture name="texture_material_${tag_name}" 
    	type="2d" 
    	content_type="image/png" 
    	file="${aruco_path}/imgs/${tag_name}.png"
    	width="1"
    	height="1"/>
    <material name="material_${tag_name}" 
    	texture="texture_material_${tag_name}" 
    	texrepeat="1 1" 
    	texuniform="false"/>
  </asset>
  <default>
    <default class="geom_${tag_name}">
      <geom 
      	type="box" 
      	contype="0"
      	conaffinity="0" 
      	condim="1"
      	size="0.167 0.167 0.01" 
      	material="material_${tag_name}"
      	euler="0 0 0" 
      	/>
    </default>
  </default>
</mujoco>
EOT
}

if [ -d "${imgs_dir}" ]; then
  echo "Images dir exists"
else
  echo "Extracting images"
  tar xf ${imgs_tar}
fi

if [ -d "${tags_directory}" ]; then
  echo "Aruco mujoco tags exists"
else
  echo "Generating aruco mujoco tags"
  mkdir ${tags_directory}
  for num in $(seq -f "%05g" 00 249); do
  	generate_aruco_mj_xml ${num}
  done
fi



