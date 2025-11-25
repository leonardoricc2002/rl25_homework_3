#!/bin/bash

if [[ ( $@ == "--help") ||  $@ == "-h" ]]
then 
    echo "Usage: $0 [IMAGE_NAME] [CONTAINER_NAME] [FOLDER_NAME]"
else
    if [[ ($# -eq 0 ) ]]
    then 
        echo "No arguments specified, using wizard mode!"

        echo "Select the docker image ..."
        
        images_nr=$(docker image ls -a | wc -l)
        if [[ $images_nr == 1 ]]
        then
            echo "[ERROR] No docker images available, exiting..."
            exit 0
        fi
    
        i=1
        for c in $(docker image ls -a --format '{{.Repository}}')
        do
            echo $i $c
            i=$(( $i + 1 ))
        done

        read image_index
        i=1
        valid_index=0
        for c in $(docker image ls -a --format '{{.Repository}}')
        do
            if [[ $i -eq $image_index ]]
            then
                img_name=$c
                valid_index=1
            fi
            i=$(( $i + 1 ))
        done
        if [[ $valid_index -eq 0 ]]
        then
            echo "[ERROR] Index not valid, exiting ..."
            exit 0
        fi

        echo "You selected the docker image:" $img_name

        echo "Enter the container name ... [IMG_NAME_cont]"
        read container_name
        if [[ ($container_name == "") ]]
        then
            cont_name=$img_name"_cont"
        else
            cont_name=$container_name
        fi
        echo "The container name is:" $cont_name

        echo "Enter the folder name ... [~/IMG_NAME_fold]"
        read folder_name
        if [[ ($folder_name == "") ]]
        then
            fold_name=$img_name"_fold"
        else
            fold_name=$folder_name
        fi
        echo "The folder name is:" $fold_name

    else
        img_name=$1
        cont_name=$2
        fold_name=$3
    fi  

    if [ ! -d /home/$USER/$fold_name ] 
    then
        echo "[WARNING] devel folder doesn't exists, creating a new one"
        mkdir -p /home/$USER/$fold_name
        #exit 0
    fi

    xhost +local:root
    
    # MODIFICA APPLICATA QUI SOTTO: La destinazione è ora /ros2_ws/src
    docker run --privileged --rm -it --name=$cont_name --net=host --env="DISPLAY=$DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro" \
        --volume=/home/$USER/$fold_name:/ros2_ws/src \
        $img_name
        
    xhost -local:root
fi