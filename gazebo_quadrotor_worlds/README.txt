If you don't want gazebo to connect to http://gazebosim.org/models all the time to check for model xml information
make the following changes:

In simulator_gazebo/gazebo/scripts/setup.sh replace

export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models

with

if [ -z "$GAZEBO_MODEL_DATABASE_URI" ]; then
    export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models
fi 

This will make the script check if you have defined the environment variable GAZEBO_MODEL_DATABASE_URI
before settings its default value.

Next, in your .bashrc file add the following line where you do your ROS exports:

export GAZEBO_MODEL_DATABASE_URI=" "

You are giving a blank URI for the model database, so Gazebo will fail connecting to it and 
won't try to read xml data from the model server. Note that this way you can't download any models
automatically that you reference in a world! So make sure that you start gazebo with the default model
database path if you are about to use a new model for the first time so it can download it. Then after
you can put the blank URI back in.
