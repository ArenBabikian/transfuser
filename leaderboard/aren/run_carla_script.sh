# export CARLA_ROOT=${1:-/home/kchitta/Documents/CARLA_0.9.10.1}
# export WORK_DIR=${2:-/home/kchitta/Documents/transfuser}

export CARLA_ROOT=/home/ubuntu/git/transfuser/carla
export WORK_DIR=/home/ubuntu/git/transfuser

export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
# BELOW is commented because we wanna use the carla API downloaded by pip
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
# export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}
export PYTHONPATH="${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}


# python carla/PythonAPI/util/config.py -m /Game/Krisztina/Maps/Krisztina/Krisztina
# python carla/PythonAPI/util/config.py --list
# python carla/PythonAPI/util/config.py --inspect
python leaderboard/aren/pritnReplay.py --logpath /home/ubuntu/git/transfuser/leaderboard/aren/_res/0919-182808/RouteScenario_Town05_2actor_X_rep0.log


# python tools/map2Xodr.py


